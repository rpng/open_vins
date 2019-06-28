#include "TrackKLT.h"


using namespace ov_core;


void TrackKLT::feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Histogram equalize
    cv::equalizeHist(img, img);

    // If we didn't have any successful tracks last time, just extract this time
    // This also handles, the tracking initalization on the first call to this extractor
    if(pts_last[cam_id].empty()) {
        perform_detection_monocular(img, pts_last[cam_id], ids_last[cam_id]);
        img_last[cam_id] = img.clone();
        return;
    }

    // First we should make that the last images have enough features so we can do KLT
    // This will "top-off" our number of tracks so always have a constant number
    perform_detection_monocular(img_last[cam_id], pts_last[cam_id], ids_last[cam_id]);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    //===================================================================================
    //===================================================================================

    // Debug
    //ROS_INFO("current points = %d,%d",(int)pts_left_last.size(),(int)pts_right_last.size());

    // Our return success masks, and predicted new features
    std::vector<uchar> mask_ll;
    std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id];

    // Lets track temporally
    perform_matching(img_last[cam_id],img,pts_last[cam_id],pts_left_new,cam_id,cam_id,mask_ll);
    rT3 =  boost::posix_time::microsec_clock::local_time();

    //===================================================================================
    //===================================================================================

    // If any of our mask is empty, that means we didn't have enough to do ransac, so just return
    if(mask_ll.empty()) {
        img_last[cam_id] = img.clone();
        pts_last[cam_id].clear();
        ids_last[cam_id].clear();
        ROS_ERROR("[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....");
        return;
    }

    // Get our "good tracks"
    std::vector<cv::KeyPoint> good_left;
    std::vector<size_t> good_ids_left;

    // Loop through all left points
    for(size_t i=0; i<pts_left_new.size(); i++) {
        // Ensure we do not have any bad KLT tracks (i.e., points are negative)
        if(pts_left_new[i].pt.x < 0 || pts_left_new[i].pt.y < 0)
            continue;
        // If it is a good track, and also tracked from left to right
        if(mask_ll[i]) {
            good_left.push_back(pts_left_new[i]);
            good_ids_left.push_back(ids_last[cam_id][i]);
        }
    }


    //===================================================================================
    //===================================================================================


    // Update our feature database, with theses new observations
    for(size_t i=0; i<good_left.size(); i++) {
        cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id);
        database->update_feature(good_ids_left.at(i), timestamp, cam_id,
                                 good_left.at(i).pt.x, good_left.at(i).pt.y,
                                 npt_l.x, npt_l.y);
    }

    // Move forward in time
    img_last[cam_id] = img.clone();
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
    rT5 =  boost::posix_time::microsec_clock::local_time();

    // Timing information
    //ROS_INFO("[TIME-KLT]: %.4f seconds for detection",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for temporal klt",(rT3-rT2).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for feature DB update (%d features)",(rT5-rT3).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-KLT]: %.4f seconds for total",(rT5-rT1).total_microseconds() * 1e-6);


}


void TrackKLT::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Histogram equalize
    cv::Mat img_left, img_right;
    cv::equalizeHist(img_leftin, img_left);
    cv::equalizeHist(img_rightin, img_right);

    // If we didn't have any successful tracks last time, just extract this time
    // This also handles, the tracking initalization on the first call to this extractor
    if(pts_last[cam_id_left].empty() || pts_last[cam_id_right].empty()) {
        perform_detection_stereo(img_left, img_right, pts_last[cam_id_left], pts_last[cam_id_right], ids_last[cam_id_left], ids_last[cam_id_right]);
        img_last[cam_id_left] = img_left.clone();
        img_last[cam_id_right] = img_right.clone();
        return;
    }


    // First we should make that the last images have enough features so we can do KLT
    // This will "top-off" our number of tracks so always have a constant number
    perform_detection_stereo(img_last[cam_id_left], img_last[cam_id_right],
                             pts_last[cam_id_left], pts_last[cam_id_right],
                             ids_last[cam_id_left], ids_last[cam_id_right]);
    rT2 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================


    // Debug
    //ROS_INFO("current points = %d,%d",(int)pts_left_last.size(),(int)pts_right_last.size());

    // Our return success masks, and predicted new features
    std::vector<uchar> mask_ll, mask_rr;
    std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id_left];
    std::vector<cv::KeyPoint> pts_right_new = pts_last[cam_id_right];

    // Lets track temporally
    boost::thread t_ll = boost::thread(&TrackKLT::perform_matching, this, boost::cref(img_last[cam_id_left]), boost::cref(img_left),
                                       boost::ref(pts_last[cam_id_left]), boost::ref(pts_left_new), cam_id_left, cam_id_left, boost::ref(mask_ll));
    boost::thread t_rr = boost::thread(&TrackKLT::perform_matching, this, boost::cref(img_last[cam_id_right]), boost::cref(img_right),
                                       boost::ref(pts_last[cam_id_right]), boost::ref(pts_right_new), cam_id_right, cam_id_right, boost::ref(mask_rr));

    // Wait till both threads finish
    t_ll.join();
    t_rr.join();
    rT3 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================


    // left to right matching
    std::vector<uchar> mask_lr;
    perform_matching(img_left, img_right, pts_left_new, pts_right_new, cam_id_left, cam_id_right, mask_lr);
    rT4 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If any of our masks are empty, that means we didn't have enough to do ransac, so just return
    if(mask_ll.empty() || mask_rr.empty() || mask_lr.empty()) {
        img_last[cam_id_left] = img_left.clone();
        img_last[cam_id_right] = img_right.clone();
        pts_last[cam_id_left].clear();
        pts_last[cam_id_right].clear();
        ids_last[cam_id_left].clear();
        ids_last[cam_id_right].clear();
        ROS_ERROR("[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....");
        return;
    }


    // Get our "good tracks"
    std::vector<cv::KeyPoint> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;


    // Loop through all left points
    for(size_t i=0; i<pts_left_new.size(); i++) {
        // Ensure we do not have any bad KLT tracks (i.e., points are negative)
        if(pts_left_new[i].pt.x < 0 || pts_left_new[i].pt.y < 0 || pts_right_new[i].pt.x < 0 || pts_right_new[i].pt.y < 0)
            continue;
        // If it is a good track, and also tracked from left to right
        if(mask_ll[i] && mask_rr[i] && mask_lr[i]) {
            good_left.push_back(pts_left_new[i]);
            good_right.push_back(pts_right_new[i]);
            good_ids_left.push_back(ids_last[cam_id_left][i]);
            good_ids_right.push_back(ids_last[cam_id_right][i]);
        }
        // We can also track mono features, so lets handle them here
        //if(mask_ll[i]) {
        //    good_left.push_back(pts_left_new[i]);
        //    good_ids_left.push_back(ids_left_last[i]);
        //}
    }


    //===================================================================================
    //===================================================================================


    // Update our feature database, with theses new observations
    for(size_t i=0; i<good_left.size(); i++) {
        // Assert that our IDs are the same (i.e., stereo )
        assert(good_ids_left.at(i)==good_ids_right.at(i));
        // Try to undistort the point
        cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id_left);
        cv::Point2f npt_r = undistort_point(good_right.at(i).pt, cam_id_right);
        // Append to the database
        database->update_feature(good_ids_left.at(i), timestamp, cam_id_left,
                                 good_left.at(i).pt.x, good_left.at(i).pt.y,
                                 npt_l.x, npt_l.y);
        database->update_feature(good_ids_left.at(i), timestamp, cam_id_right,
                                 good_right.at(i).pt.x, good_right.at(i).pt.y,
                                 npt_r.x, npt_r.y);
    }


    // Move forward in time
    img_last[cam_id_left] = img_left.clone();
    img_last[cam_id_right] = img_right.clone();
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    rT5 =  boost::posix_time::microsec_clock::local_time();


    // Timing information
    //ROS_INFO("[TIME-KLT]: %.4f seconds for detection",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for temporal klt",(rT3-rT2).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for stereo klt",(rT4-rT3).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for feature DB update (%d features)",(rT5-rT4).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-KLT]: %.4f seconds for total",(rT5-rT1).total_microseconds() * 1e-6);


}

void TrackKLT::perform_detection_monocular(const cv::Mat &img0, std::vector<cv::KeyPoint> &pts0, std::vector<size_t> &ids0) {

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d_current;
    grid_2d_current.resize((int)(img0.cols/min_px_dist)+10,(int)(img0.rows/min_px_dist)+10);
    auto it0 = pts0.begin();
    auto it2 = ids0.begin();
    while(it0 != pts0.end()) {
        // Get current left keypoint
        cv::KeyPoint kpt = *it0;
        // Check if this keypoint is near another point
        if(grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1) {
            it0 = pts0.erase(it0);
            it2 = ids0.erase(it2);
            continue;
        }
        // Else we are good, move forward to the next point
        grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
        it0++;
        it2++;
    }

    // First compute how many more features we need to extract from this image
    int num_featsneeded = num_features - (int)pts0.size();

    // If we don't need any features, just return
    if(num_featsneeded < 1)
        return;

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_FAST::perform_griding(img0, pts0_ext, num_featsneeded, grid_x, grid_y, threshold, true);

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d;
    grid_2d.resize((int)(img0.cols/min_px_dist)+10,(int)(img0.rows/min_px_dist)+10);
    for(auto& kpt : pts0) {
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for(auto& kpt : pts0_ext) {
        // See if there is a point at this location
        if(grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1)
            continue;
        // Else lets add it!
        kpts0_new.push_back(kpt);
        pts0_new.push_back(kpt.pt);
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // Loop through and record only ones that are valid
    for(size_t i=0; i<pts0_new.size(); i++) {
        // update the uv coordinates
        kpts0_new.at(i).pt = pts0_new.at(i);
        // append the new uv coordinate
        pts0.push_back(kpts0_new.at(i));
        // move id foward and append this new point
        currid++;
        ids0.push_back(currid);
    }

}


void TrackKLT::perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1,
                                        std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1,
                                        std::vector<size_t> &ids0, std::vector<size_t> &ids1) {

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d_current;
    grid_2d_current.resize((int)(img0.cols/min_px_dist)+10,(int)(img0.rows/min_px_dist)+10);
    auto it0 = pts0.begin();
    auto it1 = pts1.begin();
    auto it2 = ids0.begin();
    auto it3 = ids1.begin();
    while(it0 != pts0.end()) {
        // Get current left keypoint
        cv::KeyPoint kpt = *it0;
        // Check if this keypoint is near another point
        if(grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1) {
            it0 = pts0.erase(it0);
            it1 = pts1.erase(it1);
            it2 = ids0.erase(it2);
            it3 = ids1.erase(it3);
            continue;
        }
        // Else we are good, move forward to the next point
        grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
        it0++;
        it1++;
        it2++;
        it3++;
    }

    // First compute how many more features we need to extract from this image
    int num_featsneeded = num_features - (int)pts0.size();

    // If we don't need any features, just return
    if(num_featsneeded < 1)
        return;

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_FAST::perform_griding(img0, pts0_ext, num_featsneeded, grid_x, grid_y, threshold, true);


    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d;
    grid_2d.resize((int)(img0.cols/min_px_dist)+10,(int)(img0.rows/min_px_dist)+10);
    for(auto& kpt : pts0) {
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for(auto& kpt : pts0_ext) {
        // See if there is a point at this location
        if(grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1)
            continue;
        // Else lets add it!
        kpts0_new.push_back(kpt);
        pts0_new.push_back(kpt.pt);
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // TODO: Project points from the left frame into the right frame
    // TODO: this will not work for large baseline systems.....
    std::vector<cv::KeyPoint> kpts1_new;
    std::vector<cv::Point2f> pts1_new;
    kpts1_new = kpts0_new;
    pts1_new = pts0_new;

    // If we don't have any new points, just return
    if(pts0_new.empty())
        return;

    // Now do KLT tracking to get the valid projections
    // Note: we have a pretty big window size here since our projection might be bad
    // Note: but this might cause failure in cases of repeated textures (eg. checkerboard)
    int pyr_levels = 3;
    std::vector<uchar> mask;
    std::vector<float> error;
    cv::Size win_size(15, 15);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(img0, img1, pts0_new, pts1_new, mask, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

    // Loop through and record only ones that are valid
    for(size_t i=0; i<pts0_new.size(); i++) {
        if(mask[i] == 1) {
            // update the uv coordinates
            kpts0_new.at(i).pt = pts0_new.at(i);
            kpts1_new.at(i).pt = pts1_new.at(i);
            // append the new uv coordinate
            pts0.push_back(kpts0_new.at(i));
            pts1.push_back(kpts1_new.at(i));
            // move id foward and append this new point
            currid++;
            ids0.push_back(currid);
            ids1.push_back(currid);
        }
    }

}


void TrackKLT::perform_matching(const cv::Mat& img0, const cv::Mat& img1,
                                std::vector<cv::KeyPoint>& kpts0, std::vector<cv::KeyPoint>& kpts1,
                                size_t id0, size_t id1,
                                std::vector<uchar>& mask_out) {

    // We must have equal vectors
    assert(kpts0.size() == kpts1.size());

    // Return if we don't have any points
    if(kpts0.empty() || kpts1.empty())
        return;

    // Convert keypoints into points (stupid opencv stuff)
    std::vector<cv::Point2f> pts0, pts1;
    for(size_t i=0; i<kpts0.size(); i++) {
        pts0.push_back(kpts0.at(i).pt);
        pts1.push_back(kpts1.at(i).pt);
    }

    // If we don't have enough points for ransac just return empty
    // We set the mask to be all zeros since all points failed RANSAC
    if(pts0.size() < 10) {
        for(size_t i=0; i<pts0.size(); i++)
            mask_out.push_back((uchar)0);
        return;
    }

    // Now do KLT tracking to get the valid new points
    int pyr_levels = 3;
    std::vector<uchar> mask_klt;
    std::vector<float> error;
    cv::Size win_size(11, 11);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.01);
    cv::calcOpticalFlowPyrLK(img0, img1, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);


    // Normalize these points, so we can then do ransac
    // We don't want to do ransac on distorted image uvs since the mapping is nonlinear
    std::vector<cv::Point2f> pts0_n, pts1_n;
    for(size_t i=0; i<pts0.size(); i++) {
        pts0_n.push_back(undistort_point(pts0.at(i),id0));
        pts1_n.push_back(undistort_point(pts1.at(i),id1));
    }

    // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
    std::vector<uchar> mask_rsc;
    double max_focallength_img0 = std::max(camera_k_OPENCV.at(id0)(0,0),camera_k_OPENCV.at(id0)(1,1));
    double max_focallength_img1 = std::max(camera_k_OPENCV.at(id1)(0,0),camera_k_OPENCV.at(id1)(1,1));
    double max_focallength = std::max(max_focallength_img0,max_focallength_img1);
    cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 1/max_focallength, 0.999, mask_rsc);


    // Loop through and record only ones that are valid
    for(size_t i=0; i<mask_klt.size(); i++) {
        auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i])? 1 : 0);
        mask_out.push_back(mask);
    }

    // Copy back the updated positions
    for(size_t i=0; i<pts0.size(); i++) {
        kpts0.at(i).pt = pts0.at(i);
        kpts1.at(i).pt = pts1.at(i);
    }

}



