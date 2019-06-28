#include "TrackAruco.h"


using namespace ov_core;


void TrackAruco::feed_monocular(double timestamp, cv::Mat &imgin, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Histogram equalize
    cv::Mat img;
    cv::equalizeHist(imgin, img);

    // Clear the old data from the last timestep
    ids_aruco[cam_id].clear();
    corners[cam_id].clear();
    rejects[cam_id].clear();

    // If we are downsizing, then downsize
    cv::Mat img0;
    if(do_downsizing) {
        cv::pyrDown(img,img0,cv::Size(img.cols/2,img.rows/2));
    } else {
        img0 = img;
    }


    //===================================================================================
    //===================================================================================

    // Perform extraction
    cv::aruco::detectMarkers(img0,aruco_dict,corners[cam_id],ids_aruco[cam_id],aruco_params,rejects[cam_id]);
    rT2 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If we downsized, scale all our u,v measurements by a factor of two
    // Note: we do this so we can use these results for visulization later
    // Note: and so that the uv added is in the same image size
    if(do_downsizing) {
        for(size_t i=0; i<corners[cam_id].size(); i++) {
            for(size_t j=0; j<corners[cam_id].at(i).size(); j++) {
                corners[cam_id].at(i).at(j).x *= 2;
                corners[cam_id].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id].size(); i++) {
            for(size_t j=0; j<rejects[cam_id].at(i).size(); j++) {
                rejects[cam_id].at(i).at(j).x *= 2;
                rejects[cam_id].at(i).at(j).y *= 2;
            }
        }
    }


    //===================================================================================
    //===================================================================================


    // ID vectors, of all currently tracked IDs
    std::vector<size_t> ids_new;

    // Append to our feature database this new information
    for(size_t i=0; i<ids_aruco[cam_id].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id].at(i).size()==4);
        // Try to undistort the point
        cv::Point2f npt_l = undistort_point(corners[cam_id].at(i).at(0), cam_id);
        // Append to the ids vector and database
        ids_new.push_back((size_t)ids_aruco[cam_id].at(i));
        database->update_feature((size_t)ids_aruco[cam_id].at(i), timestamp, cam_id,
                                 corners[cam_id].at(i).at(0).x, corners[cam_id].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }


    // Move forward in time
    img_last[cam_id] = img.clone();
    ids_last[cam_id] = ids_new;
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // Timing information
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for detection",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for feature DB update (%d features)",(rT3-rT2).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for total",(rT3-rT1).total_microseconds() * 1e-6);

}


void TrackAruco::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Histogram equalize
    cv::Mat img_left, img_right;
    cv::equalizeHist(img_leftin, img_left);
    cv::equalizeHist(img_rightin, img_right);

    // Clear the old data from the last timestep
    ids_aruco[cam_id_left].clear();
    ids_aruco[cam_id_right].clear();
    corners[cam_id_left].clear();
    corners[cam_id_right].clear();
    rejects[cam_id_left].clear();
    rejects[cam_id_right].clear();

    // If we are downsizing, then downsize
    cv::Mat img0, img1;
    if(do_downsizing) {
        cv::pyrDown(img_left,img0,cv::Size(img_left.cols/2,img_left.rows/2));
        cv::pyrDown(img_right,img1,cv::Size(img_right.cols/2,img_right.rows/2));
    } else {
        img0 = img_left;
        img1 = img_right;
    }


    //===================================================================================
    //===================================================================================

    // Perform extraction (doing this is parallel is actually slower on my machine -pgeneva)
    cv::aruco::detectMarkers(img0,aruco_dict,corners[cam_id_left],ids_aruco[cam_id_left],aruco_params,rejects[cam_id_left]);
    cv::aruco::detectMarkers(img1,aruco_dict,corners[cam_id_right],ids_aruco[cam_id_right],aruco_params,rejects[cam_id_right]);
    rT2 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If we downsized, scale all our u,v measurements by a factor of two
    // Note: we do this so we can use these results for visulization later
    // Note: and so that the uv added is in the same image size
    if(do_downsizing) {
        for(size_t i=0; i<corners[cam_id_left].size(); i++) {
            for(size_t j=0; j<corners[cam_id_left].at(i).size(); j++) {
                corners[cam_id_left].at(i).at(j).x *= 2;
                corners[cam_id_left].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<corners[cam_id_right].size(); i++) {
            for(size_t j=0; j<corners[cam_id_right].at(i).size(); j++) {
                corners[cam_id_right].at(i).at(j).x *= 2;
                corners[cam_id_right].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id_left].size(); i++) {
            for(size_t j=0; j<rejects[cam_id_left].at(i).size(); j++) {
                rejects[cam_id_left].at(i).at(j).x *= 2;
                rejects[cam_id_left].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id_right].size(); i++) {
            for(size_t j=0; j<rejects[cam_id_right].at(i).size(); j++) {
                rejects[cam_id_right].at(i).at(j).x *= 2;
                rejects[cam_id_right].at(i).at(j).y *= 2;
            }
        }
    }


    //===================================================================================
    //===================================================================================


    // ID vectors, of all currently tracked IDs
    std::vector<size_t> ids_left_new, ids_right_new;

    // Append to our feature database this new information
    for(size_t i=0; i<ids_aruco[cam_id_left].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id_left].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id_left].at(i).size()==4);
        // Try to undistort the point (could fail undistortion!)
        cv::Point2f npt_l = undistort_point(corners[cam_id_left].at(i).at(0), cam_id_left);
        // Append to the ids vector and database
        ids_left_new.push_back((size_t)ids_aruco[cam_id_left].at(i));
        database->update_feature((size_t)ids_aruco[cam_id_left].at(i), timestamp, cam_id_left,
                                 corners[cam_id_left].at(i).at(0).x, corners[cam_id_left].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }
    for(size_t i=0; i<ids_aruco[cam_id_right].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id_right].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id_right].at(i).size()==4);
        // Try to undistort the point (could fail undistortion!)
        cv::Point2f npt_l = undistort_point(corners[cam_id_right].at(i).at(0), cam_id_right);
        // Append to the ids vector and database
        ids_right_new.push_back((size_t)ids_aruco[cam_id_right].at(i));
        database->update_feature((size_t)ids_aruco[cam_id_right].at(i), timestamp, cam_id_right,
                                 corners[cam_id_right].at(i).at(0).x, corners[cam_id_right].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }


    // Move forward in time
    img_last[cam_id_left] = img_left.clone();
    img_last[cam_id_right] = img_right.clone();
    ids_last[cam_id_left] = ids_left_new;
    ids_last[cam_id_right] = ids_right_new;
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // Timing information
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for detection",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for feature DB update (%d features)",(rT3-rT2).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-ARUCO]: %.4f seconds for total",(rT3-rT1).total_microseconds() * 1e-6);

}


void TrackAruco::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) {

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for(auto const& pair : img_last) {
        if(max_width < pair.second.cols) max_width = pair.second.cols;
        if(max_height < pair.second.rows) max_height = pair.second.rows;
    }

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last.size()*max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if(image_new) img_out = cv::Mat(max_height,(int)img_last.size()*max_width,CV_8UC3,cv::Scalar(0,0,0));

    // Loop through each image, and draw
    int ct = 0;
    for(auto const& pair : img_last) {
        // select the subset of the image
        cv::Mat img_temp;
        if(image_new) cv::cvtColor(img_last[pair.first], img_temp, CV_GRAY2RGB);
        else img_temp = img_out(cv::Rect(max_width*ct,0,max_width,max_height));
        // draw...
        if(!ids_aruco[pair.first].empty()) cv::aruco::drawDetectedMarkers(img_temp, corners[pair.first], ids_aruco[pair.first]);
        //if(!rejects[pair.first].empty()) cv::aruco::drawDetectedMarkers(img_temp, rejects[pair.first], cv::noArray(), cv::Scalar(100,0,255));
        // Draw what camera this is
        cv::putText(img_temp, "CAM:"+std::to_string((int)pair.first), cv::Point(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0, cv::Scalar(0,255,0),3);
        // Replace the output image
        img_temp.copyTo(img_out(cv::Rect(max_width*ct,0,img_last[pair.first].cols,img_last[pair.first].rows)));
        // move fowards
        ct++;
    }

}