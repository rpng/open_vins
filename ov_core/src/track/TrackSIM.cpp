#include "TrackSIM.h"


using namespace ov_core;




void TrackSIM::feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::Vector2d>>> &feats) {


    // Assert our two vectors are equal
    assert(camids.size()==feats.size());

    // Loop through each camera
    for(size_t i=0; i<camids.size(); i++) {

        // Current camera id
        int cam_id = camids.at(i);

        // Our good ids and points
        std::vector<cv::KeyPoint> good_left;
        std::vector<size_t> good_ids_left;

        // Update our feature database, with theses new observations
        // NOTE: we add the "currid" since we need to offset the simulator
        // NOTE: ids by the number of aruoc tags we have specified as tracking
        for(const auto &feat : feats.at(i)) {

            // Create the keypoint
            cv::KeyPoint kpt;
            kpt.pt.x = feat.second(0);
            kpt.pt.y = feat.second(1);
            good_left.push_back(kpt);
            good_ids_left.push_back(feat.first+currid);

            // Append to the database
            cv::Point2f npt_l = undistort_point(kpt.pt, cam_id);
            database->update_feature(feat.first+currid, timestamp, cam_id,
                                     kpt.pt.x, kpt.pt.y, npt_l.x, npt_l.y);
        }

        // Get our width and height
        auto wh = camera_wh.at(cam_id);

        // Move forward in time
        img_last[cam_id] = cv::Mat::zeros(cv::Size(wh.first,wh.second), CV_8UC1);
        pts_last[cam_id] = good_left;
        ids_last[cam_id] = good_ids_left;

    }


}

