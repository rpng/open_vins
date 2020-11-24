/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "TrackArr.h"


using namespace ov_core;

TrackArr::TrackArr(cv::Mat camera_matrix) {
    camera_matrix_ = camera_matrix;
}

void TrackArr::push_back(double timestamp, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {
    // Change data
    pts_old = pts_prev;
    pts_prev = pts_curr;
    pts_curr.clear();
    timestamps.at(0) = timestamps.at(1);
    timestamps.at(1) = timestamps.at(2);
    timestamps.at(2) = timestamp;

    // Update our feature database, with theses new observations
    // NOTE: we add the "currid" since we need to offset the simulator
    // NOTE: ids by the number of aruoc tags we have specified as tracking
    for(const auto &feat : feats.at(0)) {
        // Get our id value
        size_t id = feat.first;
        // Create the keypoint
        cv::Point2d pt;
        pt.x = feat.second(0);
        pt.y = feat.second(1);
        
        pts_curr.insert(std::pair<size_t, cv::Point2d>(id, pt));
    }

}

void TrackArr::show(){
    // TODO: left for real-world simulation
    return;
}

void TrackArr::calc_motion(Eigen::Vector3d& w, Eigen::Vector3d& wd, Eigen::Matrix3d& R) {
    // TODO: calculate relative motion between two consecutive camera frames
    // ASSUMPTION: all features are identified by each corresponding label

    std::vector<cv::Point2d> pts1, pts2, pts3, pts4;
    cv::Mat E_0to1, R_0to1, t_0to1, E_1to2, R_1to2, t_1to2, mask;

    // get correponding pairs between (k-2, k-1) and (k-1, k)
    get_keypoint_pairs_prev(pts1, pts2);
    get_keypoint_pairs_curr(pts3, pts4);

    // do not start estimating essential matrix when less than five points are given
    if ((pts1.size() < 8) || (pts3.size() < 8)) {
        printf(RED "[SIM]: Motion estimation failed. Requires points more than four. \n" RESET);
        R.setIdentity(3, 3);
        return;
    }
    
    // estimate R_0to1 (rotation between k-2 and k-1) and R_1to2 (rotation between k-1 and k)
    E_0to1 = cv::findEssentialMat(pts1, pts2, camera_matrix_, cv::FM_RANSAC, 0.99, 1., mask);
    cv::recoverPose(E_0to1, pts1, pts2, camera_matrix_, R_0to1, t_0to1, mask);
    E_1to2 = cv::findEssentialMat(pts3, pts4, camera_matrix_, cv::FM_RANSAC, 0.99, 1., mask);
    cv::recoverPose(E_1to2, pts3, pts4, camera_matrix_, R_1to2, t_1to2, mask);

    // estimate w_prev (angular velocity at k-1) and w_curr (angular velocity at k)
    double dt_prev = (timestamps.at(1) - timestamps.at(0));
    double dt_curr = (timestamps.at(2) - timestamps.at(1));
    Eigen::Matrix3d R_0to1_e, R_1to2_e;
    cv::cv2eigen(R_0to1, R_0to1_e);
    cv::cv2eigen(R_1to2, R_1to2_e);
    
    Eigen::Matrix3d w_prev_skewed = (Eigen::Matrix3d::Identity(3, 3) - R_0to1_e.transpose()) / dt_prev;
    Eigen::Matrix3d w_curr_skewed = (Eigen::Matrix3d::Identity(3, 3) - R_1to2_e.transpose()) / dt_curr;
    Eigen::Matrix<double, 3, 1> w_prev; w_prev << -w_prev_skewed(1, 2), w_prev_skewed(0, 2), -w_prev_skewed(0, 1);
    Eigen::Matrix<double, 3, 1> w_curr; w_curr << -w_curr_skewed(1, 2), w_curr_skewed(0, 2), -w_curr_skewed(0, 1);
    Eigen::Matrix<double, 3, 1> a_curr = (w_curr - R_1to2_e * w_prev) / ((dt_prev + dt_curr) / 2);

    w  = Eigen::Map<Eigen::Vector3d>(w_curr.data(), w_curr.cols() * w_curr.rows());
    wd = Eigen::Map<Eigen::Vector3d>(a_curr.data(), a_curr.cols() * a_curr.rows());
    R = R_1to2_e;
    
    return ;
}

void TrackArr::get_keypoint_pairs_prev(std::vector<cv::Point2d>& pts1, std::vector<cv::Point2d>& pts2) {
    // Match keypoint pairs by comparing pts_old and pts_prev
    std::map<size_t, cv::Point2d>::iterator iter;
    for(iter = pts_old.begin(); iter != pts_old.end(); iter ++) {
        if (pts_prev.find(iter->first) != pts_prev.end()) {
            pts1.emplace_back(iter->second);
            pts2.emplace_back((pts_prev.find(iter->first))->second);
        }
    }
}

void TrackArr::get_keypoint_pairs_curr(std::vector<cv::Point2d>& pts1, std::vector<cv::Point2d>& pts2) {
    // Match keypoint pairs by comparing pts_prev and pts_curr
    std::map<size_t, cv::Point2d>::iterator iter;
    for(iter = pts_prev.begin(); iter != pts_prev.end(); iter ++) {
        if (pts_curr.find(iter->first) != pts_curr.end()) {
            pts1.emplace_back(iter->second);
            pts2.emplace_back((pts_curr.find(iter->first))->second);
        }
    }
}