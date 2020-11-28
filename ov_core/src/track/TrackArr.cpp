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

    // traverse for each camera
    for (size_t i = 0; i < feats.size(); i ++) {
        // traverse for each feature point
        for (const auto &feat : feats.at(i)) {
            // Get our id value
            size_t id = feat.first;
            // Create the keypoint
            cv::Point2d pt;
            pt.x = feat.second(0);
            pt.y = feat.second(1);
            
            pts_curr.insert(std::pair<size_t, cv::Point2d>(id, pt));
        }
    }

}

void TrackArr::show(){
    // TODO: left for real-world simulation

    std::cout << "pts_old  [size : " << pts_old.size() <<  "]" << std::endl;
    for (auto it = pts_old.cbegin(); it != pts_old.cend(); ++it) {
        std::cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

    std::cout << "pts_prev [size : " << pts_prev.size() <<  "]" << std::endl;
    for (auto it = pts_prev.cbegin(); it != pts_prev.cend(); ++it) {
        std::cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

    std::cout << "pts_curr [size : " << pts_curr.size() <<  "]" << std::endl;
    for (auto it = pts_curr.cbegin(); it != pts_curr.cend(); ++it) {
        std::cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

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
    
    /* 
    // VERSION 1 (Refer to 'Here is the derivation:'): https://www.researchgate.net/post/Validity_of_Rotation_matrix_calculations_from_angular_velocity_reading
    Eigen::Matrix3d w_prev_skewed = (Eigen::Matrix3d::Identity(3, 3) - R_0to1_e.transpose()) / dt_prev;
    Eigen::Matrix3d w_curr_skewed = (Eigen::Matrix3d::Identity(3, 3) - R_1to2_e.transpose()) / dt_curr;
    Eigen::Matrix<double, 3, 1> w_prev; w_prev << -w_prev_skewed(1, 2), w_prev_skewed(0, 2), -w_prev_skewed(0, 1);
    Eigen::Matrix<double, 3, 1> w_curr; w_curr << -w_curr_skewed(1, 2), w_curr_skewed(0, 2), -w_curr_skewed(0, 1);
    Eigen::Matrix<double, 3, 1> a_curr = (w_curr - R_1to2_e * w_prev) / ((dt_prev + dt_curr) / 2);

    w  = Eigen::Map<Eigen::Vector3d>(w_curr.data(), w_curr.cols() * w_curr.rows());
    wd = Eigen::Map<Eigen::Vector3d>(a_curr.data(), a_curr.cols() * a_curr.rows());
    */

    // VERSION 2: Refer to https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-07-dynamics-fall-2009/lecture-notes/MIT16_07F09_Lec29.pdf
    Eigen::Vector3d ea_1to2 = R_1to2_e.eulerAngles(2, 1, 0);
    double phi = ea_1to2(2);
    double theta = ea_1to2(1);
    double psi = ea_1to2(0);
    // Assume that all angles used to be zero
    w(0) = phi / dt_curr + psi / dt_curr * sin(theta);
    w(1) = theta / dt_curr * cos(phi) - psi / dt_curr * cos(theta) * sin(phi);
    w(2) = theta / dt_curr * sin(phi) + psi / dt_curr * cos(phi) * cos(theta);

    // TODO: Add wd here    
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
            // std::cout << "pts_old : (" << iter->second.x << ", " << iter->second.y << ")  [ID: " << iter->first;
            // std::cout << "] matches pts_prev: (" << pts_prev.find(iter->first)->second.x << ", " << pts_prev.find(iter->first)->second.y << ")" << std::endl;
        }
    }
    // std::cout << "-- Match completed. pts_old.size() : " << pts1.size() << ", pts_prev.size(): " << pts2.size() << std::endl;
}

void TrackArr::get_keypoint_pairs_curr(std::vector<cv::Point2d>& pts1, std::vector<cv::Point2d>& pts2) {
    // Match keypoint pairs by comparing pts_prev and pts_curr
    std::map<size_t, cv::Point2d>::iterator iter;
    for(iter = pts_prev.begin(); iter != pts_prev.end(); iter ++) {
        if (pts_curr.find(iter->first) != pts_curr.end()) {
            pts1.emplace_back(iter->second);
            pts2.emplace_back((pts_curr.find(iter->first))->second);
            // std::cout << "pts_prev: (" << iter->second.x << ", " << iter->second.y << ")  [ID: " << iter->first;
            // std::cout << "] matches pts_curr: (" << pts_curr.find(iter->first)->second.x << ", " << pts_curr.find(iter->first)->second.y << ")" << std::endl;
        }
    }
    // std::cout << "-- Match completed. pts_prev.size(): " << pts1.size() << ", pts_curr.size(): " << pts2.size() << std::endl;
}