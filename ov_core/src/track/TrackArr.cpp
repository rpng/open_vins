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

using std::vector;
using std::pair;
using std::cout;
using std::endl;
using Eigen::Matrix3d; 
using Eigen::Vector3d;
using Eigen::VectorXf;
using cv::Mat;
using cv::Point2d;

TrackArr::TrackArr(Mat camera_matrix) {
    camera_matrix_ = camera_matrix;
}

void TrackArr::push_back(const double timestamp, const vector<vector<pair<size_t,VectorXf>>> &feats) {
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
            Point2d pt;
            pt.x = feat.second(0);
            pt.y = feat.second(1);
            
            pts_curr.insert(pair<size_t, Point2d>(id, pt));
        }
    }

}

void TrackArr::show(){
    // TODO: left for real-world simulation

    cout << "pts_old  [size : " << pts_old.size() <<  "]" << endl;
    for (auto it = pts_old.cbegin(); it != pts_old.cend(); ++it) {
        cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

    cout << "pts_prev [size : " << pts_prev.size() <<  "]" << endl;
    for (auto it = pts_prev.cbegin(); it != pts_prev.cend(); ++it) {
        cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

    cout << "pts_curr [size : " << pts_curr.size() <<  "]" << endl;
    for (auto it = pts_curr.cbegin(); it != pts_curr.cend(); ++it) {
        cout << "id " << it->first << ": (" << it->second.x << ", " << it->second.y << ")\n";
    }

    return;
}

void TrackArr::calc_motion(Vector3d& w, Vector3d& wd, Matrix3d& R) {
    // TODO: calculate relative motion between two consecutive camera frames
    // ASSUMPTION: all features are identified by each corresponding label

    vector<Point2d> pts1, pts2, pts3, pts4;
    Mat E_0to1, R_0to1, t_0to1, E_1to2, R_1to2, t_1to2, mask;

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
    E_1to2 = findEssentialMat(pts3, pts4, camera_matrix_, cv::FM_RANSAC, 0.99, 1., mask);
    cv::recoverPose(E_1to2, pts3, pts4, camera_matrix_, R_1to2, t_1to2, mask);
    
    double dt_prev = (timestamps.at(1) - timestamps.at(0));
    double dt_curr = (timestamps.at(2) - timestamps.at(1));
    Matrix3d R_0to1_e, R_1to2_e;
    cv::cv2eigen(R_0to1, R_0to1_e);
    cv::cv2eigen(R_1to2, R_1to2_e);
    
    // estimate w (angular velocity at k)
    Matrix3d v_skewed = R_1to2_e - R_1to2_e.transpose();
    Vector3d v; 
    v << -v_skewed(1, 2), v_skewed(0, 2), -v_skewed(0, 1);
    double w_norm = asin(v.norm() / 2) / dt_curr;
    w = R_1to2_e * v.normalized() * w_norm;

    // TODO: Add wd here
    R = R_1to2_e;
    
    return ;
}

void TrackArr::get_keypoint_pairs_prev(vector<Point2d>& pts1, vector<Point2d>& pts2) {
    // Match keypoint pairs by comparing pts_old and pts_prev
    std::map<size_t, Point2d>::iterator iter;
    for(iter = pts_old.begin(); iter != pts_old.end(); iter ++) {
        if (pts_prev.find(iter->first) != pts_prev.end()) {
            pts1.emplace_back(iter->second);
            pts2.emplace_back((pts_prev.find(iter->first))->second);
            // cout << "pts_old : (" << iter->second.x << ", " << iter->second.y << ")  [ID: " << iter->first;
            // cout << "] matches pts_prev: (" << pts_prev.find(iter->first)->second.x << ", " << pts_prev.find(iter->first)->second.y << ")" << endl;
        }
    }
    // cout << "-- Match completed. pts_old.size() : " << pts1.size() << ", pts_prev.size(): " << pts2.size() << endl;
}

void TrackArr::get_keypoint_pairs_curr(vector<Point2d>& pts1, vector<Point2d>& pts2) {
    // Match keypoint pairs by comparing pts_prev and pts_curr
    std::map<size_t, Point2d>::iterator iter;
    for(iter = pts_prev.begin(); iter != pts_prev.end(); iter ++) {
        if (pts_curr.find(iter->first) != pts_curr.end()) {
            pts1.emplace_back(iter->second);
            pts2.emplace_back((pts_curr.find(iter->first))->second);
            // cout << "pts_prev: (" << iter->second.x << ", " << iter->second.y << ")  [ID: " << iter->first;
            // cout << "] matches pts_curr: (" << pts_curr.find(iter->first)->second.x << ", " << pts_curr.find(iter->first)->second.y << ")" << endl;
        }
    }
    // cout << "-- Match completed. pts_prev.size(): " << pts1.size() << ", pts_curr.size(): " << pts2.size() << endl;
}