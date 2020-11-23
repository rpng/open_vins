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
#ifndef OV_CORE_TRACK_ARR_H
#define OV_CORE_TRACK_ARR_H

#define RESET       "\033[0m"
#define RED         "\033[31m"             /* Red */

#include <cstdio>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <Eigen/StdVector>

#include <boost/thread.hpp>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Grider_FAST.h"
#include "Grider_DOG.h"
#include "feat/FeatureDatabase.h"
#include "utils/colors.h"


namespace ov_core {
    class TrackArr {

    public:
        void feed_camera_matrix(cv::Mat cameraMatrix);
        void push_back(double timestamp, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats);
        void show();
        void calc_motion(Eigen::Vector3d& w, Eigen::Vector3d& wd, Eigen::Matrix3d& R);
        
    protected:
        void get_keypoint_pairs_prev(std::vector<cv::Point2d>& pts1, std::vector<cv::Point2d>& pts2);
        void get_keypoint_pairs_curr(std::vector<cv::Point2d>& pts1, std::vector<cv::Point2d>& pts2);

        // camera matrix
        cv::Mat cameraMatrix;
        // count until three elements of each array are filled
        size_t count = 0;
        // sets of id-point pairs
        std::map<size_t, cv::Point2d> pts_old;
        std::map<size_t, cv::Point2d> pts_prev;
        std::map<size_t, cv::Point2d> pts_curr;
        /// Last three timestamps
        std::array<double, 3> timestamps;
    };

}


#endif /* OV_CORE_TRACK_ARR_H */