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

#include "track/Grider_FAST.h"
#include "track/Grider_DOG.h"
#include "feat/FeatureDatabase.h"
#include "utils/colors.h"


using std::vector;
using std::array;
using std::map;
using std::pair;
using std::cout;
using std::endl;
using Eigen::Matrix3d; 
using Eigen::Vector3d;
using Eigen::VectorXf;
using cv::Mat;
using cv::Point2d;

namespace ov_core {
    class TrackArr {

    public:
        TrackArr(Mat camera_matrix);
        void push_back(const double timestamp, const vector<vector<pair<size_t,VectorXf>>> &feats);
        void show();
        void calc_motion(Vector3d& w, Vector3d& wd, Matrix3d& R);
        
    protected:
        void get_keypoint_pairs_prev(vector<Point2d>& pts1, vector<Point2d>& pts2);
        void get_keypoint_pairs_curr(vector<Point2d>& pts1, vector<Point2d>& pts2);

        // camera matrix
        Mat camera_matrix_;
        // sets of id-point pairs
        map<size_t, Point2d> pts_old;
        map<size_t, Point2d> pts_prev;
        map<size_t, Point2d> pts_curr;
        /// Last three timestamps
        array<double, 3> timestamps;
    };

}


#endif /* OV_CORE_TRACK_ARR_H */