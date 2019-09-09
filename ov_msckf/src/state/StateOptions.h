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
#ifndef OV_MSCKF_STATE_OPTIONS_H
#define OV_MSCKF_STATE_OPTIONS_H

#include "feat/FeatureRepresentation.h"

using namespace ov_core;

namespace ov_msckf {


    /**
     * @brief Struct which stores all our filter options
     */
    struct StateOptions {

        /// Bool to determine whether or not to do first estimate Jacobians
        bool do_fej = false;

        /// Bool to determine whether or not use imu message averaging
        bool imu_avg = false;

        /// Bool to determine if we should use Rk4 imu integration
        bool use_rk4_integration = false;

        /// Bool to determine whether or not to calibrate imu-to-camera pose
        bool do_calib_camera_pose = false;

        /// Bool to determine whether or not to calibrate camera intrinsics
        bool do_calib_camera_intrinsics = false;

        /// Bool to determine whether or not to calibrate camera to IMU time offset
        bool do_calib_camera_timeoffset = false;

        /// Max clone size of sliding window
        int max_clone_size = 8;

        /// Max number of estimated SLAM features
        int max_slam_features = 0;

        /// Max number of estimated ARUCO features
        int max_aruco_features = 1024;

        /// Number of cameras
        int num_cameras = 1;

        /// What representation our features are in
        FeatureRepresentation::Representation feat_representation = FeatureRepresentation::Representation::GLOBAL_3D;

    };


}

#endif //OV_MSCKF_STATE_OPTIONS_H