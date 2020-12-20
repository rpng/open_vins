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

#include "types/LandmarkRepresentation.h"

using namespace ov_type;

namespace ov_msckf {


    /**
     * @brief Struct which stores all our filter options
     */
    struct StateOptions {

        /// Bool to determine whether or not to do first estimate Jacobians
        bool do_fej = true;

        /// Bool to determine whether or not use imu message averaging
        bool imu_avg = false;

        /// Bool to determine if we should use Rk4 imu integration
        bool use_rk4_integration = true;

        /// Bool to determine whether or not to calibrate imu-to-camera pose
        bool do_calib_camera_pose = false;

        /// Bool to determine whether or not to calibrate camera intrinsics
        bool do_calib_camera_intrinsics = false;

        /// Bool to determine whether or not to calibrate camera to IMU time offset
        bool do_calib_camera_timeoffset = false;

        /// Max clone size of sliding window
        int max_clone_size = 11;

        /// Max number of estimated SLAM features
        int max_slam_features = 25;

        /// Max number of SLAM features we allow to be included in a single EKF update.
        int max_slam_in_update = INT_MAX;

        /// Max number of MSCKF features we will use at a given image timestep.
        int max_msckf_in_update = INT_MAX;

        /// Max number of estimated ARUCO features
        int max_aruco_features = 1024;

        /// Number of distinct cameras that we will observe features in
        int num_cameras = 1;

        /// Number of cameras (a stereo pair is not consider unique since both images occur at the same pose timestamp)
        int num_unique_cameras = 1;

        /// What representation our features are in (msckf features)
        LandmarkRepresentation::Representation feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;

        /// What representation our features are in (slam features)
        LandmarkRepresentation::Representation feat_rep_slam = LandmarkRepresentation::Representation::GLOBAL_3D;

        /// What representation our features are in (aruco tag features)
        LandmarkRepresentation::Representation feat_rep_aruco = LandmarkRepresentation::Representation::GLOBAL_3D;

        /// Nice print function of what parameters we have loaded
        void print() {
            printf("\t- use_fej: %d\n", do_fej);
            printf("\t- use_imuavg: %d\n", imu_avg);
            printf("\t- use_rk4int: %d\n", use_rk4_integration);
            printf("\t- calib_cam_extrinsics: %d\n", do_calib_camera_pose);
            printf("\t- calib_cam_intrinsics: %d\n", do_calib_camera_intrinsics);
            printf("\t- calib_cam_timeoffset: %d\n", do_calib_camera_timeoffset);
            printf("\t- max_clones: %d\n", max_clone_size);
            printf("\t- max_slam: %d\n", max_slam_features);
            printf("\t- max_slam_in_update: %d\n", max_slam_in_update);
            printf("\t- max_msckf_in_update: %d\n", max_msckf_in_update);
            printf("\t- max_aruco: %d\n", max_aruco_features);
            printf("\t- max_cameras: %d\n", num_cameras);
            printf("\t- num_unique_cameras: %d\n", num_unique_cameras);
            printf("\t- feat_rep_msckf: %s\n", LandmarkRepresentation::as_string(feat_rep_msckf).c_str());
            printf("\t- feat_rep_slam: %s\n", LandmarkRepresentation::as_string(feat_rep_slam).c_str());
            printf("\t- feat_rep_aruco: %s\n", LandmarkRepresentation::as_string(feat_rep_aruco).c_str());
        }

    };


}

#endif //OV_MSCKF_STATE_OPTIONS_H