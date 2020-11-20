/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
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
#if defined(ROS_AVAILABLE) || defined(DOXYGEN)
#ifndef OV_MSCKF_PARSE_ROSHANDLER_H
#define OV_MSCKF_PARSE_ROSHANDLER_H


#include <ros/ros.h>

#include "core/VioManagerOptions.h"


namespace ov_msckf {



    /**
     * @brief This function will load paramters from the ros node handler / paramter server
     * This is the recommended way of loading parameters as compared to the command line version.
     * @param nh ROS node handler
     * @return A fully loaded VioManagerOptions object
     */
    VioManagerOptions parse_ros_nodehandler(ros::NodeHandle &nh) {

        // Our vio manager options with defaults
        VioManagerOptions params;

        // ESTIMATOR ======================================================================

        // Main EKF parameters
        nh.param<bool>("use_fej", params.state_options.do_fej, params.state_options.do_fej);
        nh.param<bool>("use_imuavg", params.state_options.imu_avg, params.state_options.imu_avg);
        nh.param<bool>("use_rk4int", params.state_options.use_rk4_integration, params.state_options.use_rk4_integration);
        nh.param<bool>("calib_cam_extrinsics", params.state_options.do_calib_camera_pose, params.state_options.do_calib_camera_pose);
        nh.param<bool>("calib_cam_intrinsics", params.state_options.do_calib_camera_intrinsics, params.state_options.do_calib_camera_intrinsics);
        nh.param<bool>("calib_cam_timeoffset", params.state_options.do_calib_camera_timeoffset, params.state_options.do_calib_camera_timeoffset);
        nh.param<int>("max_clones", params.state_options.max_clone_size, params.state_options.max_clone_size);
        nh.param<int>("max_slam", params.state_options.max_slam_features, params.state_options.max_slam_features);
        nh.param<int>("max_slam_in_update", params.state_options.max_slam_in_update, params.state_options.max_slam_in_update);
        nh.param<int>("max_msckf_in_update", params.state_options.max_msckf_in_update, params.state_options.max_msckf_in_update);
        nh.param<int>("max_aruco", params.state_options.max_aruco_features, params.state_options.max_aruco_features);
        nh.param<int>("max_cameras", params.state_options.num_cameras, params.state_options.num_cameras);
        nh.param<double>("dt_slam_delay", params.dt_slam_delay, params.dt_slam_delay);

        // Enforce that we have enough cameras to run
        if(params.state_options.num_cameras < 1) {
            printf(RED "VioManager(): Specified number of cameras needs to be greater than zero\n" RESET);
            printf(RED "VioManager(): num cameras = %d\n" RESET, params.state_options.num_cameras);
            std::exit(EXIT_FAILURE);
        }

        // Read in stereo pair information
        std::vector<int> stereo_pairs;
        nh.param<std::vector<int>>("stereo_pairs", stereo_pairs, stereo_pairs);
        if(stereo_pairs.size() % 2 != 0) {
            printf(RED "VioManager(): Specified number of stereo pair IDs needs to be even\n" RESET);
            printf(RED "VioManager(): Example: (0,1,2,3) -> stereo tracking between 01 and 23\n" RESET);
            std::exit(EXIT_FAILURE);
        }
        for(size_t i=0; i<stereo_pairs.size(); i++) {
            if(std::count(stereo_pairs.begin(),stereo_pairs.end(),stereo_pairs.at(i)) != 1) {
                printf(RED "VioManager(): You can do stereo tracking between unique ids\n" RESET);
                printf(RED "VioManager(): %d showed up multiple times\n" RESET,stereo_pairs.at(i));
                std::exit(EXIT_FAILURE);
            }
            //if(stereo_pairs.at(i) >= params.state_options.num_cameras) {
            //    printf(RED "VioManager(): Stereo pair has an id larger then the max camera\n" RESET);
            //    printf(RED "VioManager(): %d is >= than %d\n" RESET,stereo_pairs.at(i),params.state_options.num_cameras);
            //    std::exit(EXIT_FAILURE);
            //}
        }
        std::vector<int> valid_stereo_pairs;
        for(size_t i=0; i<stereo_pairs.size(); i+=2) {
            if(stereo_pairs.at(i) >= params.state_options.num_cameras || stereo_pairs.at(i+1) >= params.state_options.num_cameras) {
                printf(RED "ignoring invalid stereo pair: %d, %d\n" RESET, stereo_pairs.at(i), stereo_pairs.at(i+1));
                continue;
            }
            params.stereo_pairs.emplace_back(stereo_pairs.at(i),stereo_pairs.at(i+1));
            valid_stereo_pairs.push_back(stereo_pairs.at(i));
            valid_stereo_pairs.push_back(stereo_pairs.at(i+1));
        }

        // Calculate number of unique image camera image streams
        params.state_options.num_unique_cameras = (int)params.stereo_pairs.size();
        for(int i=0; i<params.state_options.num_cameras; i++) {
            if(std::find(valid_stereo_pairs.begin(),valid_stereo_pairs.end(),i)!=valid_stereo_pairs.end())
                continue;
            params.state_options.num_unique_cameras++;
        }

        // Read in what representation our feature is
        std::string feat_rep_msckf_str = "GLOBAL_3D";
        std::string feat_rep_slam_str = "GLOBAL_3D";
        std::string feat_rep_aruco_str = "GLOBAL_3D";
        nh.param<std::string>("feat_rep_msckf", feat_rep_msckf_str, feat_rep_msckf_str);
        nh.param<std::string>("feat_rep_slam", feat_rep_slam_str, feat_rep_slam_str);
        nh.param<std::string>("feat_rep_aruco", feat_rep_aruco_str, feat_rep_aruco_str);

        // Set what representation we should be using
        std::transform(feat_rep_msckf_str.begin(), feat_rep_msckf_str.end(),feat_rep_msckf_str.begin(), ::toupper);
        std::transform(feat_rep_slam_str.begin(), feat_rep_slam_str.end(),feat_rep_slam_str.begin(), ::toupper);
        std::transform(feat_rep_aruco_str.begin(), feat_rep_aruco_str.end(),feat_rep_aruco_str.begin(), ::toupper);
        params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string(feat_rep_msckf_str);
        params.state_options.feat_rep_slam = LandmarkRepresentation::from_string(feat_rep_slam_str);
        params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string(feat_rep_aruco_str);
        if(params.state_options.feat_rep_msckf == LandmarkRepresentation::Representation::UNKNOWN ||
        params.state_options.feat_rep_slam == LandmarkRepresentation::Representation::UNKNOWN ||
        params.state_options.feat_rep_aruco == LandmarkRepresentation::Representation::UNKNOWN) {
            printf(RED "VioManager(): invalid feature representation specified:\n" RESET);
            printf(RED "\t- GLOBAL_3D\n" RESET);
            printf(RED "\t- GLOBAL_FULL_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_3D\n" RESET);
            printf(RED "\t- ANCHORED_FULL_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_MSCKF_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_INVERSE_DEPTH_SINGLE\n" RESET);
            std::exit(EXIT_FAILURE);
        }

        // Filter initialization
        nh.param<double>("init_window_time", params.init_window_time, params.init_window_time);
        nh.param<double>("init_imu_thresh", params.init_imu_thresh, params.init_imu_thresh);

        // Zero velocity update
        nh.param<bool>("try_zupt", params.try_zupt, params.try_zupt);
        nh.param<int>("zupt_chi2_multipler", params.zupt_options.chi2_multipler, params.zupt_options.chi2_multipler);
        nh.param<double>("zupt_max_velocity", params.zupt_max_velocity, params.zupt_max_velocity);
        nh.param<double>("zupt_noise_multiplier", params.zupt_noise_multiplier, params.zupt_noise_multiplier);

        // Recording of timing information to file
        nh.param<bool>("record_timing_information", params.record_timing_information, params.record_timing_information);
        nh.param<std::string>("record_timing_filepath", params.record_timing_filepath, params.record_timing_filepath);


        // NOISE ======================================================================

        // Our noise values for inertial sensor
        nh.param<double>("gyroscope_noise_density", params.imu_noises.sigma_w, params.imu_noises.sigma_w);
        nh.param<double>("accelerometer_noise_density", params.imu_noises.sigma_a, params.imu_noises.sigma_a);
        nh.param<double>("gyroscope_random_walk", params.imu_noises.sigma_wb, params.imu_noises.sigma_wb);
        nh.param<double>("accelerometer_random_walk", params.imu_noises.sigma_ab, params.imu_noises.sigma_ab);

        // Read in update parameters
        nh.param<double>("up_msckf_sigma_px", params.msckf_options.sigma_pix, params.msckf_options.sigma_pix);
        nh.param<int>("up_msckf_chi2_multipler", params.msckf_options.chi2_multipler, params.msckf_options.chi2_multipler);
        nh.param<double>("up_slam_sigma_px", params.slam_options.sigma_pix, params.slam_options.sigma_pix);
        nh.param<int>("up_slam_chi2_multipler", params.slam_options.chi2_multipler, params.slam_options.chi2_multipler);
        nh.param<double>("up_aruco_sigma_px", params.aruco_options.sigma_pix, params.aruco_options.sigma_pix);
        nh.param<int>("up_aruco_chi2_multipler", params.aruco_options.chi2_multipler, params.aruco_options.chi2_multipler);


        // STATE ======================================================================

        // Timeoffset from camera to IMU
        nh.param<double>("calib_camimu_dt", params.calib_camimu_dt, params.calib_camimu_dt);

        // Global gravity
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        nh.param<std::vector<double>>("gravity", gravity, gravity);
        assert(gravity.size()==3);
        params.gravity << gravity.at(0),gravity.at(1),gravity.at(2);


        // TRACKERS ======================================================================

        // Tracking flags
        nh.param<bool>("use_stereo", params.use_stereo, params.use_stereo);
        nh.param<bool>("use_klt", params.use_klt, params.use_klt);
        nh.param<bool>("use_aruco", params.use_aruco, params.use_aruco);
        nh.param<bool>("downsize_aruco", params.downsize_aruco, params.downsize_aruco);
        nh.param<bool>("downsample_cameras", params.downsample_cameras, params.downsample_cameras);
        nh.param<bool>("multi_threading", params.use_multi_threading, params.use_multi_threading);

        // General parameters
        nh.param<int>("num_pts", params.num_pts, params.num_pts);
        nh.param<int>("fast_threshold", params.fast_threshold, params.fast_threshold);
        nh.param<int>("grid_x", params.grid_x, params.grid_x);
        nh.param<int>("grid_y", params.grid_y, params.grid_y);
        nh.param<int>("min_px_dist", params.min_px_dist, params.min_px_dist);
        nh.param<double>("knn_ratio", params.knn_ratio, params.knn_ratio);

        // Feature initializer parameters
        nh.param<bool>("fi_triangulate_1d", params.featinit_options.triangulate_1d, params.featinit_options.triangulate_1d);
        nh.param<bool>("fi_refine_features", params.featinit_options.refine_features, params.featinit_options.refine_features);
        nh.param<int>("fi_max_runs", params.featinit_options.max_runs, params.featinit_options.max_runs);
        nh.param<double>("fi_init_lamda", params.featinit_options.init_lamda, params.featinit_options.init_lamda);
        nh.param<double>("fi_max_lamda", params.featinit_options.max_lamda, params.featinit_options.max_lamda);
        nh.param<double>("fi_min_dx", params.featinit_options.min_dx, params.featinit_options.min_dx);
        nh.param<double>("fi_min_dcost", params.featinit_options.min_dcost, params.featinit_options.min_dcost);
        nh.param<double>("fi_lam_mult", params.featinit_options.lam_mult, params.featinit_options.lam_mult);
        nh.param<double>("fi_min_dist", params.featinit_options.min_dist, params.featinit_options.min_dist);
        nh.param<double>("fi_max_dist", params.featinit_options.max_dist, params.featinit_options.max_dist);
        nh.param<double>("fi_max_baseline", params.featinit_options.max_baseline, params.featinit_options.max_baseline);
        nh.param<double>("fi_max_cond_number", params.featinit_options.max_cond_number, params.featinit_options.max_cond_number);



        // SIMULATION ======================================================================

        // Load the groundtruth trajectory and its spline
        nh.param<std::string>("sim_traj_path", params.sim_traj_path, params.sim_traj_path);
        nh.param<double>("sim_distance_threshold", params.sim_distance_threshold, params.sim_distance_threshold);
        nh.param<bool>("sim_do_perturbation", params.sim_do_perturbation, params.sim_do_perturbation);

        // Read in sensor simulation frequencies
        nh.param<double>("sim_freq_cam", params.sim_freq_cam, params.sim_freq_cam);
        nh.param<double>("sim_freq_imu", params.sim_freq_imu, params.sim_freq_imu);

        // Load the seeds for the random number generators
        nh.param<int>("sim_seed_state_init", params.sim_seed_state_init, params.sim_seed_state_init);
        nh.param<int>("sim_seed_preturb", params.sim_seed_preturb, params.sim_seed_preturb);
        nh.param<int>("sim_seed_measurements", params.sim_seed_measurements, params.sim_seed_measurements);



        //====================================================================================
        //====================================================================================
        //====================================================================================

        // Loop through through, and load each of the cameras
        for(int i=0; i<params.state_options.num_cameras; i++) {

            // If our distortions are fisheye or not!
            bool is_fisheye;
            nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);

            // If the desired fov we should simulate
            std::vector<int> matrix_wh;
            std::vector<int> matrix_wd_default = {752,480};
            nh.param<std::vector<int>>("cam"+std::to_string(i)+"_wh", matrix_wh, matrix_wd_default);
            matrix_wh.at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_wh.at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
            std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));

            // Camera intrinsic properties
            Eigen::Matrix<double,8,1> cam_calib;
            std::vector<double> matrix_k, matrix_d;
            std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
            std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
            nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
            nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
            matrix_k.at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(2) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(3) /= (params.downsample_cameras) ? 2.0 : 1.0;
            cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

            // Our camera extrinsics transform
            Eigen::Matrix4d T_CtoI;
            std::vector<double> matrix_TCtoI;
            std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

            // Read in from ROS, and save into our eigen mat
            nh.param<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TCtoI, matrix_TtoI_default);
            T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                    matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                    matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                    matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);

            // Load these into our state
            Eigen::Matrix<double,7,1> cam_eigen;
            cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
            cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);

            // Insert
            params.camera_fisheye.insert({i, is_fisheye});
            params.camera_intrinsics.insert({i, cam_calib});
            params.camera_extrinsics.insert({i, cam_eigen});
            params.camera_wh.insert({i, wh});

        }



        // Success, lets returned the parsed options
        return params;

    }



}


#endif //OV_MSCKF_PARSE_ROSHANDLER_H
#endif //ROS_AVAILABLE