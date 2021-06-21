/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
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


#ifndef OV_MSCKF_PARSE_CMDLINE_H
#define OV_MSCKF_PARSE_CMDLINE_H

#include "core/VioManagerOptions.h"
#include "utils/CLI11.hpp"

namespace ov_msckf {

/**
 * @brief This function will parse the command line arugments using [CLI11](https://github.com/CLIUtils/CLI11).
 * This is only used if you are not building with ROS, and thus isn't the primary supported way to pass arguments.
 * We recommend building with ROS as compared using this parser.
 * @param argc Number of parameters
 * @param argv Pointer to string passed as options
 * @return A fully loaded VioManagerOptions object
 */
VioManagerOptions parse_command_line_arguments(int argc, char **argv) {

  // Our vio manager options with defaults
  VioManagerOptions params;

  // Create our command line parser
  CLI::App app1{"parser_cmd_01"};
  app1.allow_extras();

  // ESTIMATOR ======================================================================

  // Main EKF parameters
  app1.add_option("--use_fej", params.state_options.do_fej, "");
  app1.add_option("--use_imuavg", params.state_options.imu_avg, "");
  app1.add_option("--use_rk4int", params.state_options.use_rk4_integration, "");
  app1.add_option("--calib_cam_extrinsics", params.state_options.do_calib_camera_pose, "");
  app1.add_option("--calib_cam_intrinsics", params.state_options.do_calib_camera_intrinsics, "");
  app1.add_option("--calib_cam_timeoffset", params.state_options.do_calib_camera_timeoffset, "");
  app1.add_option("--max_clones", params.state_options.max_clone_size, "");
  app1.add_option("--max_slam", params.state_options.max_slam_features, "");
  app1.add_option("--max_slam_in_update", params.state_options.max_slam_in_update, "");
  app1.add_option("--max_msckf_in_update", params.state_options.max_msckf_in_update, "");
  app1.add_option("--max_aruco", params.state_options.max_aruco_features, "");
  app1.add_option("--max_cameras", params.state_options.num_cameras, "");
  app1.add_option("--dt_slam_delay", params.dt_slam_delay, "");

  // Stereo pairs
  std::vector<int> stereo_pairs;
  app1.add_option("--stereo_pairs", stereo_pairs, "");

  // Read in what representation our feature is
  std::string feat_rep_msckf_str = "GLOBAL_3D";
  std::string feat_rep_slam_str = "GLOBAL_3D";
  std::string feat_rep_aruco_str = "GLOBAL_3D";
  app1.add_option("--feat_rep_msckf", feat_rep_msckf_str, "");
  app1.add_option("--feat_rep_slam", feat_rep_slam_str, "");
  app1.add_option("--feat_rep_aruco", feat_rep_aruco_str, "");

  // Filter initialization
  app1.add_option("--init_window_time", params.init_window_time, "");
  app1.add_option("--init_imu_thresh", params.init_imu_thresh, "");

  // Zero velocity update
  app1.add_option("--try_zupt", params.try_zupt, "");
  app1.add_option("--zupt_chi2_multipler", params.zupt_options.chi2_multipler, "");
  app1.add_option("--zupt_max_velocity", params.zupt_max_velocity, "");
  app1.add_option("--zupt_noise_multiplier", params.zupt_noise_multiplier, "");
  app1.add_option("--zupt_max_disparity", params.zupt_max_disparity, "");
  app1.add_option("--zupt_only_at_beginning", params.zupt_only_at_beginning, "");

  // Recording of timing information to file
  app1.add_option("--record_timing_information", params.record_timing_information, "");
  app1.add_option("--record_timing_filepath", params.record_timing_filepath, "");

  // NOISE ======================================================================

  // Our noise values for inertial sensor
  app1.add_option("--gyroscope_noise_density", params.imu_noises.sigma_w, "");
  app1.add_option("--accelerometer_noise_density", params.imu_noises.sigma_a, "");
  app1.add_option("--gyroscope_random_walk", params.imu_noises.sigma_wb, "");
  app1.add_option("--accelerometer_random_walk", params.imu_noises.sigma_ab, "");

  // Read in update parameters
  app1.add_option("--up_msckf_sigma_px", params.msckf_options.sigma_pix, "");
  app1.add_option("--up_msckf_chi2_multipler", params.msckf_options.chi2_multipler, "");
  app1.add_option("--up_slam_sigma_px", params.slam_options.sigma_pix, "");
  app1.add_option("--up_slam_chi2_multipler", params.slam_options.chi2_multipler, "");
  app1.add_option("--up_aruco_sigma_px", params.aruco_options.sigma_pix, "");
  app1.add_option("--up_aruco_chi2_multipler", params.aruco_options.chi2_multipler, "");

  // STATE ======================================================================

  // Timeoffset from camera to IMU
  app1.add_option("--calib_camimu_dt", params.calib_camimu_dt, "");

  // Global gravity
  app1.add_option("--gravity_mag", params.gravity_mag, "");

  // TRACKERS ======================================================================

  // Tracking flags
  app1.add_option("--use_stereo", params.use_stereo, "");
  app1.add_option("--use_klt", params.use_klt, "");
  app1.add_option("--use_aruco", params.use_aruco, "");
  app1.add_option("--downsize_aruco", params.downsize_aruco, "");
  app1.add_option("--downsample_cameras", params.downsample_cameras, "");
  app1.add_option("--multi_threading", params.use_multi_threading, "");

  // General parameters
  app1.add_option("--num_pts", params.num_pts, "");
  app1.add_option("--fast_threshold", params.fast_threshold, "");
  app1.add_option("--grid_x", params.grid_x, "");
  app1.add_option("--grid_y", params.grid_y, "");
  app1.add_option("--min_px_dist", params.min_px_dist, "");
  app1.add_option("--knn_ratio", params.knn_ratio, "");

  // Feature initializer parameters
  app1.add_option("--fi_triangulate_1d", params.featinit_options.triangulate_1d, "");
  app1.add_option("--fi_refine_features", params.featinit_options.refine_features, "");
  app1.add_option("--fi_max_runs", params.featinit_options.max_runs, "");
  app1.add_option("--fi_init_lamda", params.featinit_options.init_lamda, "");
  app1.add_option("--fi_max_lamda", params.featinit_options.max_lamda, "");
  app1.add_option("--fi_min_dx", params.featinit_options.min_dx, "");
  app1.add_option("--fi_min_dcost", params.featinit_options.min_dcost, "");
  app1.add_option("--fi_lam_mult", params.featinit_options.lam_mult, "");
  app1.add_option("--fi_min_dist", params.featinit_options.min_dist, "");
  app1.add_option("--fi_max_dist", params.featinit_options.max_dist, "");
  app1.add_option("--fi_max_baseline", params.featinit_options.max_baseline, "");
  app1.add_option("--fi_max_cond_number", params.featinit_options.max_cond_number, "");

  // SIMULATION ======================================================================

  // Load the groundtruth trajectory and its spline
  app1.add_option("--sim_traj_path", params.sim_traj_path, "");
  app1.add_option("--sim_distance_threshold", params.sim_distance_threshold, "");
  app1.add_option("--sim_do_perturbation", params.sim_do_perturbation, "");

  // Read in sensor simulation frequencies
  app1.add_option("--sim_freq_cam", params.sim_freq_cam, "");
  app1.add_option("--sim_freq_imu", params.sim_freq_imu, "");

  // Load the seeds for the random number generators
  app1.add_option("--sim_seed_state_init", params.sim_seed_state_init, "");
  app1.add_option("--sim_seed_preturb", params.sim_seed_preturb, "");
  app1.add_option("--sim_seed_measurements", params.sim_seed_measurements, "");

  // CMD PARSE ==============================================================================

  // Finally actually parse the command line and load it
  try {
    app1.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    std::exit(app1.exit(e));
  }

  // Read in stereo pair information
  if (stereo_pairs.size() % 2 != 0) {
    printf(RED "VioManager(): Specified number of stereo pair IDs needs to be even\n" RESET);
    printf(RED "VioManager(): Example: (0,1,2,3) -> stereo tracking between 01 and 23\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  for (size_t i = 0; i < stereo_pairs.size(); i++) {
    if (std::count(stereo_pairs.begin(), stereo_pairs.end(), stereo_pairs.at(i)) != 1) {
      printf(RED "VioManager(): You can do stereo tracking between unique ids\n" RESET);
      printf(RED "VioManager(): %d showed up multiple times\n" RESET, stereo_pairs.at(i));
      std::exit(EXIT_FAILURE);
    }
    // if(stereo_pairs.at(i) >= params.state_options.num_cameras) {
    //    printf(RED "VioManager(): Stereo pair has an id larger then the max camera\n" RESET);
    //    printf(RED "VioManager(): %d is >= than %d\n" RESET,stereo_pairs.at(i),params.state_options.num_cameras);
    //    std::exit(EXIT_FAILURE);
    //}
  }
  std::vector<int> valid_stereo_pairs;
  for (size_t i = 0; i < stereo_pairs.size(); i += 2) {
    if (stereo_pairs.at(i) >= params.state_options.num_cameras || stereo_pairs.at(i + 1) >= params.state_options.num_cameras) {
      printf(RED "ignoring invalid stereo pair: %d, %d\n" RESET, stereo_pairs.at(i), stereo_pairs.at(i + 1));
      continue;
    }
    params.stereo_pairs.emplace_back(stereo_pairs.at(i), stereo_pairs.at(i + 1));
    valid_stereo_pairs.push_back(stereo_pairs.at(i));
    valid_stereo_pairs.push_back(stereo_pairs.at(i + 1));
  }

  // Calculate number of unique image camera image streams
  params.state_options.num_unique_cameras = (int)params.stereo_pairs.size();
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    if (std::find(valid_stereo_pairs.begin(), valid_stereo_pairs.end(), i) != valid_stereo_pairs.end())
      continue;
    params.state_options.num_unique_cameras++;
  }

  // Set what representation we should be using
  std::transform(feat_rep_msckf_str.begin(), feat_rep_msckf_str.end(), feat_rep_msckf_str.begin(), ::toupper);
  std::transform(feat_rep_slam_str.begin(), feat_rep_slam_str.end(), feat_rep_slam_str.begin(), ::toupper);
  std::transform(feat_rep_aruco_str.begin(), feat_rep_aruco_str.end(), feat_rep_aruco_str.begin(), ::toupper);
  params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string(feat_rep_msckf_str);
  params.state_options.feat_rep_slam = LandmarkRepresentation::from_string(feat_rep_slam_str);
  params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string(feat_rep_aruco_str);
  if (params.state_options.feat_rep_msckf == LandmarkRepresentation::Representation::UNKNOWN ||
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

  // Enforce that we have enough cameras to run
  if (params.state_options.num_cameras < 1) {
    printf(RED "VioManager(): Specified number of cameras needs to be greater than zero\n" RESET);
    printf(RED "VioManager(): num cameras = %d\n" RESET, params.state_options.num_cameras);
    std::exit(EXIT_FAILURE);
  }

  //====================================================================================
  //====================================================================================
  //====================================================================================

  // Create our command line parser for the cameras
  // NOTE: we need to first parse how many cameras we have before we can parse this
  CLI::App app2{"parser_cmd_02"};
  app2.allow_extras();

  // Set the defaults
  std::vector<int> p_fish;
  std::vector<std::vector<double>> p_intrinsic;
  std::vector<std::vector<double>> p_extrinsic;
  std::vector<std::vector<int>> p_wh;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    p_fish.push_back(false);
    p_intrinsic.push_back({458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05});
    p_extrinsic.push_back({0, 0, 0, 1, 0, 0, 0});
    p_wh.push_back({752, 480});
    app2.add_option("--cam" + std::to_string(i) + "_fisheye", p_fish.at(i));
    app2.add_option("--cam" + std::to_string(i) + "_intrinsic", p_intrinsic.at(i), "");
    app2.add_option("--cam" + std::to_string(i) + "_extrinsic", p_extrinsic.at(i), "");
    app2.add_option("--cam" + std::to_string(i) + "_wh", p_wh.at(i), "");
  }

  // Finally actually parse the command line and load it
  try {
    app2.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    std::exit(app2.exit(e));
  }

  // Finally load it into our params
  for (int i = 0; i < params.state_options.num_cameras; i++) {

    // Halve if we are doing downsampling
    p_wh.at(i).at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
    p_wh.at(i).at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
    p_intrinsic.at(i).at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
    p_intrinsic.at(i).at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
    p_intrinsic.at(i).at(2) /= (params.downsample_cameras) ? 2.0 : 1.0;
    p_intrinsic.at(i).at(3) /= (params.downsample_cameras) ? 2.0 : 1.0;

    // Convert to Eigen
    assert(p_intrinsic.at(i).size() == 8);
    Eigen::Matrix<double, 8, 1> intrinsics;
    intrinsics << p_intrinsic.at(i).at(0), p_intrinsic.at(i).at(1), p_intrinsic.at(i).at(2), p_intrinsic.at(i).at(3),
        p_intrinsic.at(i).at(4), p_intrinsic.at(i).at(5), p_intrinsic.at(i).at(6), p_intrinsic.at(i).at(7);
    assert(p_extrinsic.at(i).size() == 7);
    Eigen::Matrix<double, 7, 1> extrinsics;
    extrinsics << p_extrinsic.at(i).at(0), p_extrinsic.at(i).at(1), p_extrinsic.at(i).at(2), p_extrinsic.at(i).at(3),
        p_extrinsic.at(i).at(4), p_extrinsic.at(i).at(5), p_extrinsic.at(i).at(6);
    assert(p_wh.at(i).size() == 2);

    // Insert
    params.camera_fisheye.insert({i, p_fish.at(i)});
    params.camera_intrinsics.insert({i, intrinsics});
    params.camera_extrinsics.insert({i, extrinsics});
    params.camera_wh.insert({i, {p_wh.at(i).at(0), p_wh.at(i).at(1)}});
  }

  // Success, lets returned the parsed options
  return params;
}

} // namespace ov_msckf

#endif // OV_MSCKF_PARSE_CMDLINE_H