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

#ifndef OV_INIT_INERTIALINITIALIZEROPTIONS_H
#define OV_INIT_INERTIALINITIALIZEROPTIONS_H

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "feat/FeatureInitializerOptions.h"
#include "track/TrackBase.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_init {

/**
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and simulation.
 * If you are going to add a parameter here you will need to add it to the parsers.
 * You will also need to add it to the print statement at the bottom of each.
 */
struct InertialInitializerOptions {

  /**
   * @brief This function will load the non-simulation parameters of the system and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    print_and_load_initializer(parser);
    print_and_load_noise(parser);
    print_and_load_state(parser);
  }

  // CALIBRATION ============================

  /// If we should only do dynamic state initialization
  bool init_only_use_dynamic = false;

  /// If we should only do static state initialization
  bool init_only_use_static = false;

  /// Bool to determine whether or not to recover imu-to-camera pose
  bool init_calib_camera_pose = false;

  /// Bool to determine whether or not to recover camera to IMU time offset
  bool init_calib_camera_timeoffset = false;

  /// Bool to determine whether or not to recover acceleration bias
  bool init_state_bias_accel = true;

  /// Bool to determine whether or not to enforce gravity magnitude
  bool init_enforce_gravity_magnitude = true;

  /// What camera rate we should track features at
  double init_camera_rate = 10.0;

  /// Amount of time we will initialize over (seconds)
  double init_window_time = 1.0;

  /// Variance threshold on our acceleration to be classified as moving
  double init_imu_thresh = 1.0;

  /// Max disparity we will consider the unit to be stationary
  double init_max_disparity = 1.0;

  /// Number of features we should try to track
  int init_max_features = 20;

  /**
   * @brief This function will load print out all initializer settings loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_initializer(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("INITIALIZATION SETTINGS:\n");
    if (parser != nullptr) {
      parser->parse_config("init_only_use_dynamic", init_only_use_dynamic);
      parser->parse_config("init_only_use_static", init_only_use_static);
      parser->parse_config("init_camera_pose", init_calib_camera_pose);
      parser->parse_config("init_camera_timeoffset", init_calib_camera_timeoffset);
      parser->parse_config("init_state_bias_accel", init_state_bias_accel);
      parser->parse_config("init_enforce_gravity_magnitude", init_enforce_gravity_magnitude);
      parser->parse_config("init_camera_rate", init_camera_rate);
      parser->parse_config("init_window_time", init_window_time);
      parser->parse_config("init_imu_thresh", init_imu_thresh);
      parser->parse_config("init_max_disparity", init_max_disparity);
      parser->parse_config("init_max_features", init_max_features);
    }
    PRINT_DEBUG("  - init_only_use_dynamic: %d\n", init_only_use_dynamic);
    PRINT_DEBUG("  - init_only_use_static: %d\n", init_only_use_static);
    if (init_only_use_dynamic && init_only_use_static) {
      PRINT_ERROR(RED "cannot only use static and dynamic initialization!\n" RESET);
      PRINT_ERROR(RED "  init_only_use_dynamic = %d\n" RESET, init_only_use_dynamic);
      PRINT_ERROR(RED "  init_only_use_dynamic = %d\n" RESET, init_only_use_static);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - init_calib_camera_pose: %d\n", init_calib_camera_pose);
    PRINT_DEBUG("  - init_calib_camera_timeoffset: %d\n", init_calib_camera_timeoffset);
    PRINT_DEBUG("  - init_state_ba: %d\n", init_state_bias_accel);
    PRINT_DEBUG("  - init_enforce_gravity_magnitude: %d\n", init_enforce_gravity_magnitude);
    PRINT_DEBUG("  - init_camera_rate: %.2f\n", init_camera_rate);
    PRINT_DEBUG("  - init_window_time: %.2f\n", init_window_time);
    PRINT_DEBUG("  - init_imu_thresh: %.2f\n", init_imu_thresh);
    PRINT_DEBUG("  - init_max_disparity: %.2f\n", init_max_disparity);
    PRINT_DEBUG("  - init_max_features: %.2f\n", init_max_features);
    // Ensure we have enough frames to work with
    double dt = 1.0 / init_camera_rate;
    int num_frames = std::floor(init_window_time / dt);
    if (num_frames < 4) {
      PRINT_ERROR(RED "number of requested frames to init not enough!!\n" RESET);
      PRINT_ERROR(RED "  init_camera_rate = %.2f (%.2f seconds)\n" RESET, init_camera_rate, dt);
      PRINT_ERROR(RED "  init_window_time = %.2f seconds\n" RESET, init_window_time);
      PRINT_ERROR(RED "  num init frames = %d\n" RESET, num_frames);
      std::exit(EXIT_FAILURE);
    }
    if (init_max_features < 15) {
      PRINT_ERROR(RED "number of requested feature tracks to init not enough!!\n" RESET);
      PRINT_ERROR(RED "  init_max_features = %d\n" RESET, init_max_features);
      std::exit(EXIT_FAILURE);
    }
  }

  // NOISE / CHI2 ============================

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;
  double sigma_w_2 = pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;
  double sigma_wb_2 = pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;
  double sigma_a_2 = pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;
  double sigma_ab_2 = pow(3.0000e-03, 2);

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;
  double sigma_pix_sq = 1;

  /**
   * @brief This function will load print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_noise(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("NOISE PARAMETERS:\n");
    if (parser != nullptr) {
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_noise_density", sigma_w);
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_random_walk", sigma_wb);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_noise_density", sigma_a);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_random_walk", sigma_ab);
      sigma_w_2 = std::pow(sigma_w, 2);
      sigma_wb_2 = std::pow(sigma_wb, 2);
      sigma_a_2 = std::pow(sigma_a, 2);
      sigma_ab_2 = std::pow(sigma_ab, 2);
      parser->parse_config("up_slam_sigma_px", sigma_pix);
      sigma_pix_sq = std::pow(sigma_pix, 2);
    }
    PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
    PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
    PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
    PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
    PRINT_DEBUG("  - sigma_pix: %.2f\n", sigma_pix);
  }

  // STATE DEFAULTS ==========================

  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// Number of distinct cameras that we will observe features in
  int num_cameras = 1;

//  /// Time offset between camera and IMU.
//  double calib_camimu_dt = 0.0;
//
//  /// Map between camid and camera model (true=fisheye, false=radtan)
//  std::map<size_t, bool> camera_fisheye;
//
//  /// Map between camid and intrinsics. Values depends on the model but each should be a 4x1 vector normally.
//  std::map<size_t, Eigen::VectorXd> camera_intrinsics;
//
//  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
//  std::map<size_t, Eigen::VectorXd> camera_extrinsics;
//
//  /// Map between camid and the dimensions of incoming images (width/cols, height/rows). This is normally only used during simulation.
//  std::map<size_t, std::pair<int, int>> camera_wh;
//
//  /// If we should try to load a mask and use it to reject invalid features
//  bool use_mask = false;
//
//  /// Mask images for each camera
//  std::map<size_t, cv::Mat> masks;

  /**
   * @brief This function will load and print all state parameters (e.g. sensor extrinsics)
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_state(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("gravity_mag", gravity_mag);
      parser->parse_config("max_cameras", num_cameras); // might be redundant
//      for (int i = 0; i < num_cameras; i++) {
//
//        // Time offset (use the first one)
//        // TODO: support multiple time offsets between cameras
//        if (i == 0) {
//          parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "timeshift_cam_imu", calib_camimu_dt, false);
//        }
//
//        // Distortion model
//        std::string dist_model = "radtan";
//        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_model", dist_model);
//
//        // Distortion parameters
//        std::vector<double> cam_calib1 = {1, 1, 0, 0};
//        std::vector<double> cam_calib2 = {0, 0, 0, 0};
//        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "intrinsics", cam_calib1);
//        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_coeffs", cam_calib2);
//        Eigen::VectorXd cam_calib = Eigen::VectorXd::Zero(8);
//        cam_calib << cam_calib1.at(0), cam_calib1.at(1), cam_calib1.at(2), cam_calib1.at(3), cam_calib2.at(0), cam_calib2.at(1),
//            cam_calib2.at(2), cam_calib2.at(3);
//        cam_calib(0) /= (downsample_cameras) ? 2.0 : 1.0;
//        cam_calib(1) /= (downsample_cameras) ? 2.0 : 1.0;
//        cam_calib(2) /= (downsample_cameras) ? 2.0 : 1.0;
//        cam_calib(3) /= (downsample_cameras) ? 2.0 : 1.0;
//
//        // FOV / resolution
//        std::vector<int> matrix_wh = {1, 1};
//        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "resolution", matrix_wh);
//        matrix_wh.at(0) /= (downsample_cameras) ? 2.0 : 1.0;
//        matrix_wh.at(1) /= (downsample_cameras) ? 2.0 : 1.0;
//        std::pair<int, int> wh(matrix_wh.at(0), matrix_wh.at(1));
//
//        // Extrinsics
//        Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
//        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "T_imu_cam", T_CtoI);
//
//        // Load these into our state
//        Eigen::Matrix<double, 7, 1> cam_eigen;
//        cam_eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
//        cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
//
//        // Insert
//        camera_fisheye.insert({i, dist_model == "equidistant"});
//        camera_intrinsics.insert({i, cam_calib});
//        camera_extrinsics.insert({i, cam_eigen});
//        camera_wh.insert({i, wh});
//      }
//      parser->parse_config("use_mask", use_mask);
//      if (use_mask) {
//        for (int i = 0; i < num_cameras; i++) {
//          std::string mask_path;
//          std::string mask_node = "mask" + std::to_string(i);
//          parser->parse_config(mask_node, mask_path);
//          std::string total_mask_path = parser->get_config_folder() + mask_path;
//          if (!boost::filesystem::exists(total_mask_path)) {
//            PRINT_ERROR(RED "Initializer(): invalid mask path:\n" RESET);
//            PRINT_ERROR(RED "\t- mask%d - %s\n" RESET, i, total_mask_path.c_str());
//            std::exit(EXIT_FAILURE);
//          }
//          masks.insert({i, cv::imread(total_mask_path, cv::IMREAD_GRAYSCALE)});
//        }
//      }
    }
    PRINT_DEBUG("STATE PARAMETERS:\n");
    PRINT_DEBUG("  - gravity_mag: %.4f\n", gravity_mag);
    PRINT_DEBUG("  - gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
    PRINT_DEBUG("  - num_cameras: %d\n", num_cameras);
//    PRINT_DEBUG("  - calib_camimu_dt: %.4f\n", calib_camimu_dt);
//    PRINT_DEBUG("  - camera masks?: %d\n", use_mask);
//    assert(num_cameras == (int)camera_fisheye.size());
//    for (int n = 0; n < num_cameras; n++) {
//      std::stringstream ss;
//      ss << "cam_" << n << "_fisheye:" << camera_fisheye.at(n) << std::endl;
//      ss << "cam_" << n << "_wh:" << std::endl << camera_wh.at(n).first << " x " << camera_wh.at(n).second << std::endl;
//      ss << "cam_" << n << "_intrinsic(0:3):" << std::endl << camera_intrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
//      ss << "cam_" << n << "_intrinsic(4:7):" << std::endl << camera_intrinsics.at(n).block(4, 0, 4, 1).transpose() << std::endl;
//      ss << "cam_" << n << "_extrinsic(0:3):" << std::endl << camera_extrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
//      ss << "cam_" << n << "_extrinsic(4:6):" << std::endl << camera_extrinsics.at(n).block(4, 0, 3, 1).transpose() << std::endl;
//      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
//      T_CtoI.block(0, 0, 3, 3) = ov_core::quat_2_Rot(camera_extrinsics.at(n).block(0, 0, 4, 1)).transpose();
//      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * camera_extrinsics.at(n).block(4, 0, 3, 1);
//      ss << "T_C" << n << "toI:" << std::endl << T_CtoI << std::endl << std::endl;
//      PRINT_DEBUG(ss.str().c_str());
//    }
  }

  // SIMULATOR ===============================

  /// Seed for initial states (i.e. random feature 3d positions in the generated map)
  int sim_seed_state_init = 0;

  /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are enabled.
  int sim_seed_preturb = 0;

  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements,
  /// but diffferent noise values.
  int sim_seed_measurements = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  std::string sim_traj_path = "../ov_data/sim/udel_gore.txt";

  /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init from groundtruth in
  /// simulation.
  double sim_distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Feature distance we generate features from (minimum)
  double sim_min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  double sim_max_feature_gen_distance = 10;

  /**
   * @brief This function will load print out all simulated parameters.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_simulation(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("sim_seed_state_init", sim_seed_state_init);
      parser->parse_config("sim_seed_preturb", sim_seed_preturb);
      parser->parse_config("sim_seed_measurements", sim_seed_measurements);
      parser->parse_config("sim_do_perturbation", sim_do_perturbation);
      parser->parse_config("sim_traj_path", sim_traj_path);
      parser->parse_config("sim_distance_threshold", sim_distance_threshold);
      parser->parse_config("sim_freq_cam", sim_freq_cam);
      parser->parse_config("sim_freq_imu", sim_freq_imu);
      parser->parse_config("sim_min_feature_gen_dist", sim_min_feature_gen_distance);
      parser->parse_config("sim_max_feature_gen_dist", sim_max_feature_gen_distance);
    }
    PRINT_DEBUG("SIMULATION PARAMETERS:\n");
    PRINT_WARNING(BOLDRED "  - state init seed: %d \n" RESET, sim_seed_state_init);
    PRINT_WARNING(BOLDRED "  - perturb seed: %d \n" RESET, sim_seed_preturb);
    PRINT_WARNING(BOLDRED "  - measurement seed: %d \n" RESET, sim_seed_measurements);
    PRINT_WARNING(BOLDRED "  - do perturb?: %d\n" RESET, sim_do_perturbation);
    PRINT_DEBUG("  - traj path: %s\n", sim_traj_path.c_str());
    PRINT_DEBUG("  - dist thresh: %.2f\n", sim_distance_threshold);
    PRINT_DEBUG("  - cam feq: %.2f\n", sim_freq_cam);
    PRINT_DEBUG("  - imu feq: %.2f\n", sim_freq_imu);
    PRINT_DEBUG("  - min feat dist: %.2f\n", sim_min_feature_gen_distance);
    PRINT_DEBUG("  - max feat dist: %.2f\n", sim_max_feature_gen_distance);
  }
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZEROPTIONS_H