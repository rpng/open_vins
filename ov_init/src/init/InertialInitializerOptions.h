/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
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

  // INITIALIZATION ============================

  /// Amount of time we will initialize over (seconds)
  double init_window_time = 1.0;

  /// Variance threshold on our acceleration to be classified as moving
  double init_imu_thresh = 1.0;

  /// Max disparity we will consider the unit to be stationary
  double init_max_disparity = 1.0;

  /// Number of features we should try to track
  int init_max_features = 50;

  /// If we should perform dynamic initialization
  bool init_dyn_use = false;

  /// If we should optimize and recover the calibration in our MLE
  bool init_dyn_mle_opt_calib = false;

  /// Max number of MLE iterations for dynamic initialization
  int init_dyn_mle_max_iter = 20;

  /// Max number of MLE threads for dynamic initialization
  int init_dyn_mle_max_threads = 20;

  /// Max time for MLE optimization (seconds)
  double init_dyn_mle_max_time = 5.0;

  /// Number of poses to use during initialization (max should be cam freq * window)
  int init_dyn_num_pose = 5;

  /// Minimum degrees we need to rotate before we try to init (sum of norm)
  double init_dyn_min_deg = 45.0;

  /// Magnitude we will inflate initial covariance of orientation
  double init_dyn_inflation_orientation = 10.0;

  /// Magnitude we will inflate initial covariance of velocity
  double init_dyn_inflation_velocity = 10.0;

  /// Magnitude we will inflate initial covariance of gyroscope bias
  double init_dyn_inflation_bias_gyro = 100.0;

  /// Magnitude we will inflate initial covariance of accelerometer bias
  double init_dyn_inflation_bias_accel = 100.0;

  /// Minimum reciprocal condition number acceptable for our covariance recovery (min_sigma / max_sigma <
  /// sqrt(min_reciprocal_condition_number))
  double init_dyn_min_rec_cond = 1e-15;

  /// Initial IMU gyroscope bias values for dynamic initialization (will be optimized)
  Eigen::Vector3d init_dyn_bias_g = Eigen::Vector3d::Zero();

  /// Initial IMU accelerometer bias values for dynamic initialization (will be optimized)
  Eigen::Vector3d init_dyn_bias_a = Eigen::Vector3d::Zero();

  /**
   * @brief This function will load print out all initializer settings loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_initializer(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("INITIALIZATION SETTINGS:\n");
    if (parser != nullptr) {
      parser->parse_config("init_window_time", init_window_time);
      parser->parse_config("init_imu_thresh", init_imu_thresh);
      parser->parse_config("init_max_disparity", init_max_disparity);
      parser->parse_config("init_max_features", init_max_features);
      parser->parse_config("init_dyn_use", init_dyn_use);
      parser->parse_config("init_dyn_mle_opt_calib", init_dyn_mle_opt_calib);
      parser->parse_config("init_dyn_mle_max_iter", init_dyn_mle_max_iter);
      parser->parse_config("init_dyn_mle_max_threads", init_dyn_mle_max_threads);
      parser->parse_config("init_dyn_mle_max_time", init_dyn_mle_max_time);
      parser->parse_config("init_dyn_num_pose", init_dyn_num_pose);
      parser->parse_config("init_dyn_min_deg", init_dyn_min_deg);
      parser->parse_config("init_dyn_inflation_ori", init_dyn_inflation_orientation);
      parser->parse_config("init_dyn_inflation_vel", init_dyn_inflation_velocity);
      parser->parse_config("init_dyn_inflation_bg", init_dyn_inflation_bias_gyro);
      parser->parse_config("init_dyn_inflation_ba", init_dyn_inflation_bias_accel);
      parser->parse_config("init_dyn_min_rec_cond", init_dyn_min_rec_cond);
      std::vector<double> bias_g = {0, 0, 0};
      std::vector<double> bias_a = {0, 0, 0};
      parser->parse_config("init_dyn_bias_g", bias_g);
      parser->parse_config("init_dyn_bias_a", bias_a);
      init_dyn_bias_g << bias_g.at(0), bias_g.at(1), bias_g.at(2);
      init_dyn_bias_a << bias_a.at(0), bias_a.at(1), bias_a.at(2);
    }
    PRINT_DEBUG("  - init_window_time: %.2f\n", init_window_time);
    PRINT_DEBUG("  - init_imu_thresh: %.2f\n", init_imu_thresh);
    PRINT_DEBUG("  - init_max_disparity: %.2f\n", init_max_disparity);
    PRINT_DEBUG("  - init_max_features: %.2f\n", init_max_features);
    if (init_max_features < 15) {
      PRINT_ERROR(RED "number of requested feature tracks to init not enough!!\n" RESET);
      PRINT_ERROR(RED "  init_max_features = %d\n" RESET, init_max_features);
      std::exit(EXIT_FAILURE);
    }
    if (init_imu_thresh <= 0.0 && !init_dyn_use) {
      PRINT_ERROR(RED "need to have an IMU threshold for static initialization!\n" RESET);
      PRINT_ERROR(RED "  init_imu_thresh = %.3f\n" RESET, init_imu_thresh);
      PRINT_ERROR(RED "  init_dyn_use = %d\n" RESET, init_dyn_use);
      std::exit(EXIT_FAILURE);
    }
    if (init_max_disparity <= 0.0 && !init_dyn_use) {
      PRINT_ERROR(RED "need to have an DISPARITY threshold for static initialization!\n" RESET);
      PRINT_ERROR(RED "  init_max_disparity = %.3f\n" RESET, init_max_disparity);
      PRINT_ERROR(RED "  init_dyn_use = %d\n" RESET, init_dyn_use);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - init_dyn_use: %d\n", init_dyn_use);
    PRINT_DEBUG("  - init_dyn_mle_opt_calib: %d\n", init_dyn_mle_opt_calib);
    PRINT_DEBUG("  - init_dyn_mle_max_iter: %d\n", init_dyn_mle_max_iter);
    PRINT_DEBUG("  - init_dyn_mle_max_threads: %d\n", init_dyn_mle_max_threads);
    PRINT_DEBUG("  - init_dyn_mle_max_time: %.2f\n", init_dyn_mle_max_time);
    PRINT_DEBUG("  - init_dyn_num_pose: %d\n", init_dyn_num_pose);
    PRINT_DEBUG("  - init_dyn_min_deg: %.2f\n", init_dyn_min_deg);
    PRINT_DEBUG("  - init_dyn_inflation_ori: %.2e\n", init_dyn_inflation_orientation);
    PRINT_DEBUG("  - init_dyn_inflation_vel: %.2e\n", init_dyn_inflation_velocity);
    PRINT_DEBUG("  - init_dyn_inflation_bg: %.2e\n", init_dyn_inflation_bias_gyro);
    PRINT_DEBUG("  - init_dyn_inflation_ba: %.2e\n", init_dyn_inflation_bias_accel);
    PRINT_DEBUG("  - init_dyn_min_rec_cond: %.2e\n", init_dyn_min_rec_cond);
    if (init_dyn_num_pose < 4) {
      PRINT_ERROR(RED "number of requested frames to init not enough!!\n" RESET);
      PRINT_ERROR(RED "  init_dyn_num_pose = %d (4 min)\n" RESET, init_dyn_num_pose);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - init_dyn_bias_g: %.2f, %.2f, %.2f\n", init_dyn_bias_g(0), init_dyn_bias_g(1), init_dyn_bias_g(2));
    PRINT_DEBUG("  - init_dyn_bias_a: %.2f, %.2f, %.2f\n", init_dyn_bias_a(0), init_dyn_bias_a(1), init_dyn_bias_a(2));
  }

  // NOISE / CHI2 ============================

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

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
      parser->parse_config("up_slam_sigma_px", sigma_pix);
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

  /// If we should process two cameras are being stereo or binocular. If binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// Will half the resolution all tracking image (aruco will be 1/4 instead of halved if dowsize_aruoc also enabled)
  bool downsample_cameras = false;

  /// Time offset between camera and IMU (t_imu = t_cam + t_off)
  double calib_camimu_dt = 0.0;

  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w, cam_h)
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> camera_intrinsics;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, Eigen::VectorXd> camera_extrinsics;

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
      parser->parse_config("use_stereo", use_stereo);
      parser->parse_config("downsample_cameras", downsample_cameras);
      for (int i = 0; i < num_cameras; i++) {

        // Time offset (use the first one)
        // TODO: support multiple time offsets between cameras
        if (i == 0) {
          parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "timeshift_cam_imu", calib_camimu_dt, false);
        }

        // Distortion model
        std::string dist_model = "radtan";
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_model", dist_model);

        // Distortion parameters
        std::vector<double> cam_calib1 = {1, 1, 0, 0};
        std::vector<double> cam_calib2 = {0, 0, 0, 0};
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "intrinsics", cam_calib1);
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_coeffs", cam_calib2);
        Eigen::VectorXd cam_calib = Eigen::VectorXd::Zero(8);
        cam_calib << cam_calib1.at(0), cam_calib1.at(1), cam_calib1.at(2), cam_calib1.at(3), cam_calib2.at(0), cam_calib2.at(1),
            cam_calib2.at(2), cam_calib2.at(3);
        cam_calib(0) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(1) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(2) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(3) /= (downsample_cameras) ? 2.0 : 1.0;

        // FOV / resolution
        std::vector<int> matrix_wh = {1, 1};
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "resolution", matrix_wh);
        matrix_wh.at(0) /= (downsample_cameras) ? 2.0 : 1.0;
        matrix_wh.at(1) /= (downsample_cameras) ? 2.0 : 1.0;

        // Extrinsics
        Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "T_imu_cam", T_CtoI);

        // Load these into our state
        Eigen::Matrix<double, 7, 1> cam_eigen;
        cam_eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
        cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);

        // Create intrinsics model
        if (dist_model == "equidistant") {
          camera_intrinsics.insert({i, std::make_shared<ov_core::CamEqui>(matrix_wh.at(0), matrix_wh.at(1))});
          camera_intrinsics.at(i)->set_value(cam_calib);
        } else {
          camera_intrinsics.insert({i, std::make_shared<ov_core::CamRadtan>(matrix_wh.at(0), matrix_wh.at(1))});
          camera_intrinsics.at(i)->set_value(cam_calib);
        }
        camera_extrinsics.insert({i, cam_eigen});
      }
    }
    PRINT_DEBUG("STATE PARAMETERS:\n");
    PRINT_DEBUG("  - gravity_mag: %.4f\n", gravity_mag);
    PRINT_DEBUG("  - gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
    PRINT_DEBUG("  - num_cameras: %d\n", num_cameras);
    PRINT_DEBUG("  - use_stereo: %d\n", use_stereo);
    PRINT_DEBUG("  - downsize cameras: %d\n", downsample_cameras);
    if (num_cameras != (int)camera_intrinsics.size() || num_cameras != (int)camera_extrinsics.size()) {
      PRINT_ERROR(RED "[SIM]: camera calib size does not match max cameras...\n" RESET);
      PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras (camera_intrinsics)\n" RESET, (int)camera_intrinsics.size(), num_cameras);
      PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras (camera_extrinsics)\n" RESET, (int)camera_extrinsics.size(), num_cameras);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - calib_camimu_dt: %.4f\n", calib_camimu_dt);
    for (int n = 0; n < num_cameras; n++) {
      std::stringstream ss;
      ss << "cam_" << n << "_fisheye:" << (std::dynamic_pointer_cast<ov_core::CamEqui>(camera_intrinsics.at(n)) != nullptr) << std::endl;
      ss << "cam_" << n << "_wh:" << std::endl << camera_intrinsics.at(n)->w() << " x " << camera_intrinsics.at(n)->h() << std::endl;
      ss << "cam_" << n << "_intrinsic(0:3):" << std::endl
         << camera_intrinsics.at(n)->get_value().block(0, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_intrinsic(4:7):" << std::endl
         << camera_intrinsics.at(n)->get_value().block(4, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_extrinsic(0:3):" << std::endl << camera_extrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_extrinsic(4:6):" << std::endl << camera_extrinsics.at(n).block(4, 0, 3, 1).transpose() << std::endl;
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = ov_core::quat_2_Rot(camera_extrinsics.at(n).block(0, 0, 4, 1)).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * camera_extrinsics.at(n).block(4, 0, 3, 1);
      ss << "T_C" << n << "toI:" << std::endl << T_CtoI << std::endl << std::endl;
      PRINT_DEBUG(ss.str().c_str());
    }
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