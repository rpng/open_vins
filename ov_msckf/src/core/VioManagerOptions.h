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

#ifndef OV_MSCKF_VIOMANAGEROPTIONS_H
#define OV_MSCKF_VIOMANAGEROPTIONS_H

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "state/Propagator.h"
#include "state/StateOptions.h"
#include "update/UpdaterOptions.h"

#include "init/InertialInitializerOptions.h"

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "feat/FeatureInitializerOptions.h"
#include "track/TrackBase.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_msckf {

/**
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and simulation.
 * If you are going to add a parameter here you will need to add it to the parsers.
 * You will also need to add it to the print statement at the bottom of each.
 */
struct VioManagerOptions {

  /**
   * @brief This function will load the non-simulation parameters of the system and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    print_and_load_estimator(parser);
    print_and_load_noise(parser);
    print_and_load_state(parser);
    print_and_load_trackers(parser);
  }

  // ESTIMATOR ===============================

  /// Core state options (e.g. number of cameras, use fej, stereo, what calibration to enable etc)
  StateOptions state_options;

  /// Our state initialization options (e.g. window size, num features, if we should get the calibration)
  ov_init::InertialInitializerOptions init_options;

  /// Delay, in seconds, that we should wait from init before we start estimating SLAM features
  double dt_slam_delay = 2.0;

  /// If we should try to use zero velocity update
  bool try_zupt = false;

  /// Max velocity we will consider to try to do a zupt (i.e. if above this, don't do zupt)
  double zupt_max_velocity = 1.0;

  /// Multiplier of our zupt measurement IMU noise matrix (default should be 1.0)
  double zupt_noise_multiplier = 1.0;

  /// Max disparity we will consider to try to do a zupt (i.e. if above this, don't do zupt)
  double zupt_max_disparity = 1.0;

  /// If we should only use the zupt at the very beginning static initialization phase
  bool zupt_only_at_beginning = false;

  /// If we should record the timing performance to file
  bool record_timing_information = false;

  /// The path to the file we will record the timing information into
  std::string record_timing_filepath = "ov_msckf_timing.txt";

  /**
   * @brief This function will load print out all estimator settings loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_estimator(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("ESTIMATOR PARAMETERS:\n");
    state_options.print(parser);
    init_options.print_and_load(parser);
    if (parser != nullptr) {
      parser->parse_config("dt_slam_delay", dt_slam_delay);
      parser->parse_config("try_zupt", try_zupt);
      parser->parse_config("zupt_max_velocity", zupt_max_velocity);
      parser->parse_config("zupt_noise_multiplier", zupt_noise_multiplier);
      parser->parse_config("zupt_max_disparity", zupt_max_disparity);
      parser->parse_config("zupt_only_at_beginning", zupt_only_at_beginning);
      parser->parse_config("record_timing_information", record_timing_information);
      parser->parse_config("record_timing_filepath", record_timing_filepath);
    }
    PRINT_DEBUG("  - dt_slam_delay: %.1f\n", dt_slam_delay);
    PRINT_DEBUG("  - zero_velocity_update: %d\n", try_zupt);
    PRINT_DEBUG("  - zupt_max_velocity: %.2f\n", zupt_max_velocity);
    PRINT_DEBUG("  - zupt_noise_multiplier: %.2f\n", zupt_noise_multiplier);
    PRINT_DEBUG("  - zupt_max_disparity: %.4f\n", zupt_max_disparity);
    PRINT_DEBUG("  - zupt_only_at_beginning?: %d\n", zupt_only_at_beginning);
    PRINT_DEBUG("  - record timing?: %d\n", (int)record_timing_information);
    PRINT_DEBUG("  - record timing filepath: %s\n", record_timing_filepath.c_str());
  }

  // NOISE / CHI2 ============================

  /// IMU noise (gyroscope and accelerometer)
  Propagator::NoiseManager imu_noises;

  /// Update options for MSCKF features (pixel noise and chi2 multiplier)
  UpdaterOptions msckf_options;

  /// Update options for SLAM features (pixel noise and chi2 multiplier)
  UpdaterOptions slam_options;

  /// Update options for ARUCO features (pixel noise and chi2 multiplier)
  UpdaterOptions aruco_options;

  /// Update options for zero velocity (chi2 multiplier)
  UpdaterOptions zupt_options;

  /**
   * @brief This function will load print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_noise(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("NOISE PARAMETERS:\n");
    if (parser != nullptr) {
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_noise_density", imu_noises.sigma_w);
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_random_walk", imu_noises.sigma_wb);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_noise_density", imu_noises.sigma_a);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_random_walk", imu_noises.sigma_ab);
      imu_noises.sigma_w_2 = std::pow(imu_noises.sigma_w, 2);
      imu_noises.sigma_wb_2 = std::pow(imu_noises.sigma_wb, 2);
      imu_noises.sigma_a_2 = std::pow(imu_noises.sigma_a, 2);
      imu_noises.sigma_ab_2 = std::pow(imu_noises.sigma_ab, 2);
    }
    imu_noises.print();
    if (parser != nullptr) {
      parser->parse_config("up_msckf_sigma_px", msckf_options.sigma_pix);
      parser->parse_config("up_msckf_chi2_multipler", msckf_options.chi2_multipler);
      parser->parse_config("up_slam_sigma_px", slam_options.sigma_pix);
      parser->parse_config("up_slam_chi2_multipler", slam_options.chi2_multipler);
      parser->parse_config("up_aruco_sigma_px", aruco_options.sigma_pix);
      parser->parse_config("up_aruco_chi2_multipler", aruco_options.chi2_multipler);
      msckf_options.sigma_pix_sq = std::pow(msckf_options.sigma_pix, 2);
      slam_options.sigma_pix_sq = std::pow(slam_options.sigma_pix, 2);
      aruco_options.sigma_pix_sq = std::pow(aruco_options.sigma_pix, 2);
      parser->parse_config("zupt_chi2_multipler", zupt_options.chi2_multipler);
    }
    PRINT_DEBUG("  Updater MSCKF Feats:\n");
    msckf_options.print();
    PRINT_DEBUG("  Updater SLAM Feats:\n");
    slam_options.print();
    PRINT_DEBUG("  Updater ARUCO Tags:\n");
    aruco_options.print();
    PRINT_DEBUG("  Updater ZUPT:\n");
    zupt_options.print();
  }

  // STATE DEFAULTS ==========================

  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// Time offset between camera and IMU.
  double calib_camimu_dt = 0.0;

  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w, cam_h)
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> camera_intrinsics;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, Eigen::VectorXd> camera_extrinsics;

  /// If we should try to load a mask and use it to reject invalid features
  bool use_mask = false;

  /// Mask images for each camera
  std::map<size_t, cv::Mat> masks;

  /**
   * @brief This function will load and print all state parameters (e.g. sensor extrinsics)
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_state(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("gravity_mag", gravity_mag);
      parser->parse_config("max_cameras", state_options.num_cameras); // might be redundant
      parser->parse_config("downsample_cameras", downsample_cameras); // might be redundant
      for (int i = 0; i < state_options.num_cameras; i++) {

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
        std::pair<int, int> wh(matrix_wh.at(0), matrix_wh.at(1));

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
      parser->parse_config("use_mask", use_mask);
      if (use_mask) {
        for (int i = 0; i < state_options.num_cameras; i++) {
          std::string mask_path;
          std::string mask_node = "mask" + std::to_string(i);
          parser->parse_config(mask_node, mask_path);
          std::string total_mask_path = parser->get_config_folder() + mask_path;
          if (!boost::filesystem::exists(total_mask_path)) {
            PRINT_ERROR(RED "VioManager(): invalid mask path:\n" RESET);
            PRINT_ERROR(RED "\t- mask%d - %s\n" RESET, i, total_mask_path.c_str());
            std::exit(EXIT_FAILURE);
          }
          masks.insert({i, cv::imread(total_mask_path, cv::IMREAD_GRAYSCALE)});
        }
      }
    }
    PRINT_DEBUG("STATE PARAMETERS:\n");
    PRINT_DEBUG("  - gravity_mag: %.4f\n", gravity_mag);
    PRINT_DEBUG("  - gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
    PRINT_DEBUG("  - camera masks?: %d\n", use_mask);
    if (state_options.num_cameras != (int)camera_intrinsics.size() || state_options.num_cameras != (int)camera_extrinsics.size()) {
      PRINT_ERROR(RED "[SIM]: camera calib size does not match max cameras...\n" RESET);
      PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras (camera_intrinsics)\n" RESET, (int)camera_intrinsics.size(),
                  state_options.num_cameras);
      PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras (camera_extrinsics)\n" RESET, (int)camera_extrinsics.size(),
                  state_options.num_cameras);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - calib_camimu_dt: %.4f\n", calib_camimu_dt);
    for (int n = 0; n < state_options.num_cameras; n++) {
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

  // TRACKERS ===============================

  /// If we should process two cameras are being stereo or binocular. If binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// If we should use KLT tracking, or descriptor matcher
  bool use_klt = true;

  /// If should extract aruco tags and estimate them
  bool use_aruco = true;

  /// Will half the resolution of the aruco tag image (will be faster)
  bool downsize_aruco = true;

  /// Will half the resolution all tracking image (aruco will be 1/4 instead of halved if dowsize_aruoc also enabled)
  bool downsample_cameras = false;

  /// If our front-end should try to use some multi-threading for stereo matching
  bool use_multi_threading = true;

  /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
  int num_pts = 150;

  /// Fast extraction threshold
  int fast_threshold = 20;

  /// Number of grids we should split column-wise to do feature extraction in
  int grid_x = 5;

  /// Number of grids we should split row-wise to do feature extraction in
  int grid_y = 5;

  /// Will check after doing KLT track and remove any features closer than this
  int min_px_dist = 10;

  /// What type of pre-processing histogram method should be applied to images
  ov_core::TrackBase::HistogramMethod histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  /// KNN ration between top two descriptor matcher which is required to be a good match
  double knn_ratio = 0.85;

  /// Parameters used by our feature initialize / triangulator
  ov_core::FeatureInitializerOptions featinit_options;

  /**
   * @brief This function will load print out all parameters related to visual tracking
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_trackers(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("use_stereo", use_stereo);
      parser->parse_config("use_klt", use_klt);
      parser->parse_config("use_aruco", use_aruco);
      parser->parse_config("downsize_aruco", downsize_aruco);
      parser->parse_config("downsample_cameras", downsample_cameras);
      parser->parse_config("multi_threading", use_multi_threading);
      parser->parse_config("num_pts", num_pts);
      parser->parse_config("fast_threshold", fast_threshold);
      parser->parse_config("grid_x", grid_x);
      parser->parse_config("grid_y", grid_y);
      parser->parse_config("min_px_dist", min_px_dist);
      std::string histogram_method_str = "HISTOGRAM";
      parser->parse_config("histogram_method", histogram_method_str);
      if (histogram_method_str == "NONE") {
        histogram_method = ov_core::TrackBase::NONE;
      } else if (histogram_method_str == "HISTOGRAM") {
        histogram_method = ov_core::TrackBase::HISTOGRAM;
      } else if (histogram_method_str == "CLAHE") {
        histogram_method = ov_core::TrackBase::CLAHE;
      } else {
        printf(RED "VioManager(): invalid feature histogram specified:\n" RESET);
        printf(RED "\t- NONE\n" RESET);
        printf(RED "\t- HISTOGRAM\n" RESET);
        printf(RED "\t- CLAHE\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      parser->parse_config("knn_ratio", knn_ratio);
    }
    PRINT_DEBUG("FEATURE TRACKING PARAMETERS:\n");
    PRINT_DEBUG("  - use_stereo: %d\n", use_stereo);
    PRINT_DEBUG("  - use_klt: %d\n", use_klt);
    PRINT_DEBUG("  - use_aruco: %d\n", use_aruco);
    PRINT_DEBUG("  - downsize aruco: %d\n", downsize_aruco);
    PRINT_DEBUG("  - downsize cameras: %d\n", downsample_cameras);
    PRINT_DEBUG("  - use multi-threading: %d\n", use_multi_threading);
    PRINT_DEBUG("  - num_pts: %d\n", num_pts);
    PRINT_DEBUG("  - fast threshold: %d\n", fast_threshold);
    PRINT_DEBUG("  - grid X by Y: %d by %d\n", grid_x, grid_y);
    PRINT_DEBUG("  - min px dist: %d\n", min_px_dist);
    PRINT_DEBUG("  - hist method: %d\n", (int)histogram_method);
    PRINT_DEBUG("  - knn ratio: %.3f\n", knn_ratio);
    featinit_options.print(parser);
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
  std::string sim_traj_path = "src/open_vins/ov_data/sim/udel_gore.txt";

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

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGEROPTIONS_H