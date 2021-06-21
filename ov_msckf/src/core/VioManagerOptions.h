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
#include <string>
#include <vector>

#include "feat/FeatureInitializerOptions.h"
#include "state/Propagator.h"
#include "state/StateOptions.h"
#include "update/UpdaterOptions.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"

using namespace std;
using namespace ov_core;

namespace ov_msckf {

/**
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and simulation.
 * If you are going to add a parameter here you will need to add it to the parsers.
 * You will also need to add it to the print statement at the bottom of each.
 */
struct VioManagerOptions {

  // ESTIMATOR ===============================

  /// Core state options (e.g. number of cameras, use fej, stereo, what calibration to enable etc)
  StateOptions state_options;

  /// Delay, in seconds, that we should wait from init before we start estimating SLAM features
  double dt_slam_delay = 2.0;

  /// Amount of time we will initialize over (seconds)
  double init_window_time = 1.0;

  ///  Variance threshold on our acceleration to be classified as moving
  double init_imu_thresh = 1.0;

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
   * @brief This function will print out all estimator settings loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_estimator() {
    printf("ESTIMATOR PARAMETERS:\n");
    state_options.print();
    printf("\t- dt_slam_delay: %.1f\n", dt_slam_delay);
    printf("\t- init_window_time: %.2f\n", init_window_time);
    printf("\t- init_imu_thresh: %.2f\n", init_imu_thresh);
    printf("\t- zero_velocity_update: %d\n", try_zupt);
    printf("\t- zupt_max_velocity: %.2f\n", zupt_max_velocity);
    printf("\t- zupt_noise_multiplier: %.2f\n", zupt_noise_multiplier);
    printf("\t- zupt_max_disparity: %.4f\n", zupt_max_disparity);
    printf("\t- zupt_only_at_beginning?: %d\n", zupt_only_at_beginning);
    printf("\t- record timing?: %d\n", (int)record_timing_information);
    printf("\t- record timing filepath: %s\n", record_timing_filepath.c_str());
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
   * @brief This function will print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_noise() {
    printf("NOISE PARAMETERS:\n");
    imu_noises.print();
    printf("\tUpdater MSCKF Feats:\n");
    msckf_options.print();
    printf("\tUpdater SLAM Feats:\n");
    slam_options.print();
    printf("\tUpdater ARUCO Tags:\n");
    aruco_options.print();
    printf("\tUpdater ZUPT:\n");
    zupt_options.print();
  }

  // STATE DEFAULTS ==========================

  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// Time offset between camera and IMU.
  double calib_camimu_dt = 0.0;

  /// Map between camid and camera model (true=fisheye, false=radtan)
  std::map<size_t, bool> camera_fisheye;

  /// Map between camid and intrinsics. Values depends on the model but each should be a 4x1 vector normally.
  std::map<size_t, Eigen::VectorXd> camera_intrinsics;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, Eigen::VectorXd> camera_extrinsics;

  /// Map between camid and the dimensions of incoming images (width/cols, height/rows). This is normally only used during simulation.
  std::map<size_t, std::pair<int, int>> camera_wh;

  /**
   * @brief This function will print out all simulated parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_state() {
    printf("STATE PARAMETERS:\n");
    printf("\t- gravity_mag: %.4f\n", gravity_mag);
    printf("\t- gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
    printf("\t- calib_camimu_dt: %.4f\n", calib_camimu_dt);
    assert(state_options.num_cameras == (int)camera_fisheye.size());
    for (int n = 0; n < state_options.num_cameras; n++) {
      std::cout << "cam_" << n << "_fisheye:" << camera_fisheye.at(n) << std::endl;
      std::cout << "cam_" << n << "_wh:" << endl << camera_wh.at(n).first << " x " << camera_wh.at(n).second << std::endl;
      std::cout << "cam_" << n << "_intrinsic(0:3):" << endl << camera_intrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
      std::cout << "cam_" << n << "_intrinsic(4:7):" << endl << camera_intrinsics.at(n).block(4, 0, 4, 1).transpose() << std::endl;
      std::cout << "cam_" << n << "_extrinsic(0:3):" << endl << camera_extrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
      std::cout << "cam_" << n << "_extrinsic(4:6):" << endl << camera_extrinsics.at(n).block(4, 0, 3, 1).transpose() << std::endl;
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = quat_2_Rot(camera_extrinsics.at(n).block(0, 0, 4, 1)).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * camera_extrinsics.at(n).block(4, 0, 3, 1);
      std::cout << "T_C" << n << "toI:" << endl << T_CtoI << std::endl << std::endl;
    }
  }

  // TRACKERS ===============================

  /// If we should process two cameras are being stereo or binocular. If binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// Defined set of stereo camera id pairs which we will process together and try to generate stereo constraints between.
  std::vector<std::pair<int, int>> stereo_pairs;

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

  /// KNN ration between top two descriptor matcher which is required to be a good match
  double knn_ratio = 0.85;

  /// Parameters used by our feature initialize / triangulator
  FeatureInitializerOptions featinit_options;

  /**
   * @brief This function will print out all parameters releated to our visual trackers.
   */
  void print_trackers() {
    printf("FEATURE TRACKING PARAMETERS:\n");
    printf("\t- num_pts: %d\n", num_pts);
    printf("\t- use_stereo: %d\n", use_stereo);
    printf("\t- stereo pairs: ");
    for (const auto &pair : stereo_pairs) {
      printf("[%d, %d] ", pair.first, pair.second);
    }
    printf("\n");
    printf("\t- downsize aruco: %d\n", downsize_aruco);
    printf("\t- downsize cameras: %d\n", downsample_cameras);
    printf("\t- use multi-threading: %d\n", use_multi_threading);
    featinit_options.print();
  }

  // SIMULATOR ===============================

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  string sim_traj_path = "../ov_data/sim/udel_gore.txt";

  /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init from groundtruth in
  /// simulation.
  double sim_distance_threshold = 1.0;

  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Seed for initial states (i.e. random feature 3d positions in the generated map)
  int sim_seed_state_init = 0;

  /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are enabled.
  int sim_seed_preturb = 0;

  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements,
  /// but diffferent noise values.
  int sim_seed_measurements = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /**
   * @brief This function will print out all simulated parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   */
  void print_simulation() {
    printf("SIMULATION PARAMETERS:\n");
    printf(BOLDRED "\t- state init seed: %d \n" RESET, sim_seed_state_init);
    printf(BOLDRED "\t- perturb seed: %d \n" RESET, sim_seed_preturb);
    printf(BOLDRED "\t- measurement seed: %d \n" RESET, sim_seed_measurements);
    printf("\t- traj path: %s\n", sim_traj_path.c_str());
    printf("\t- dist thresh: %.2f\n", sim_distance_threshold);
    printf("\t- cam feq: %.2f\n", sim_freq_cam);
    printf("\t- imu feq: %.2f\n", sim_freq_imu);
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGEROPTIONS_H