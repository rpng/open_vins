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

#ifndef OV_MSCKF_STATE_OPTIONS_H
#define OV_MSCKF_STATE_OPTIONS_H

#include "types/LandmarkRepresentation.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace ov_msckf {

/**
 * @brief Struct which stores all our filter options
 */
struct StateOptions {

  /// Bool to determine whether or not to do first estimate Jacobians
  bool do_fej = true;

  /// Numerical integration methods
  enum IntegrationMethod { DISCRETE, RK4, ANALYTICAL };

  /// What type of numerical integration is used during propagation
  IntegrationMethod integration_method = IntegrationMethod::RK4;

  /// Bool to determine whether or not to calibrate imu-to-camera pose
  bool do_calib_camera_pose = false;

  /// Bool to determine whether or not to calibrate camera intrinsics
  bool do_calib_camera_intrinsics = false;

  /// Bool to determine whether or not to calibrate camera to IMU time offset
  bool do_calib_camera_timeoffset = false;

  /// Bool to determine whether or not to calibrate the IMU intrinsics
  bool do_calib_imu_intrinsics = false;

  /// Bool to determine whether or not to calibrate the Gravity sensitivity
  bool do_calib_imu_g_sensitivity = false;

  /// IMU intrinsic models
  enum ImuModel { KALIBR, RPNG };

  /// What model our IMU intrinsics are
  ImuModel imu_model = ImuModel::KALIBR;

  /// Max clone size of sliding window
  int max_clone_size = 11;

  /// Max number of estimated SLAM features
  int max_slam_features = 25;

  /// Max number of SLAM features we allow to be included in a single EKF update.
  int max_slam_in_update = 1000;

  /// Max number of MSCKF features we will use at a given image timestep.
  int max_msckf_in_update = 1000;

  /// Max number of estimated ARUCO features
  int max_aruco_features = 1024;

  /// Number of distinct cameras that we will observe features in
  int num_cameras = 1;

  /// What representation our features are in (msckf features)
  ov_type::LandmarkRepresentation::Representation feat_rep_msckf = ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;

  /// What representation our features are in (slam features)
  ov_type::LandmarkRepresentation::Representation feat_rep_slam = ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;

  /// What representation our features are in (aruco tag features)
  ov_type::LandmarkRepresentation::Representation feat_rep_aruco = ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;

  /// Nice print function of what parameters we have loaded
  void print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("use_fej", do_fej);

      // Integration method
      std::string integration_str = "rk4";
      parser->parse_config("integration", integration_str);
      if (integration_str == "discrete") {
        integration_method = IntegrationMethod::DISCRETE;
      } else if (integration_str == "rk4") {
        integration_method = IntegrationMethod::RK4;
      } else if (integration_str == "analytical") {
        integration_method = IntegrationMethod::ANALYTICAL;
      } else {
        PRINT_ERROR(RED "invalid imu integration model: %s\n" RESET, integration_str.c_str());
        PRINT_ERROR(RED "please select a valid model: discrete, rk4, analytical\n" RESET);
        std::exit(EXIT_FAILURE);
      }

      // Calibration booleans
      parser->parse_config("calib_cam_extrinsics", do_calib_camera_pose);
      parser->parse_config("calib_cam_intrinsics", do_calib_camera_intrinsics);
      parser->parse_config("calib_cam_timeoffset", do_calib_camera_timeoffset);
      parser->parse_config("calib_imu_intrinsics", do_calib_imu_intrinsics);
      parser->parse_config("calib_imu_g_sensitivity", do_calib_imu_g_sensitivity);

      // State parameters
      parser->parse_config("max_clones", max_clone_size);
      parser->parse_config("max_slam", max_slam_features);
      parser->parse_config("max_slam_in_update", max_slam_in_update);
      parser->parse_config("max_msckf_in_update", max_msckf_in_update);
      parser->parse_config("num_aruco", max_aruco_features);
      parser->parse_config("max_cameras", num_cameras);

      // Feature representations
      std::string rep1 = ov_type::LandmarkRepresentation::as_string(feat_rep_msckf);
      parser->parse_config("feat_rep_msckf", rep1);
      feat_rep_msckf = ov_type::LandmarkRepresentation::from_string(rep1);
      std::string rep2 = ov_type::LandmarkRepresentation::as_string(feat_rep_slam);
      parser->parse_config("feat_rep_slam", rep2);
      feat_rep_slam = ov_type::LandmarkRepresentation::from_string(rep2);
      std::string rep3 = ov_type::LandmarkRepresentation::as_string(feat_rep_aruco);
      parser->parse_config("feat_rep_aruco", rep3);
      feat_rep_aruco = ov_type::LandmarkRepresentation::from_string(rep3);

      // IMU model
      std::string imu_model_str = "kalibr";
      parser->parse_external("relative_config_imu", "imu0", "model", imu_model_str);
      if (imu_model_str == "kalibr" || imu_model_str == "calibrated") {
        imu_model = ImuModel::KALIBR;
      } else if (imu_model_str == "rpng") {
        imu_model = ImuModel::RPNG;
      } else {
        PRINT_ERROR(RED "invalid imu model: %s\n" RESET, imu_model_str.c_str());
        PRINT_ERROR(RED "please select a valid model: kalibr, rpng\\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      if (imu_model_str == "calibrated" && (do_calib_imu_intrinsics || do_calib_imu_g_sensitivity)) {
        PRINT_ERROR(RED "calibrated IMU model selected, but requested calibration!\n" RESET);
        PRINT_ERROR(RED "please select what model you have: kalibr, rpng\n" RESET);
        std::exit(EXIT_FAILURE);
      }
    }
    PRINT_DEBUG("  - use_fej: %d\n", do_fej);
    PRINT_DEBUG("  - integration: %d\n", integration_method);
    PRINT_DEBUG("  - calib_cam_extrinsics: %d\n", do_calib_camera_pose);
    PRINT_DEBUG("  - calib_cam_intrinsics: %d\n", do_calib_camera_intrinsics);
    PRINT_DEBUG("  - calib_cam_timeoffset: %d\n", do_calib_camera_timeoffset);
    PRINT_DEBUG("  - calib_imu_intrinsics: %d\n", do_calib_imu_intrinsics);
    PRINT_DEBUG("  - calib_imu_g_sensitivity: %d\n", do_calib_imu_g_sensitivity);
    PRINT_DEBUG("  - imu_model: %d\n", imu_model);
    PRINT_DEBUG("  - max_clones: %d\n", max_clone_size);
    PRINT_DEBUG("  - max_slam: %d\n", max_slam_features);
    PRINT_DEBUG("  - max_slam_in_update: %d\n", max_slam_in_update);
    PRINT_DEBUG("  - max_msckf_in_update: %d\n", max_msckf_in_update);
    PRINT_DEBUG("  - max_aruco: %d\n", max_aruco_features);
    PRINT_DEBUG("  - max_cameras: %d\n", num_cameras);
    PRINT_DEBUG("  - feat_rep_msckf: %s\n", ov_type::LandmarkRepresentation::as_string(feat_rep_msckf).c_str());
    PRINT_DEBUG("  - feat_rep_slam: %s\n", ov_type::LandmarkRepresentation::as_string(feat_rep_slam).c_str());
    PRINT_DEBUG("  - feat_rep_aruco: %s\n", ov_type::LandmarkRepresentation::as_string(feat_rep_aruco).c_str());
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_OPTIONS_H