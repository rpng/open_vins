/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
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

#include "InertialInitializer.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db)
    : params(params_), _db(db) {

  // Vector of our IMU data
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // Create initializers
  init_static = std::make_shared<StaticInitializer>(params, _db, imu_data);
  init_dynamic = std::make_shared<DynamicInitializer>(params, _db, imu_data);
}

bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                     std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // Get the newest and oldest timestamps we will try to initialize between!
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  double oldest_time = newest_cam_time - params.init_window_time - 0.01;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }

  // Remove all measurements that are older then our initialization window
  // Then we will try to use all features that are in the feature database!
  _db->cleanup_measurements(oldest_time);
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // Compute the disparity of the system at the current timestep
  // If disparity is zero or negative we will always use the static initializer
  bool disparity_detected_moving = false;
  if (params.init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    int num_features = 0;
    double average_disparity = 0.0;
    double variance_disparity = 0.0;
    FeatureHelper::compute_disparity(_db, average_disparity, variance_disparity, num_features);

    // Return if we can't compute the disparity
    if (num_features < 10) {
      PRINT_DEBUG(YELLOW "[init]: not enough features to compute disparity %d < 10\n" RESET, num_features);
      return false;
    }

    // Check if it passed our check!
    PRINT_DEBUG(YELLOW "[init]: disparity of the platform is %.4f (%.4f threshold)\n" RESET, average_disparity, params.init_max_disparity);
    disparity_detected_moving = (average_disparity > params.init_max_disparity);
  }

  // Use our static initializer!
  if (!disparity_detected_moving && params.init_imu_thresh > 0.0) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else {
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
    std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> _calib_IMUtoCAM;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> _cam_intrinsics;
    return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM, _calib_IMUtoCAM, _cam_intrinsics);
  }
}
