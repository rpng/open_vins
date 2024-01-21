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

#include "InertialInitializer.h"

#include "dynamic/DynamicInitializer.h"
#include "static/StaticInitializer.h"

#include "feat/FeatureHelper.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

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

void InertialInitializer::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // Append it to our vector
  imu_data->emplace_back(message);

  // Sort our imu data (handles any out of order measurements)
  // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // Loop through and delete imu messages that are older than our requested time
  // std::cout << "INIT: imu_data.size() " << imu_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
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
  double oldest_time = newest_cam_time - params.init_window_time - 0.10;
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
  bool disparity_detected_moving_1to0 = false;
  bool disparity_detected_moving_2to1 = false;
  if (params.init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    // Only compute the disparity for the oldest half of the initialization period
    double newest_time_allowed = newest_cam_time - 0.5 * params.init_window_time;
    int num_features0 = 0;
    int num_features1 = 0;
    double avg_disp0, avg_disp1;
    double var_disp0, var_disp1;
    FeatureHelper::compute_disparity(_db, avg_disp0, var_disp0, num_features0, newest_time_allowed);
    FeatureHelper::compute_disparity(_db, avg_disp1, var_disp1, num_features1, newest_cam_time, newest_time_allowed);

    // Return if we can't compute the disparity
    int feat_thresh = 15;
    if (num_features0 < feat_thresh || num_features1 < feat_thresh) {
      PRINT_WARNING(YELLOW "[init]: not enough feats to compute disp: %d,%d < %d\n" RESET, num_features0, num_features1, feat_thresh);
      return false;
    }

    // Check if it passed our check!
    PRINT_INFO(YELLOW "[init]: disparity is %.3f,%.3f (%.2f thresh)\n" RESET, avg_disp0, avg_disp1, params.init_max_disparity);
    disparity_detected_moving_1to0 = (avg_disp0 > params.init_max_disparity);
    disparity_detected_moving_2to1 = (avg_disp1 > params.init_max_disparity);
  }

  // Use our static initializer!
  // CASE1: if our disparity says we were static in last window and have moved in the newest, we have a jerk
  // CASE2: if both disparities are below the threshold, then the platform has been stationary during both periods
  bool has_jerk = (!disparity_detected_moving_1to0 && disparity_detected_moving_2to1);
  bool is_still = (!disparity_detected_moving_1to0 && !disparity_detected_moving_2to1);
  if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && params.init_imu_thresh > 0.0) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else if (params.init_dyn_use && !is_still) {
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
    return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM);
  } else {
    std::string msg = (has_jerk) ? "" : "no accel jerk detected";
    msg += (has_jerk || is_still) ? "" : ", ";
    msg += (is_still) ? "" : "platform moving too much";
    PRINT_INFO(YELLOW "[init]: failed static init: %s\n" RESET, msg.c_str());
  }
  return false;
}
