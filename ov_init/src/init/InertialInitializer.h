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

#ifndef OV_INIT_INERTIALINITIALIZER_H
#define OV_INIT_INERTIALINITIALIZER_H

#include "init/InertialInitializerOptions.h"
#include "static/StaticInitializer.h"

#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace ov_init {

/**
 * @brief Initializer for visual-inertial system.
 *
 * This will try to do both dynamic and state initialization of the state.
 * The user can request to wait for a jump in our IMU readings (i.e. device is picked up) or to initialize as soon as possible.
 * For state initialization, the user needs to specify the calibration beforehand, otherwise dynamic is always used.
 *
 * The logic is as follows:
 * 1. Try to perform dynamic initialization of state elements.
 * 2. If this fails and we have calibration then we can try to do static initialization
 * 3. If the unit is stationary and we are waiting for a jerk, just return, otherwise initialize the state!
 *
 * The dynamic system is based on an implementation and extension of the work [Estimator initialization in vision-aided inertial navigation
 * with unknown camera-IMU calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization
 * problem by first creating a linear system for recovering the camera to IMU rotation, then for velocity, gravity, and feature positions,
 * and finally a full optimization to allow for covariance recovery.
 */
class InertialInitializer {

public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   */
  explicit InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   */
  void feed_imu(const ov_core::ImuData &message) {

    // Append it to our vector
    imu_data->push_back(message);

    // Delete all measurements older than three of our initialization windows
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end() && it0->timestamp < message.timestamp - 3 * params.init_window_time) {
      it0 = imu_data->erase(it0);
    }
  }

  /**
   * @brief Try to get the initialized system
   *
   * @param[out] timestamp Timestamp we have initialized the state at
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param[out] t_imu Our imu type (need to have correct ids)
   * @param wait_for_jerk If true we will wait for a "jerk"
   * @return True if we have successfully initialized our system
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

protected:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /// Static initialization helper class
  std::shared_ptr<StaticInitializer> init_static;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
