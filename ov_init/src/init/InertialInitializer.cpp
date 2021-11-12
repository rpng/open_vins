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

#include "InertialInitializer.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(InertialInitializerOptions &params_) : params(params_) {

  // Vector of our IMU data
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // Create static initializer
  init_static = std::make_shared<StaticInitializer>(params, imu_data);

  // TODO: create the feature tracker here
  // TODO: create dynamic initialize class object
}

bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                     std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // TODO: calculate the current disparity to see if we are stationary

  // TODO: if not stationary, first try to do our dynamic initialization, return on success

  // TODO: return if we need to initalize any of the calibration (only dynamic can do this)

  // TODO: return if we are wait_for_jerk and disparity says we are moving (handles constant accel case)

  // TODO: if we have calibration, check if we are still, and use our static initalizer

  // TEMP: use our static initializer!
  return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
}
