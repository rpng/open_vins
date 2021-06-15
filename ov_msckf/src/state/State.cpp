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


#include "State.h"

#include <memory>

using namespace ov_core;
using namespace ov_msckf;

State::State(StateOptions &options) {

  // Save our options
  _options = options;

  // Append the imu to the state and covariance
  int current_id = 0;
  _imu = std::make_shared<IMU>();
  _imu->set_local_id(current_id);
  _variables.push_back(_imu);
  current_id += _imu->size();

  // Camera to IMU time offset
  _calib_dt_CAMtoIMU = std::make_shared<Vec>(1);
  if (_options.do_calib_camera_timeoffset) {
    _calib_dt_CAMtoIMU->set_local_id(current_id);
    _variables.push_back(_calib_dt_CAMtoIMU);
    current_id += _calib_dt_CAMtoIMU->size();
  }

  // Loop through each camera and create extrinsic and intrinsics
  for (int i = 0; i < _options.num_cameras; i++) {

    // Allocate extrinsic transform
    auto pose = std::make_shared<PoseJPL>();

    // Allocate intrinsics for this camera
    auto intrin = std::make_shared<Vec>(8);

    // Add these to the corresponding maps
    _calib_IMUtoCAM.insert({i, pose});
    _cam_intrinsics.insert({i, intrin});
    _cam_intrinsics_model.insert({i, false});

    // If calibrating camera-imu pose, add to variables
    if (_options.do_calib_camera_pose) {
      pose->set_local_id(current_id);
      _variables.push_back(pose);
      current_id += pose->size();
    }

    // If calibrating camera intrinsics, add to variables
    if (_options.do_calib_camera_intrinsics) {
      intrin->set_local_id(current_id);
      _variables.push_back(intrin);
      current_id += intrin->size();
    }
  }

  // Finally initialize our covariance to small value
  _Cov = 1e-3 * Eigen::MatrixXd::Identity(current_id, current_id);

  // Finally, set some of our priors for our calibration parameters
  if (_options.do_calib_camera_timeoffset) {
    _Cov(_calib_dt_CAMtoIMU->id(), _calib_dt_CAMtoIMU->id()) = std::pow(0.01, 2);
  }
  if (_options.do_calib_camera_pose) {
    for (int i = 0; i < _options.num_cameras; i++) {
      _Cov.block(_calib_IMUtoCAM.at(i)->id(), _calib_IMUtoCAM.at(i)->id(), 3, 3) = std::pow(0.001, 2) * Eigen::MatrixXd::Identity(3, 3);
      _Cov.block(_calib_IMUtoCAM.at(i)->id() + 3, _calib_IMUtoCAM.at(i)->id() + 3, 3, 3) =
          std::pow(0.01, 2) * Eigen::MatrixXd::Identity(3, 3);
    }
  }
  if (_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < _options.num_cameras; i++) {
      _Cov.block(_cam_intrinsics.at(i)->id(), _cam_intrinsics.at(i)->id(), 4, 4) = std::pow(1.0, 2) * Eigen::MatrixXd::Identity(4, 4);
      _Cov.block(_cam_intrinsics.at(i)->id() + 4, _cam_intrinsics.at(i)->id() + 4, 4, 4) =
          std::pow(0.005, 2) * Eigen::MatrixXd::Identity(4, 4);
    }
  }
}
