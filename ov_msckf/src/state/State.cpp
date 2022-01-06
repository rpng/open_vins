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

using namespace ov_core;
using namespace ov_type;
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

  // Append the imu intrinsics to the state and covariance
  // NOTE: these need to be right "next" to the IMU state in the covariance
  // NOTE: since if calibrating these will evolve / be correlated during propagation
  _imu_x_dw = std::make_shared<Vec>(6);
  _imu_x_da = std::make_shared<Vec>(6);
  if (options.imu_model == 0) {
    // lower triangular of the matrix (column-wise)
    Eigen::Matrix<double, 6, 1> _imu_default = Eigen::Matrix<double, 6, 1>::Zero();
    _imu_default << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0;
    _imu_x_dw->set_value(_imu_default);
    _imu_x_dw->set_fej(_imu_default);
    _imu_x_da->set_value(_imu_default);
    _imu_x_da->set_fej(_imu_default);
  } else {
    // upper triangular of the matrix (column-wise)
    Eigen::Matrix<double, 6, 1> _imu_default = Eigen::Matrix<double, 6, 1>::Zero();
    _imu_default << 1.0, 0.0, 1.0, 0.0, 0.0, 1.0;
    _imu_x_dw->set_fej(_imu_default);
    _imu_x_dw->set_fej(_imu_default);
    _imu_x_da->set_fej(_imu_default);
    _imu_x_da->set_fej(_imu_default);
  }
  _imu_x_tg = std::make_shared<Vec>(9);
  _imu_quat_gyrotoI = std::make_shared<JPLQuat>();
  _imu_quat_acctoI = std::make_shared<JPLQuat>();
  if (options.do_calib_imu_intrinsics) {

    // Gyroscope dw
    _imu_x_dw->set_local_id(current_id);
    _variables.push_back(_imu_x_dw);
    current_id += _imu_x_dw->size();

    // Accelerometer da
    _imu_x_da->set_local_id(current_id);
    _variables.push_back(_imu_x_da);
    current_id += _imu_x_da->size();

    // Gyroscope gravity sensitivity
    if (options.do_calib_imu_g_sensitivity) {
      _imu_x_tg->set_local_id(current_id);
      _variables.push_back(_imu_x_tg);
      current_id += _imu_x_tg->size();
    }

    // If kalibr model, R_GyrotoI is calibrated
    // If rpng model, R_AcctoI is calibrated
    if (options.imu_model == 0) {
      _imu_quat_gyrotoI->set_local_id(current_id);
      _variables.push_back(_imu_quat_gyrotoI);
      current_id += _imu_quat_gyrotoI->size();
    } else {
      _imu_quat_acctoI->set_local_id(current_id);
      _variables.push_back(_imu_quat_acctoI);
      current_id += _imu_quat_acctoI->size();
    }
  }

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
  if (_options.do_calib_imu_intrinsics) {
    _Cov.block<6, 6>(_imu_x_dw->id(), _imu_x_dw->id()) = std::pow(0.005, 2) * Eigen::Matrix<double, 6, 6>::Identity();
    _Cov.block<6, 6>(_imu_x_da->id(), _imu_x_da->id()) = std::pow(0.008, 2) * Eigen::Matrix<double, 6, 6>::Identity();
    if (_options.do_calib_imu_g_sensitivity) {
      _Cov.block<9, 9>(_imu_x_tg->id(), _imu_x_tg->id()) = std::pow(0.005, 2) * Eigen::Matrix<double, 9, 9>::Identity();
    }
    if (_options.imu_model == 0) {
      // if kalibr model, R_GyrotoI is calibrated
      _Cov.block<3, 3>(_imu_quat_gyrotoI->id(), _imu_quat_gyrotoI->id()) = std::pow(0.005, 2) * Eigen::Matrix<double, 3, 3>::Identity();
    } else {
      // if rpng model, R_AcctoI is calibrated
      _Cov.block<3, 3>(_imu_quat_acctoI->id(), _imu_quat_acctoI->id()) = std::pow(0.005, 2) * Eigen::Matrix<double, 3, 3>::Identity();
    }
  }

  if (_options.do_calib_camera_timeoffset) {
    _Cov(_calib_dt_CAMtoIMU->id(), _calib_dt_CAMtoIMU->id()) = std::pow(0.01, 2);
  }
  if (_options.do_calib_camera_pose) {
    for (int i = 0; i < _options.num_cameras; i++) {
      _Cov.block(_calib_IMUtoCAM.at(i)->id(), _calib_IMUtoCAM.at(i)->id(), 3, 3) = std::pow(0.001, 2) * Eigen::MatrixXd::Identity(3, 3);
      _Cov.block(_calib_IMUtoCAM.at(i)->id() + 3, _calib_IMUtoCAM.at(i)->id() + 3, 3, 3) =
          std::pow(0.015, 2) * Eigen::MatrixXd::Identity(3, 3);
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

Eigen::Matrix3d State::Dw() {
  Eigen::Matrix3d Dw = Eigen::Matrix3d::Identity();
  if (_options.imu_model == 0) {
    // if kalibr model, lower triangular of the matrix is used
    Dw << _imu_x_dw->value()(0), 0, 0, _imu_x_dw->value()(1), _imu_x_dw->value()(3), 0, _imu_x_dw->value()(2), _imu_x_dw->value()(4),
        _imu_x_dw->value()(5);
  } else {
    // if rpng model, upper triangular of the matrix is used
    Dw << _imu_x_dw->value()(0), _imu_x_dw->value()(1), _imu_x_dw->value()(3), 0, _imu_x_dw->value()(2), _imu_x_dw->value()(4), 0, 0,
        _imu_x_dw->value()(5);
  }
  return Dw;
}

Eigen::Matrix3d State::Da() {
  Eigen::Matrix3d Da = Eigen::Matrix3d::Identity();
  if (_options.imu_model == 0) {
    // if kalibr model, lower triangular of the matrix is used
    Da << _imu_x_da->value()(0), 0, 0, _imu_x_da->value()(1), _imu_x_da->value()(3), 0, _imu_x_da->value()(2), _imu_x_da->value()(4),
        _imu_x_da->value()(5);
  } else {
    // if rpng model, upper triangular of the matrix is used
    Da << _imu_x_da->value()(0), _imu_x_da->value()(1), _imu_x_da->value()(3), 0, _imu_x_da->value()(2), _imu_x_da->value()(4), 0, 0,
        _imu_x_da->value()(5);
  }
  return Da;
}

Eigen::Matrix3d State::Tg() {
  Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
  Tg << _imu_x_tg->value()(0), _imu_x_tg->value()(3), _imu_x_tg->value()(6), _imu_x_tg->value()(1), _imu_x_tg->value()(4),
      _imu_x_tg->value()(7), _imu_x_tg->value()(2), _imu_x_tg->value()(5), _imu_x_tg->value()(8);

  return Tg;
}

Eigen::Matrix3d State::R_AcctoI() { return _imu_quat_acctoI->Rot(); }

Eigen::Matrix3d State::R_GyrotoI() { return _imu_quat_gyrotoI->Rot(); }
