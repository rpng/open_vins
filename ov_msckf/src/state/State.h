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

#ifndef OV_MSCKF_STATE_H
#define OV_MSCKF_STATE_H

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "StateOptions.h"
#include "cam/CamBase.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"

namespace ov_msckf {

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
 * We additionally have more parameters for online estimation of calibration and SLAM features.
 * We also have the covariance of the system, which should be managed using the StateHelper class.
 */
class State {

public:
  /**
   * @brief Default Constructor (will initialize variables to defaults)
   * @param options_ Options structure containing filter options
   */
  State(StateOptions &options_);

  ~State() {}

  /**
   * @brief Will return the timestep that we will marginalize next.
   * As of right now, since we are using a sliding window, this is the oldest clone.
   * But if you wanted to do a keyframe system, you could selectively marginalize clones.
   * @return timestep of clone we will marginalize
   */
  double margtimestep() {
    std::lock_guard<std::mutex> lock(_mutex_state);
    double time = INFINITY;
    for (const auto &clone_imu : _clones_IMU) {
      if (clone_imu.first < time) {
        time = clone_imu.first;
      }
    }
    return time;
  }

  /**
   * @brief Calculates the current max size of the covariance
   * @return Size of the current covariance matrix
   */
  int max_covariance_size() { return (int)_Cov.rows(); }

  /**
   * @brief Gyroscope and accelerometer intrinsic matrix (scale imperfection and axis misalignment)
   *
   * If kalibr model, lower triangular of the matrix is used
   * If rpng model, upper triangular of the matrix is used
   *
   * @return 3x3 matrix of current imu gyroscope / accelerometer intrinsics
   */
  static Eigen::Matrix3d Dm(StateOptions::ImuModel imu_model, const Eigen::MatrixXd &vec) {
    assert(vec.rows() == 6);
    assert(vec.cols() == 1);
    Eigen::Matrix3d D_matrix = Eigen::Matrix3d::Identity();
    if (imu_model == StateOptions::ImuModel::KALIBR) {
      D_matrix << vec(0), 0, 0, vec(1), vec(3), 0, vec(2), vec(4), vec(5);
    } else {
      D_matrix << vec(0), vec(1), vec(3), 0, vec(2), vec(4), 0, 0, vec(5);
    }
    return D_matrix;
  }

  /**
   * @brief Gyroscope gravity sensitivity
   *
   * For both kalibr and rpng models, this a 3x3 that is column-wise filled.
   *
   * @return 3x3 matrix of current gravity sensitivity
   */
  static Eigen::Matrix3d Tg(const Eigen::MatrixXd &vec) {
    assert(vec.rows() == 9);
    assert(vec.cols() == 1);
    Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
    Tg << vec(0), vec(3), vec(6), vec(1), vec(4), vec(7), vec(2), vec(5), vec(8);
    return Tg;
  }

  /**
   * @brief Calculates the error state size for imu intrinsics.
   *
   * This is used to construct our state transition which depends on if we are estimating calibration.
   * 15 if doing intrinsics, another +9 if doing grav sensitivity
   *
   * @return size of error state
   */
  int imu_intrinsic_size() const {
    int sz = 0;
    if (_options.do_calib_imu_intrinsics) {
      sz += 15;
      if (_options.do_calib_imu_g_sensitivity) {
        sz += 9;
      }
    }
    return sz;
  }

  /// Mutex for locking access to the state
  std::mutex _mutex_state;

  /// Current timestamp (should be the last update time in camera clock frame!)
  double _timestamp = -1;

  /// Struct containing filter options
  StateOptions _options;

  /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
  std::shared_ptr<ov_type::IMU> _imu;

  /// Map between imaging times and clone poses (q_GtoIi, p_IiinG)
  std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;

  /// Our current set of SLAM features (3d positions)
  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;

  /// Time offset base IMU to camera (t_imu = t_cam + t_off)
  std::shared_ptr<ov_type::Vec> _calib_dt_CAMtoIMU;

  /// Calibration poses for each camera (R_ItoC, p_IinC)
  std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> _calib_IMUtoCAM;

  /// Camera intrinsics
  std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> _cam_intrinsics;

  /// Camera intrinsics camera objects
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> _cam_intrinsics_cameras;

  /// Gyroscope IMU intrinsics (scale imperfection and axis misalignment)
  std::shared_ptr<ov_type::Vec> _calib_imu_dw;

  /// Accelerometer IMU intrinsics (scale imperfection and axis misalignment)
  std::shared_ptr<ov_type::Vec> _calib_imu_da;

  /// Gyroscope gravity sensitivity
  std::shared_ptr<ov_type::Vec> _calib_imu_tg;

  /// Rotation from gyroscope frame to the "IMU" accelerometer frame (kalibr model)
  std::shared_ptr<ov_type::JPLQuat> _calib_imu_GYROtoIMU;

  /// Rotation from accelerometer to the "IMU" gyroscope frame frame (rpng model)
  std::shared_ptr<ov_type::JPLQuat> _calib_imu_ACCtoIMU;

private:
  // Define that the state helper is a friend class of this class
  // This will allow it to access the below functions which should normally not be called
  // This prevents a developer from thinking that the "insert clone" will actually correctly add it to the covariance
  friend class StateHelper;

  /// Covariance of all active variables
  Eigen::MatrixXd _Cov;

  /// Vector of variables
  std::vector<std::shared_ptr<ov_type::Type>> _variables;
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_H