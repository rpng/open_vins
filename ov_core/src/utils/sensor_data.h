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

#ifndef OV_CORE_SENSOR_DATA_H
#define OV_CORE_SENSOR_DATA_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

#include "utils/quat_ops.h"

namespace ov_core {

/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData {

  /// Timestamp of the reading
  double timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for a collection of camera measurements.
 *
 * For each image we have a camera id and timestamp that it occured at.
 * If there are multiple cameras we will treat it as pair-wise stereo tracking.
 */
struct CameraData {

  /// Timestamp of the reading
  double timestamp;

  /// Camera ids for each of the images collected
  std::vector<int> sensor_ids;

  /// Raw image we have collected for each camera
  std::vector<cv::Mat> images;

  /// Tracking masks for each camera we have
  std::vector<cv::Mat> masks;

  /// Sort function to allow for using of STL containers
  bool operator<(const CameraData &other) const {
    if (timestamp == other.timestamp) {
      int id = *std::min_element(sensor_ids.begin(), sensor_ids.end());
      int id_other = *std::min_element(other.sensor_ids.begin(), other.sensor_ids.end());
      return id < id_other;
    } else {
      return timestamp < other.timestamp;
    }
  }
};

/**
 * @brief Struct of our imu noise parameters
 */
struct ImuConfig {

  /// imu model, 0: Kalibr model and 1: RPNG model
  int imu_model = 0;

  /// the columnwise elements for Dw
  Eigen::Matrix<double, 6, 1> vec_dw;

  /// the columnwise elements for Da
  Eigen::Matrix<double, 6, 1> vec_da;

  /// the ccolumnwise elements for Tg
  Eigen::Matrix<double, 9, 1> vec_tg;

  /// the JPL quat for R_AcctoI
  Eigen::Matrix<double, 4, 1> q_ACCtoIMU;

  /// the JPL quat for R_GyrotoI
  Eigen::Matrix<double, 4, 1> q_GYROtoIMU;

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
    PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
    PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
    PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
  }

  /// get the IMU Dw
  Eigen::Matrix3d Dw() {
    Eigen::Matrix3d Dw = Eigen::Matrix3d::Identity();
    if (imu_model == 0) {
      // Kalibr model with lower triangular of the matrix
      Dw << vec_dw(0), 0, 0, vec_dw(1), vec_dw(3), 0, vec_dw(2), vec_dw(4), vec_dw(5);
    } else {
      // rpng model with upper triangular of the matrix
      Dw << vec_dw(0), vec_dw(1), vec_dw(3), 0, vec_dw(2), vec_dw(4), 0, 0, vec_dw(5);
    }
    return Dw;
  }

  /// get the IMU Da
  Eigen::Matrix3d Da() {
    Eigen::Matrix3d Da = Eigen::Matrix3d::Identity();
    if (imu_model == 0) {
      // kalibr model with lower triangular of the matrix
      Da << vec_da(0), 0, 0, vec_da(1), vec_da(3), 0, vec_da(2), vec_da(4), vec_da(5);
    } else {
      // rpng model with upper triangular of the matrix
      Da << vec_da(0), vec_da(1), vec_da(3), 0, vec_da(2), vec_da(4), 0, 0, vec_da(5);
    }
    return Da;
  }

  /// get the IMU Tw
  Eigen::Matrix3d Tw() { return Dw().inverse(); }

  /// get the IMU Ta
  Eigen::Matrix3d Ta() { return Da().inverse(); }

  /// get the IMU Tg
  Eigen::Matrix3d Tg() {
    Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
    Tg << vec_tg(0), vec_tg(3), vec_tg(6), vec_tg(1), vec_tg(4), vec_tg(7), vec_tg(2), vec_tg(5), vec_tg(8);
    return Tg;
  }

  /// get the R_AcctoI
  Eigen::Matrix3d R_AcctoI() { return ov_core::quat_2_Rot(q_ACCtoIMU); }
  /// get the R_ItoAcc
  Eigen::Matrix3d R_ItoAcc() { return R_AcctoI().transpose(); }

  /// get the R_GyrotoI
  Eigen::Matrix3d R_GyrotoI() { return ov_core::quat_2_Rot(q_GYROtoIMU); }

  /// get the R_ItoGyro
  Eigen::Matrix3d R_ItoGyro() { return R_GyrotoI().transpose(); }
};

} // namespace ov_core

#endif // OV_CORE_SENSOR_DATA_H