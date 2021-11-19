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
      return std::min_element(sensor_ids.begin(), sensor_ids.end()) < std::min_element(other.sensor_ids.begin(), other.sensor_ids.end());
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

  /// imu intrinsics
  Eigen::Matrix<double,6,1> imu_x_dw; // the columnwise elements for Dw
  Eigen::Matrix<double,6,1> imu_x_da; // the columnwise elements for Da
  Eigen::Matrix<double,9,1> imu_x_tg; // the ccolumnwise elements for Tg
  Eigen::Matrix<double,4,1> imu_quat_AcctoI; // the JPL quat for R_AcctoI
  Eigen::Matrix<double,4,1> imu_quat_GyrotoI; // the JPL quat for R_GyrotoI

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope white noise covariance
  double sigma_w_2 = pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Gyroscope random walk covariance
  double sigma_wb_2 = pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer white noise covariance
  double sigma_a_2 = pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Accelerometer random walk covariance
  double sigma_ab_2 = pow(3.0000e-03, 2);

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
    PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
    PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
    PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
  }

  /// get the IMU Dw
  Eigen::Matrix3d Dw(){
    Eigen::Matrix3d Dw = Eigen::Matrix3d::Identity();
    if(imu_model == 0){
      Dw <<
          imu_x_dw(0), 0,                 0,
          imu_x_dw(1), imu_x_dw(3), 0,
          imu_x_dw(2), imu_x_dw(4), imu_x_dw(5);
    }else{
      Dw <<
          imu_x_dw(0), imu_x_dw(1), imu_x_dw(3),
          0,                 imu_x_dw(2), imu_x_dw(4),
          0,                  0,                imu_x_dw(5);
    }
    return Dw;
  }

  /// get the IMU Da
  Eigen::Matrix3d Da(){
    Eigen::Matrix3d Da = Eigen::Matrix3d::Identity();
    if(imu_model == 0){
      Da <<
         imu_x_da(0), 0,                 0,
          imu_x_da(1), imu_x_da(3), 0,
          imu_x_da(2), imu_x_da(4), imu_x_da(5);
    }else{
      Da <<
         imu_x_da(0), imu_x_da(1), imu_x_da(3),
          0,                 imu_x_da(2), imu_x_da(4),
          0,                  0,                imu_x_da(5);
    }
    return Da;
  }

  /// get the IMU Tg
  Eigen::Matrix3d Tg(){
    Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
    Tg << imu_x_tg(0), imu_x_tg(3), imu_x_tg(6),
        imu_x_tg(1), imu_x_tg(4), imu_x_tg(7),
        imu_x_tg(2), imu_x_tg(5), imu_x_tg(8);
    return Tg;
  }

  /// get the R_AcctoI
  Eigen::Matrix3d R_AcctoI(){
    return ov_core::quat_2_Rot(imu_quat_AcctoI);
  }

  /// get the R_GyrotoI
  Eigen::Matrix3d R_GyrotoI(){
    return ov_core::quat_2_Rot(imu_quat_GyrotoI);
  }




};





} // namespace ov_core

#endif // OV_CORE_SENSOR_DATA_H