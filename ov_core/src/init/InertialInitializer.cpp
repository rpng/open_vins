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

void InertialInitializer::feed_imu(const ImuData &message) {

  // Append it to our vector
  imu_data.push_back(message);

  // Delete all measurements older than three of our initialization windows
  auto it0 = imu_data.begin();
  while (it0 != imu_data.end() && it0->timestamp < message.timestamp - 3 * _window_length) {
    it0 = imu_data.erase(it0);
  }
}

bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double, 4, 1> &q_GtoI0, Eigen::Matrix<double, 3, 1> &b_w0,
                                              Eigen::Matrix<double, 3, 1> &v_I0inG, Eigen::Matrix<double, 3, 1> &b_a0,
                                              Eigen::Matrix<double, 3, 1> &p_I0inG, bool wait_for_jerk) {

  // Return if we don't have any measurements
  if (imu_data.size() < 2) {
    return false;
  }

  // Newest and oldest imu timestamp
  double newesttime = imu_data.at(imu_data.size() - 1).timestamp;
  double oldesttime = imu_data.at(0).timestamp;

  // Return if we don't have enough for two windows
  if(newesttime-oldesttime < 2 * _window_length) {
    // printf(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // First lets collect a window of IMU readings from the newest measurement to the oldest
  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : imu_data) {
    if (data.timestamp > newesttime - 1 * _window_length && data.timestamp <= newesttime - 0 * _window_length) {
      window_1to0.push_back(data);
    }
    if (data.timestamp > newesttime - 2 * _window_length && data.timestamp <= newesttime - 1 * _window_length) {
      window_2to1.push_back(data);
    }
  }

  // Return if both of these failed
  if (window_1to0.empty() || window_2to1.empty()) {
    // printf(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // Calculate the sample variance for the newest window from 1 to 0
  Eigen::Vector3d a_avg_1to0 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_1to0) {
    a_avg_1to0 += data.am;
  }
  a_avg_1to0 /= (int)window_1to0.size();
  double a_var_1to0 = 0;
  for (const ImuData &data : window_1to0) {
    a_var_1to0 += (data.am - a_avg_1to0).dot(data.am - a_avg_1to0);
  }
  a_var_1to0 = std::sqrt(a_var_1to0 / ((int)window_1to0.size() - 1));

  // Calculate the sample variance for the second newest window from 2 to 1
  Eigen::Vector3d a_avg_2to1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg_2to1 = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < window_2to1.size(); i++) {
    a_avg_2to1 += window_2to1.at(i).am;
    w_avg_2to1 += window_2to1.at(i).wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  //printf(BOLDGREEN "[INIT-IMU]: IMU excitation, %.4f,%.4f\n" RESET, a_var_1to0, a_var_2to1);

  // If it is below the threshold and we want to wait till we detect a jerk
  if (a_var_1to0 < _imu_excite_threshold && wait_for_jerk) {
    printf(YELLOW "[INIT-IMU]: no IMU excitation, below threshold %.4f < %.4f\n" RESET, a_var_1to0, _imu_excite_threshold);
    return false;
  }

  // We should also check that the old state was below the threshold!
  // This is the case when we have started up moving, and thus we need to wait for a period of stationary motion
  if (a_var_2to1 > _imu_excite_threshold && wait_for_jerk) {
    printf(YELLOW "[INIT-IMU]: to much IMU excitation, above threshold %.4f > %.4f\n" RESET, a_var_2to1, _imu_excite_threshold);
    return false;
  }

  // If it is above the threshold and we are not waiting for a jerk
  // Then we are not stationary (i.e. moving) so we should wait till we are
  if ((a_var_1to0 > _imu_excite_threshold || a_var_2to1 > _imu_excite_threshold) && !wait_for_jerk) {
    printf(YELLOW "[INIT-IMU]: to much IMU excitation, above threshold %.4f,%.4f > %.4f\n" RESET, a_var_1to0, a_var_2to1, _imu_excite_threshold);
    return false;
  }

  // Get z axis, which alines with -g (z_in_G=0,0,1)
  Eigen::Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Matrix<double, 3, 1> y_axis = skew_x(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix<double, 3, 3> Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  // Create our state variables
  Eigen::Matrix<double, 4, 1> q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  Eigen::Matrix<double, 3, 1> bg = w_avg_2to1;
  Eigen::Matrix<double, 3, 1> ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * _gravity;

  // Set our state variables
  time0 = window_2to1.at(window_2to1.size() - 1).timestamp;
  q_GtoI0 = q_GtoI;
  b_w0 = bg;
  v_I0inG = Eigen::Matrix<double, 3, 1>::Zero();
  b_a0 = ba;
  p_I0inG = Eigen::Matrix<double, 3, 1>::Zero();

  // Done!!!
  return true;
}
