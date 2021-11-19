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

#include "StaticInitializer.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

bool StaticInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<Type>> &order,
                                   std::shared_ptr<IMU> t_imu, bool wait_for_jerk) {

  // Return if we don't have any measurements
  if (imu_data->size() < 2) {
    return false;
  }

  // Newest and oldest imu timestamp
  double newesttime = imu_data->at(imu_data->size() - 1).timestamp;
  double oldesttime = imu_data->at(0).timestamp;

  // Return if we don't have enough for two windows
  if (newesttime - oldesttime < 2 * params.init_window_time) {
    PRINT_DEBUG(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // First lets collect a window of IMU readings from the newest measurement to the oldest
  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : *imu_data) {
    if (data.timestamp > newesttime - 1 * params.init_window_time && data.timestamp <= newesttime - 0 * params.init_window_time) {
      window_1to0.push_back(data);
    }
    if (data.timestamp > newesttime - 2 * params.init_window_time && data.timestamp <= newesttime - 1 * params.init_window_time) {
      window_2to1.push_back(data);
    }
  }

  // Return if both of these failed
  if (window_1to0.size() < 2 || window_2to1.size() < 2) {
    PRINT_DEBUG(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // First we can check the disparity of the two
  if (params.init_max_disparity > 0) {

    // Get time in the camera
    double time0_imu_in_cam = window_2to1.at(0).timestamp;
    double time1_imu_in_cam = window_2to1.at(window_2to1.size() - 1).timestamp;

    // Find the closest camera timestamps
    std::map<double, bool> camera_times;
    for (auto const &row : _db->get_internal_data()) {
      for (auto const &times : row.second->timestamps) {
        for (auto const &time : times.second) {
          camera_times[time] = true;
        }
      }
    }
    auto it0 = camera_times.lower_bound(time0_imu_in_cam);
    auto it1 = camera_times.lower_bound(time1_imu_in_cam);
    if (it0 == camera_times.end() || it1 == camera_times.end()) {
      PRINT_DEBUG(YELLOW "[INIT-IMU]: unable to find camera timestamps to compute disparity (have you tracked features?)\n" RESET);
      return false;
    }

    // Get the disparity statistics from this image to the previous
    double time0_cam = it0->first;
    double time1_cam = it1->first;
    int num_features = 0;
    double average_disparity = 0.0;
    double variance_disparity = 0.0;
    FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, average_disparity, variance_disparity, num_features);

    // Remove old timestamps
    _db->cleanup_measurements(time0_imu_in_cam);

    // Return if we can't compute the disparity
    if (num_features < 10) {
      PRINT_DEBUG(YELLOW "[INIT-IMU]: not enough features to compute disparity %d < 10\n" RESET, num_features);
      return false;
    }

    // Check if it passed our check!
    if (average_disparity > params.init_max_disparity) {
      PRINT_DEBUG(YELLOW "[INIT-IMU]: disparity says the platform is moving %.4f < %.4f\n" RESET, average_disparity,
                  params.init_max_disparity);
      return false;
    }
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
  for (const ImuData &data : window_2to1) {
    a_avg_2to1 += data.am;
    w_avg_2to1 += data.wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  // PRINT_DEBUG(BOLDGREEN "[INIT-IMU]: IMU excitation, %.4f,%.4f\n" RESET, a_var_1to0, a_var_2to1);

  // If it is below the threshold and we want to wait till we detect a jerk
  if (a_var_1to0 < params.init_imu_thresh && wait_for_jerk) {
    PRINT_DEBUG(YELLOW "[INIT-IMU]: no IMU excitation, below threshold %.4f < %.4f\n" RESET, a_var_1to0, params.init_imu_thresh);
    return false;
  }

  // We should also check that the old state was below the threshold!
  // This is the case when we have started up moving, and thus we need to wait for a period of stationary motion
  if (a_var_2to1 > params.init_imu_thresh && wait_for_jerk) {
    PRINT_DEBUG(YELLOW "[INIT-IMU]: to much IMU excitation, above threshold %.4f > %.4f\n" RESET, a_var_2to1, params.init_imu_thresh);
    return false;
  }

  // If it is above the threshold and we are not waiting for a jerk
  // Then we are not stationary (i.e. moving) so we should wait till we are
  if ((a_var_1to0 > params.init_imu_thresh || a_var_2to1 > params.init_imu_thresh) && !wait_for_jerk) {
    PRINT_DEBUG(YELLOW "[INIT-IMU]: to much IMU excitation, above threshold %.4f,%.4f > %.4f\n" RESET, a_var_1to0, a_var_2to1,
                params.init_imu_thresh);
    return false;
  }

  // Get z axis, which aligns with -g (z_in_G=0,0,1)
  Eigen::Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Vector3d y_axis = skew_x(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix3d Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  // Create our state variables
  Eigen::Matrix<double, 4, 1> q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  Eigen::Vector3d gravity_inG;
  gravity_inG << 0.0, 0.0, params.gravity_mag;
  Eigen::Vector3d bg = w_avg_2to1;
  Eigen::Vector3d ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * gravity_inG;

  // Set our state variables
  timestamp = window_2to1.at(window_2to1.size() - 1).timestamp;
  Eigen::VectorXd imu_state = Eigen::VectorXd::Zero(16);
  imu_state.block(0, 0, 4, 1) = q_GtoI;
  imu_state.block(10, 0, 3, 1) = bg;
  imu_state.block(13, 0, 3, 1) = ba;
  assert(t_imu != nullptr);
  t_imu->set_value(imu_state);
  t_imu->set_fej(imu_state);

  // Create base covariance and its covariance ordering
  order.clear();
  order.push_back(t_imu);
  covariance = 1e-3 * Eigen::MatrixXd::Identity(t_imu->size(), t_imu->size());

  // Make velocity uncertainty a bit bigger
  covariance.block(t_imu->v()->id(), t_imu->v()->id(), 3, 3) *= 2;

  // A VIO system has 4dof unobservabile directions which can be arbitarily picked.
  // This means that on startup, we can fix the yaw and position to be 100 percent known.
  // Thus, after determining the global to current IMU orientation after initialization, we can propagate the global error
  // into the new IMU pose. In this case the position is directly equivalent, but the orientation needs to be propagated.
  covariance(t_imu->q()->id() + 2, t_imu->q()->id() + 2) = 0.0;
  covariance.block(t_imu->p()->id(), t_imu->p()->id(), 3, 3).setZero();

  // Propagate into the current local IMU frame
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  Eigen::Matrix3d R_GtoI = quat_2_Rot(q_GtoI);
  covariance.block(t_imu->q()->id(), t_imu->q()->id(), 3, 3) =
      R_GtoI * covariance.block(t_imu->q()->id(), t_imu->q()->id(), 3, 3) * R_GtoI.transpose();

  // Return :D
  return true;
}
