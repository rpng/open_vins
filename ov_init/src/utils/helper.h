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

#ifndef OV_INIT_HELPER
#define OV_INIT_HELPER

#include "cpi/CpiV1.h"
#include "types/IMU.h"

namespace ov_init {

/**
 * @brief Has a bunch of helper functions for dynamic initialization (should all be static)
 */
class InitializerHelper {
public:
  /**
   * @brief Nice helper function that will linearly interpolate between two imu messages.
   *
   * This should be used instead of just "cutting" imu messages that bound the camera times
   * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
   *
   * @param imu_1 imu at begining of interpolation interval
   * @param imu_2 imu at end of interpolation interval
   * @param timestamp Timestamp being interpolated to
   */
  static ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1, const ov_core::ImuData &imu_2, double timestamp) {
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    ov_core::ImuData data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }

  /**
   * @brief Helper function that given current imu data, will select imu readings between the two times.
   *
   * This will create measurements that we will integrate with, and an extra measurement at the end.
   * We use the @ref interpolate_data() function to "cut" the imu readings at the beginning and end of the integration.
   * The timestamps passed should already take into account the time offset values.
   *
   * @param imu_data_tmp IMU data we will select measurements from
   * @param time0 Start timestamp
   * @param time1 End timestamp
   * @return Vector of measurements (if we could compute them)
   */
  static std::vector<ov_core::ImuData> select_imu_readings(const std::vector<ov_core::ImuData> &imu_data_tmp, double time0, double time1) {
    // Our vector imu readings
    std::vector<ov_core::ImuData> prop_data;

    // Ensure we have some measurements in the first place!
    if (imu_data_tmp.empty()) {
      return prop_data;
    }

    // Loop through and find all the needed measurements to propagate with
    // Note we split measurements based on the given state time, and the update timestamp
    for (size_t i = 0; i < imu_data_tmp.size() - 1; i++) {

      // START OF THE INTEGRATION PERIOD
      if (imu_data_tmp.at(i + 1).timestamp > time0 && imu_data_tmp.at(i).timestamp < time0) {
        ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i), imu_data_tmp.at(i + 1), time0);
        prop_data.push_back(data);
        continue;
      }

      // MIDDLE OF INTEGRATION PERIOD
      if (imu_data_tmp.at(i).timestamp >= time0 && imu_data_tmp.at(i + 1).timestamp <= time1) {
        prop_data.push_back(imu_data_tmp.at(i));
        continue;
      }

      // END OF THE INTEGRATION PERIOD
      if (imu_data_tmp.at(i + 1).timestamp > time1) {
        if (imu_data_tmp.at(i).timestamp > time1 && i == 0) {
          break;
        } else if (imu_data_tmp.at(i).timestamp > time1) {
          ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i - 1), imu_data_tmp.at(i), time1);
          prop_data.push_back(data);
        } else {
          prop_data.push_back(imu_data_tmp.at(i));
        }
        if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
          ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i), imu_data_tmp.at(i + 1), time1);
          prop_data.push_back(data);
        }
        break;
      }
    }

    // Check that we have at least one measurement to propagate with
    if (prop_data.empty()) {
      return prop_data;
    }

    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (size_t i = 0; i < prop_data.size() - 1; i++) {
      if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12) {
        prop_data.erase(prop_data.begin() + i);
        i--;
      }
    }

    // Success :D
    return prop_data;
  }

  /**
   * @brief Given a gravity vector, compute the rotation from the inertial reference frame to this vector.
   *
   * The key assumption here is that our gravity is along the vertical direction in the inertial frame.
   * We can take this vector (z_in_G=0,0,1) and find two arbitrary tangent directions to it.
   * https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
   *
   * @param gravity_inI Gravity in our sensor frame
   * @param R_GtoI Rotation from the arbitrary inertial reference frame to this gravity vector
   */
  static void gram_schmidt(const Eigen::Vector3d &gravity_inI, Eigen::Matrix3d &R_GtoI) {

    // This will find an orthogonal vector to gravity which is our local z-axis
    // We need to ensure we normalize after each one such that we obtain unit vectors
    Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
    Eigen::Vector3d x_axis, y_axis;
    Eigen::Vector3d e_1(1.0, 0.0, 0.0);
    Eigen::Vector3d e_2(0.0, 1.0, 0.0);
    double inner1 = e_1.dot(z_axis) / z_axis.norm();
    double inner2 = e_2.dot(z_axis) / z_axis.norm();
    if (fabs(inner1) < fabs(inner2)) {
      x_axis = z_axis.cross(e_1);
      x_axis = x_axis / x_axis.norm();
      y_axis = z_axis.cross(x_axis);
      y_axis = y_axis / y_axis.norm();
    } else {
      x_axis = z_axis.cross(e_2);
      x_axis = x_axis / x_axis.norm();
      y_axis = z_axis.cross(x_axis);
      y_axis = y_axis / y_axis.norm();
    }

    // Original method
    // https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
    // x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    // x_axis = x_axis / x_axis.norm();
    // y_axis = ov_core::skew_x(z_axis) * x_axis;
    // y_axis = y_axis / y_axis.norm();

    // Rotation from our global (where gravity is only along the z-axis) to the local one
    R_GtoI.block(0, 0, 3, 1) = x_axis;
    R_GtoI.block(0, 1, 3, 1) = y_axis;
    R_GtoI.block(0, 2, 3, 1) = z_axis;
  }

  /**
   * @brief Compute coefficients for the constrained optimization quadratic problem.
   *
   * https://gist.github.com/goldbattle/3791cbb11bbf4f5feb3f049dad72bfdd
   *
   * @param D Gravity in our sensor frame
   * @param d Rotation from the arbitrary inertial reference frame to this gravity vector
   * @param gravity_mag Scalar size of gravity (normally is 9.81)
   * @return Coefficents from highest to the constant
   */
  static Eigen::Matrix<double, 7, 1> compute_dongsi_coeff(Eigen::MatrixXd &D, const Eigen::MatrixXd &d, double gravity_mag) {

    // matlab constants
    assert(D.rows() == 3);
    assert(D.cols() == 3);
    assert(d.rows() == 3);
    double D1_1 = D(0, 0), D1_2 = D(0, 1), D1_3 = D(0, 2);
    double D2_1 = D(1, 0), D2_2 = D(1, 1), D2_3 = D(1, 2);
    double D3_1 = D(2, 0), D3_2 = D(2, 1), D3_3 = D(2, 2);
    double d1 = d(0, 0), d2 = d(1, 0), d3 = d(2, 0);
    double g = gravity_mag;

    // squared version we subbed for x^2
    double D1_1_sq = D1_1 * D1_1, D1_2_sq = D1_2 * D1_2, D1_3_sq = D1_3 * D1_3;
    double D2_1_sq = D2_1 * D2_1, D2_2_sq = D2_2 * D2_2, D2_3_sq = D2_3 * D2_3;
    double D3_1_sq = D3_1 * D3_1, D3_2_sq = D3_2 * D3_2, D3_3_sq = D3_3 * D3_3;
    double d1_sq = d1 * d1, d2_sq = d2 * d2, d3_sq = d3 * d3;
    double g_sq = g * g;

    // Compute the coefficients
    Eigen::Matrix<double, 7, 1> coeff = Eigen::Matrix<double, 7, 1>::Zero();
    coeff(6) =
        -(-D1_1_sq * D2_2_sq * D3_3_sq * g_sq + D1_1_sq * D2_2_sq * d3_sq + 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * D3_3 * g_sq -
          D1_1_sq * D2_2 * D2_3 * d2 * d3 - D1_1_sq * D2_2 * D3_2 * d2 * d3 - D1_1_sq * D2_3_sq * D3_2_sq * g_sq +
          D1_1_sq * D2_3 * D3_2 * d2_sq + D1_1_sq * D2_3 * D3_2 * d3_sq - D1_1_sq * D2_3 * D3_3 * d2 * d3 -
          D1_1_sq * D3_2 * D3_3 * d2 * d3 + D1_1_sq * D3_3_sq * d2_sq + 2 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq -
          2 * D1_1 * D1_2 * D2_1 * D2_2 * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq + D1_1 * D1_2 * D2_1 * D2_3 * d2 * d3 +
          D1_1 * D1_2 * D2_1 * D3_2 * d2 * d3 - 2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_2 * D2_2 * D2_3 * d1 * d3 +
          D1_1 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq - D1_1 * D1_2 * D2_3 * D3_1 * d2_sq -
          D1_1 * D1_2 * D2_3 * D3_1 * d3_sq - D1_1 * D1_2 * D2_3 * D3_2 * d1 * d2 + D1_1 * D1_2 * D2_3 * D3_3 * d1 * d3 +
          D1_1 * D1_2 * D3_1 * D3_3 * d2 * d3 - D1_1 * D1_2 * D3_3_sq * d1 * d2 - 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq +
          D1_1 * D1_3 * D2_1 * D2_2 * d2 * d3 + 2 * D1_1 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq - D1_1 * D1_3 * D2_1 * D3_2 * d2_sq -
          D1_1 * D1_3 * D2_1 * D3_2 * d3_sq + D1_1 * D1_3 * D2_1 * D3_3 * d2 * d3 + 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq -
          D1_1 * D1_3 * D2_2_sq * d1 * d3 - 2 * D1_1 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq + D1_1 * D1_3 * D2_2 * D3_2 * d1 * d2 +
          D1_1 * D1_3 * D2_3 * D3_1 * d2 * d3 - D1_1 * D1_3 * D2_3 * D3_2 * d1 * d3 + D1_1 * D1_3 * D3_1 * D3_2 * d2 * d3 -
          2 * D1_1 * D1_3 * D3_1 * D3_3 * d2_sq + D1_1 * D1_3 * D3_2 * D3_3 * d1 * d2 + D1_1 * D2_1 * D2_2 * D3_2 * d1 * d3 -
          D1_1 * D2_1 * D2_3 * D3_2 * d1 * d2 + D1_1 * D2_1 * D3_2 * D3_3 * d1 * d3 - D1_1 * D2_1 * D3_3_sq * d1 * d2 -
          D1_1 * D2_2_sq * D3_1 * d1 * d3 + D1_1 * D2_2 * D2_3 * D3_1 * d1 * d2 - D1_1 * D2_3 * D3_1 * D3_2 * d1 * d3 +
          D1_1 * D2_3 * D3_1 * D3_3 * d1 * d2 - D1_2_sq * D2_1_sq * D3_3_sq * g_sq + D1_2_sq * D2_1_sq * d3_sq +
          2 * D1_2_sq * D2_1 * D2_3 * D3_1 * D3_3 * g_sq - D1_2_sq * D2_1 * D2_3 * d1 * d3 - D1_2_sq * D2_1 * D3_1 * d2 * d3 -
          D1_2_sq * D2_3_sq * D3_1_sq * g_sq + D1_2_sq * D2_3 * D3_1 * d1 * d2 + 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * D3_3 * g_sq -
          D1_2 * D1_3 * D2_1_sq * d2 * d3 - 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * D3_3 * g_sq + D1_2 * D1_3 * D2_1 * D2_2 * d1 * d3 -
          2 * D1_2 * D1_3 * D2_1 * D2_3 * D3_1 * D3_2 * g_sq + D1_2 * D1_3 * D2_1 * D3_1 * d2_sq + D1_2 * D1_3 * D2_1 * D3_1 * d3_sq -
          D1_2 * D1_3 * D2_1 * D3_3 * d1 * d3 + 2 * D1_2 * D1_3 * D2_2 * D2_3 * D3_1_sq * g_sq - D1_2 * D1_3 * D2_2 * D3_1 * d1 * d2 -
          D1_2 * D1_3 * D3_1_sq * d2 * d3 + D1_2 * D1_3 * D3_1 * D3_3 * d1 * d2 - D1_2 * D2_1_sq * D3_2 * d1 * d3 +
          D1_2 * D2_1 * D2_2 * D3_1 * d1 * d3 + D1_2 * D2_1 * D2_3 * D3_2 * d1_sq + D1_2 * D2_1 * D2_3 * D3_2 * d3_sq -
          D1_2 * D2_1 * D2_3 * D3_3 * d2 * d3 - D1_2 * D2_1 * D3_1 * D3_3 * d1 * d3 - D1_2 * D2_1 * D3_2 * D3_3 * d2 * d3 +
          D1_2 * D2_1 * D3_3_sq * d1_sq + D1_2 * D2_1 * D3_3_sq * d2_sq - D1_2 * D2_2 * D2_3 * D3_1 * d1_sq -
          D1_2 * D2_2 * D2_3 * D3_1 * d3_sq + D1_2 * D2_2 * D2_3 * D3_3 * d1 * d3 + D1_2 * D2_2 * D3_1 * D3_3 * d2 * d3 -
          D1_2 * D2_2 * D3_3_sq * d1 * d2 + D1_2 * D2_3_sq * D3_1 * d2 * d3 - D1_2 * D2_3_sq * D3_2 * d1 * d3 +
          D1_2 * D2_3 * D3_1_sq * d1 * d3 - D1_2 * D2_3 * D3_1 * D3_3 * d1_sq - D1_2 * D2_3 * D3_1 * D3_3 * d2_sq +
          D1_2 * D2_3 * D3_2 * D3_3 * d1 * d2 - D1_3_sq * D2_1_sq * D3_2_sq * g_sq + 2 * D1_3_sq * D2_1 * D2_2 * D3_1 * D3_2 * g_sq -
          D1_3_sq * D2_1 * D3_1 * d2 * d3 + D1_3_sq * D2_1 * D3_2 * d1 * d3 - D1_3_sq * D2_2_sq * D3_1_sq * g_sq +
          D1_3_sq * D3_1_sq * d2_sq - D1_3_sq * D3_1 * D3_2 * d1 * d2 + D1_3 * D2_1_sq * D3_2 * d1 * d2 -
          D1_3 * D2_1 * D2_2 * D3_1 * d1 * d2 - D1_3 * D2_1 * D2_2 * D3_2 * d1_sq - D1_3 * D2_1 * D2_2 * D3_2 * d3_sq +
          D1_3 * D2_1 * D2_2 * D3_3 * d2 * d3 + D1_3 * D2_1 * D3_1 * D3_3 * d1 * d2 + D1_3 * D2_1 * D3_2_sq * d2 * d3 -
          D1_3 * D2_1 * D3_2 * D3_3 * d1_sq - D1_3 * D2_1 * D3_2 * D3_3 * d2_sq + D1_3 * D2_2_sq * D3_1 * d1_sq +
          D1_3 * D2_2_sq * D3_1 * d3_sq - D1_3 * D2_2_sq * D3_3 * d1 * d3 - D1_3 * D2_2 * D2_3 * D3_1 * d2 * d3 +
          D1_3 * D2_2 * D2_3 * D3_2 * d1 * d3 - D1_3 * D2_2 * D3_1 * D3_2 * d2 * d3 + D1_3 * D2_2 * D3_2 * D3_3 * d1 * d2 -
          D1_3 * D2_3 * D3_1_sq * d1 * d2 + D1_3 * D2_3 * D3_1 * D3_2 * d1_sq + D1_3 * D2_3 * D3_1 * D3_2 * d2_sq -
          D1_3 * D2_3 * D3_2_sq * d1 * d2 + D2_1 * D2_2 * D3_2 * D3_3 * d1 * d3 - D2_1 * D2_2 * D3_3_sq * d1 * d2 -
          D2_1 * D2_3 * D3_2_sq * d1 * d3 + D2_1 * D2_3 * D3_2 * D3_3 * d1 * d2 - D2_2_sq * D3_1 * D3_3 * d1 * d3 +
          D2_2_sq * D3_3_sq * d1_sq + D2_2 * D2_3 * D3_1 * D3_2 * d1 * d3 + D2_2 * D2_3 * D3_1 * D3_3 * d1 * d2 -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * d1_sq - D2_3_sq * D3_1 * D3_2 * d1 * d2 + D2_3_sq * D3_2_sq * d1_sq) /
        g_sq;
    coeff(5) =
        (-(2 * D1_1_sq * D2_2_sq * D3_3 * g_sq - 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * g_sq + 2 * D1_1_sq * D2_2 * D3_3_sq * g_sq -
           2 * D1_1_sq * D2_2 * d3_sq - 2 * D1_1_sq * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1_sq * D2_3 * d2 * d3 +
           2 * D1_1_sq * D3_2 * d2 * d3 - 2 * D1_1_sq * D3_3 * d2_sq - 4 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq +
           2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_1 * D1_2 * D2_1 * D3_3_sq * g_sq + 2 * D1_1 * D1_2 * D2_1 * d3_sq +
           2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq + 2 * D1_1 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_1 * D1_2 * D2_3 * d1 * d3 -
           2 * D1_1 * D1_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D3_3 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 2 * D1_1 * D1_3 * D2_1 * d2 * d3 - 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * g_sq -
           4 * D1_1 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 2 * D1_1 * D1_3 * D2_2 * d1 * d3 + 2 * D1_1 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D3_1 * d2_sq - 2 * D1_1 * D1_3 * D3_2 * d1 * d2 - 2 * D1_1 * D2_1 * D3_2 * d1 * d3 +
           2 * D1_1 * D2_1 * D3_3 * d1 * d2 + 2 * D1_1 * D2_2_sq * D3_3_sq * g_sq - 2 * D1_1 * D2_2_sq * d3_sq -
           4 * D1_1 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1 * D2_2 * D2_3 * d2 * d3 + 2 * D1_1 * D2_2 * D3_1 * d1 * d3 +
           2 * D1_1 * D2_2 * D3_2 * d2 * d3 + 2 * D1_1 * D2_3_sq * D3_2_sq * g_sq - 2 * D1_1 * D2_3 * D3_1 * d1 * d2 -
           2 * D1_1 * D2_3 * D3_2 * d2_sq - 2 * D1_1 * D2_3 * D3_2 * d3_sq + 2 * D1_1 * D2_3 * D3_3 * d2 * d3 +
           2 * D1_1 * D3_2 * D3_3 * d2 * d3 - 2 * D1_1 * D3_3_sq * d2_sq + 2 * D1_2_sq * D2_1_sq * D3_3 * g_sq -
           2 * D1_2_sq * D2_1 * D2_3 * D3_1 * g_sq - 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * g_sq + 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * g_sq +
           2 * D1_2 * D1_3 * D2_1 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D1_3 * D2_3 * D3_1_sq * g_sq - 2 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq +
           2 * D1_2 * D2_1 * D2_2 * d3_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq - 2 * D1_2 * D2_1 * D3_3 * d1_sq -
           2 * D1_2 * D2_1 * D3_3 * d2_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D2_2 * D2_3 * d1 * d3 -
           2 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_2 * D2_2 * D3_3 * d1 * d2 - 2 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq +
           2 * D1_2 * D2_3 * D3_1 * d1_sq + 2 * D1_2 * D2_3 * D3_1 * d2_sq + 2 * D1_2 * D2_3 * D3_1 * d3_sq -
           2 * D1_2 * D2_3 * D3_3 * d1 * d3 - 2 * D1_2 * D3_1 * D3_3 * d2 * d3 + 2 * D1_2 * D3_3_sq * d1 * d2 -
           2 * D1_3_sq * D2_1 * D3_1 * D3_2 * g_sq + 2 * D1_3_sq * D2_2 * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq -
           2 * D1_3 * D2_1 * D2_2 * d2 * d3 - 2 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq + 2 * D1_3 * D2_1 * D3_2 * d1_sq +
           2 * D1_3 * D2_1 * D3_2 * d2_sq + 2 * D1_3 * D2_1 * D3_2 * d3_sq - 2 * D1_3 * D2_1 * D3_3 * d2 * d3 -
           2 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq + 2 * D1_3 * D2_2_sq * d1 * d3 + 2 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq -
           2 * D1_3 * D2_2 * D3_1 * d1_sq - 2 * D1_3 * D2_2 * D3_1 * d3_sq - 2 * D1_3 * D2_2 * D3_2 * d1 * d2 +
           2 * D1_3 * D2_2 * D3_3 * d1 * d3 + 2 * D1_3 * D3_1 * D3_3 * d2_sq - 2 * D1_3 * D3_2 * D3_3 * d1 * d2 -
           2 * D2_1 * D2_2 * D3_2 * d1 * d3 + 2 * D2_1 * D2_2 * D3_3 * d1 * d2 - 2 * D2_1 * D3_2 * D3_3 * d1 * d3 +
           2 * D2_1 * D3_3_sq * d1 * d2 + 2 * D2_2_sq * D3_1 * d1 * d3 - 2 * D2_2_sq * D3_3 * d1_sq - 2 * D2_2 * D2_3 * D3_1 * d1 * d2 +
           2 * D2_2 * D2_3 * D3_2 * d1_sq + 2 * D2_2 * D3_1 * D3_3 * d1 * d3 - 2 * D2_2 * D3_3_sq * d1_sq -
           2 * D2_3 * D3_1 * D3_3 * d1 * d2 + 2 * D2_3 * D3_2 * D3_3 * d1_sq) /
         g_sq);
    coeff(4) =
        ((D1_1_sq * D2_2_sq * g_sq + 4 * D1_1_sq * D2_2 * D3_3 * g_sq - 2 * D1_1_sq * D2_3 * D3_2 * g_sq + D1_1_sq * D3_3_sq * g_sq -
          D1_1_sq * d2_sq - D1_1_sq * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_2 * g_sq - 4 * D1_1 * D1_2 * D2_1 * D3_3 * g_sq +
          2 * D1_1 * D1_2 * D2_3 * D3_1 * g_sq + D1_1 * D1_2 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D3_2 * g_sq -
          4 * D1_1 * D1_3 * D2_2 * D3_1 * g_sq - 2 * D1_1 * D1_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_3 * d1 * d3 + D1_1 * D2_1 * d1 * d2 +
          4 * D1_1 * D2_2_sq * D3_3 * g_sq - 4 * D1_1 * D2_2 * D2_3 * D3_2 * g_sq + 4 * D1_1 * D2_2 * D3_3_sq * g_sq -
          4 * D1_1 * D2_2 * d3_sq - 4 * D1_1 * D2_3 * D3_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * d2 * d3 + D1_1 * D3_1 * d1 * d3 +
          4 * D1_1 * D3_2 * d2 * d3 - 4 * D1_1 * D3_3 * d2_sq + D1_2_sq * D2_1_sq * g_sq + 2 * D1_2 * D1_3 * D2_1 * D3_1 * g_sq -
          4 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_2 * D2_1 * D3_3_sq * g_sq -
          D1_2 * D2_1 * d1_sq - D1_2 * D2_1 * d2_sq + 2 * D1_2 * D2_1 * d3_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq +
          D1_2 * D2_2 * d1 * d2 + 2 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 3 * D1_2 * D2_3 * d1 * d3 - 3 * D1_2 * D3_1 * d2 * d3 +
          4 * D1_2 * D3_3 * d1 * d2 + D1_3_sq * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
          2 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 3 * D1_3 * D2_1 * d2 * d3 - 2 * D1_3 * D2_2_sq * D3_1 * g_sq -
          4 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 4 * D1_3 * D2_2 * d1 * d3 + 2 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq - D1_3 * D3_1 * d1_sq +
          2 * D1_3 * D3_1 * d2_sq - D1_3 * D3_1 * d3_sq - 3 * D1_3 * D3_2 * d1 * d2 + D1_3 * D3_3 * d1 * d3 + D2_1 * D2_2 * d1 * d2 -
          3 * D2_1 * D3_2 * d1 * d3 + 4 * D2_1 * D3_3 * d1 * d2 + D2_2_sq * D3_3_sq * g_sq - D2_2_sq * d1_sq - D2_2_sq * d3_sq -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + D2_2 * D2_3 * d2 * d3 + 4 * D2_2 * D3_1 * d1 * d3 + D2_2 * D3_2 * d2 * d3 -
          4 * D2_2 * D3_3 * d1_sq + D2_3_sq * D3_2_sq * g_sq - 3 * D2_3 * D3_1 * d1 * d2 + 2 * D2_3 * D3_2 * d1_sq - D2_3 * D3_2 * d2_sq -
          D2_3 * D3_2 * d3_sq + D2_3 * D3_3 * d2 * d3 + D3_1 * D3_3 * d1 * d3 + D3_2 * D3_3 * d2 * d3 - D3_3_sq * d1_sq - D3_3_sq * d2_sq) /
         g_sq);
    coeff(3) =
        ((2 * D1_1 * d2_sq + 2 * D1_1 * d3_sq + 2 * D2_2 * d1_sq + 2 * D2_2 * d3_sq + 2 * D3_3 * d1_sq + 2 * D3_3 * d2_sq -
          2 * D1_1 * D2_2_sq * g_sq - 2 * D1_1_sq * D2_2 * g_sq - 2 * D1_1 * D3_3_sq * g_sq - 2 * D1_1_sq * D3_3 * g_sq -
          2 * D2_2 * D3_3_sq * g_sq - 2 * D2_2_sq * D3_3 * g_sq - 2 * D1_2 * d1 * d2 - 2 * D1_3 * d1 * d3 - 2 * D2_1 * d1 * d2 -
          2 * D2_3 * d2 * d3 - 2 * D3_1 * d1 * d3 - 2 * D3_2 * d2 * d3 + 2 * D1_1 * D1_2 * D2_1 * g_sq + 2 * D1_1 * D1_3 * D3_1 * g_sq +
          2 * D1_2 * D2_1 * D2_2 * g_sq - 8 * D1_1 * D2_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * D3_2 * g_sq + 4 * D1_2 * D2_1 * D3_3 * g_sq -
          2 * D1_2 * D2_3 * D3_1 * g_sq - 2 * D1_3 * D2_1 * D3_2 * g_sq + 4 * D1_3 * D2_2 * D3_1 * g_sq + 2 * D1_3 * D3_1 * D3_3 * g_sq +
          2 * D2_2 * D2_3 * D3_2 * g_sq + 2 * D2_3 * D3_2 * D3_3 * g_sq) /
         g_sq);
    coeff(2) =
        (-(d1_sq + d2_sq + d3_sq - D1_1_sq * g_sq - D2_2_sq * g_sq - D3_3_sq * g_sq - 4 * D1_1 * D2_2 * g_sq + 2 * D1_2 * D2_1 * g_sq -
           4 * D1_1 * D3_3 * g_sq + 2 * D1_3 * D3_1 * g_sq - 4 * D2_2 * D3_3 * g_sq + 2 * D2_3 * D3_2 * g_sq) /
         g_sq);
    coeff(1) = (-(2 * D1_1 * g_sq + 2 * D2_2 * g_sq + 2 * D3_3 * g_sq) / g_sq);
    coeff(0) = 1;

    // finally return
    return coeff;
  }
};
} // namespace ov_init

#endif /* OV_INIT_HELPER */
