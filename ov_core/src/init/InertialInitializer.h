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


#ifndef OV_CORE_INERTIALINITIALIZER_H
#define OV_CORE_INERTIALINITIALIZER_H

#include "utils/colors.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace ov_core {

/**
 * @brief Initializer for visual-inertial system.
 *
 * This class has a series of functions that can be used to initialize your system.
 * Right now we have our implementation that assumes that the imu starts from standing still.
 * In the future we plan to add support for structure-from-motion dynamic initialization.
 *
 * To initialize from standstill:
 * 1. Collect all inertial measurements
 * 2. See if within the last window there was a jump in acceleration
 * 3. If the jump is past our threshold we should init (i.e. we have started moving)
 * 4. Use the *previous* window, which should have been stationary to initialize orientation
 * 5. Return a roll and pitch aligned with gravity and biases.
 *
 */
class InertialInitializer {

public:
  /**
   * @brief Default constructor
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   * @param window_length Amount of time we will initialize over (seconds)
   * @param imu_excite_threshold Variance threshold on our acceleration to be classified as moving
   */
  InertialInitializer(double gravity_mag, double window_length, double imu_excite_threshold)
      : _window_length(window_length), _imu_excite_threshold(imu_excite_threshold) {
    _gravity << 0.0, 0.0, gravity_mag;
  }

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   */
  void feed_imu(const ImuData &message);

  /**
   * @brief Try to initialize the system using just the imu
   *
   * This will check if we have had a large enough jump in our acceleration.
   * If we have then we will use the period of time before this jump to initialize the state.
   * This assumes that our imu is sitting still and is not moving (so this would fail if we are experiencing constant acceleration).
   *
   * In the case that we do not wait for a jump (i.e. `wait_for_jerk` is false), then the system will try to initialize as soon as possible.
   * This is only recommended if you have zero velocity update enabled to handle the stationary cases.
   * To initialize in this case, we need to have the average angular variance be below the set threshold (i.e. we need to be stationary).
   *
   * @param[out] time0 Timestamp that the returned state is at
   * @param[out] q_GtoI0 Orientation at initialization
   * @param[out] b_w0 Gyro bias at initialization
   * @param[out] v_I0inG Velocity at initialization
   * @param[out] b_a0 Acceleration bias at initialization
   * @param[out] p_I0inG Position at initialization
   * @param wait_for_jerk If true we will wait for a "jerk"
   * @return True if we have successfully initialized our system
   */
  bool initialize_with_imu(double &time0, Eigen::Matrix<double, 4, 1> &q_GtoI0, Eigen::Matrix<double, 3, 1> &b_w0,
                           Eigen::Matrix<double, 3, 1> &v_I0inG, Eigen::Matrix<double, 3, 1> &b_a0, Eigen::Matrix<double, 3, 1> &p_I0inG,
                           bool wait_for_jerk = true);

protected:
  /// Gravity vector
  Eigen::Matrix<double, 3, 1> _gravity;

  /// Amount of time we will initialize over (seconds)
  double _window_length;

  /// Variance threshold on our acceleration to be classified as moving
  double _imu_excite_threshold;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ImuData> imu_data;
};

} // namespace ov_core

#endif // OV_CORE_INERTIALINITIALIZER_H
