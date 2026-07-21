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

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "state/Propagator.h"
#include "state/State.h"

using namespace ov_msckf;

int main() {
  constexpr double gravity = 9.81;
  constexpr double state_time_cam = 1.0;
  constexpr double cam_to_imu_offset = 0.05;
  constexpr double requested_time_imu = 1.15;
  constexpr double expected_dt = 0.10;

  StateOptions state_options;
  auto state = std::make_shared<State>(state_options);
  state->_timestamp = state_time_cam;
  state->_calib_dt_CAMtoIMU->set_value(
      Eigen::VectorXd::Constant(1, cam_to_imu_offset));

  Propagator propagator(NoiseManager(), gravity);
  for (int i = 0; i <= 11; ++i) {
    ov_core::ImuData imu;
    imu.timestamp = 1.04 + 0.01 * i;
    imu.wm.setZero();
    imu.am << 1.0, 0.0, gravity;
    propagator.feed_imu(imu);
  }

  Eigen::Matrix<double, 13, 1> state_plus;
  Eigen::Matrix<double, 12, 12> covariance;
  if (!propagator.fast_state_propagate(
          state, requested_time_imu, state_plus, covariance)) {
    std::cerr << "Fast propagation unexpectedly failed." << std::endl;
    return EXIT_FAILURE;
  }

  const double expected_position_x = 0.5 * expected_dt * expected_dt;
  const double expected_velocity_x = expected_dt;
  if (std::abs(state_plus(4) - expected_position_x) > 1e-6 ||
      std::abs(state_plus(7) - expected_velocity_x) > 1e-6) {
    std::cerr << "Fast propagation used the wrong clock interval: p_x="
              << state_plus(4) << ", v_x=" << state_plus(7) << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
