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

/**
 * @namespace ov_msckf
 * @brief Extended Kalman Filter estimator
 *
 * This is an implementation of a [Multi-State Constraint Kalman Filter (MSCKF)](https://ieeexplore.ieee.org/document/4209642) @cite
 * Mourikis2007ICRA which leverages inertial and visual feature information. We want to stress that this is **not** a "vanilla"
 * implementation of the filter and instead has many more features and improvements over the original. In additional we have a modular type
 * system which allows us to initialize and marginalizing variables out of state with ease. Please see the following documentation pages for
 * derivation details:
 *
 * - @ref propagation --- Inertial propagation derivations and details.
 * - @ref update --- General state update for the different measurements.
 * - @ref fej --- Background on First-Estimate Jacobians and how we use them.
 * - @ref dev-index --- High level details on how the type system and covariance management works.
 *
 * The key features of the system are the following:
 *
 * - Sliding stochastic imu clones
 * - First-Estimate Jacobians to maintain consistency
 * - Camera intrinsics and extrinsic online calibration
 * - Time offset between camera and imu calibration
 * - Inertial intrinsic calibration (including g-sensitivity)
 * - Standard MSCKF features with nullspace projection
 * - 3d SLAM feature support (with six different representations)
 * - Generic simulation of trajectories through and environment (see @ref ov_msckf::Simulator)
 *
 * We suggest those that are interested to first checkout the State and Propagator which should provide a nice introduction to the code.
 * Both the slam and msckf features leverage the same Jacobian code, and thus we also recommend looking at the UpdaterHelper class for
 * details on that.
 *
 */
namespace ov_msckf {}
