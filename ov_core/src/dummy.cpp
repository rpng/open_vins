/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
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
 * @namespace ov_core
 *
 * This has the core algorithms that all projects within the Open VINS ecosystem leverage.
 * The purpose is to allow for the reuse of code that could be shared between different localization systems (i.e. msckf-based, batch-based, etc.).
 * These algorithms are the foundation which is necessary before we can even write an estimator that can perform localization.
 * The key components of the ov_core codebase are the following:
 *
 * - Closed-form preintegration @cite Eckenhoff2019IJRR
 * - 3d feature initialization
 * - Inertial state initialization
 * - Visual-inertial simulator and SE(3) b-spline
 * - KLT, descriptor, aruco, and simulation feature trackers
 * - Groundtruth dataset reader
 * - Quaternion and other manifold math operations
 * - Generic type system and their implementations
 *
 * Please take a look at classes that we offer for the user to leverage as each has its own documentation.
 * If you are looking for the estimator please take a look at the ov_msckf project which leverages these algorithms.
 *
 */
namespace ov_core { }





