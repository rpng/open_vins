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

/**
 * @namespace ov_init
 *
 * This is an implementation and extension of the work [Estimator initialization in vision-aided inertial navigation with unknown camera-IMU
 * calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization problem by first creating a
 * linear system for recovering the camera to IMU rotation, then for velocity, gravity, and feature positions, and finally a full
 * optimization to allow for covariance recovery. Specifically, we focus on also recovering the biases of the platform and the time offset.
 *
 */
namespace ov_init {}
