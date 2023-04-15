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
 * @namespace ov_init
 * @brief State initialization code
 *
 * Right now this contains StaticInitializer and DynamicInitializer initialization code for a visual-inertial system.
 * It will wait for the platform to stationary, and then initialize its orientation in the gravity frame.
 *
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 *
 * If the platform is not stationary then we leverage dynamic initialization to try to recover the initial state.
 * This is an implementation of the work [Estimator initialization in vision-aided inertial navigation with unknown camera-IMU
 * calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization problem by first creating a
 * linear system for recovering tthe velocity, gravity, and feature positions.
 * After the initial recovery, a full optimization is performed to allow for covariance recovery.
 * See this [tech report](https://pgeneva.com/downloads/reports/tr_init.pdf) for a high level walk through.
 */
namespace ov_init {}
