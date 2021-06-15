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
 * @namespace ov_eval
 *
 * This project contains the key evaluation and research scripts for localization methods.
 * These come from the necessity of trying to quantify the accuracy of the estimated trajectory while
 * also allowing for the comparision to other methods.
 * Please see the following documentation pages for details:
 *
 * - @ref eval-metrics --- Definitions of different metrics for estimator accuracy.
 * - @ref eval-error --- Error evaluation methods for evaluating system performance.
 * - @ref eval-timing --- Timing of estimator components and complexity.
 *
 * The key methods that we have included are as follows:
 *
 * - Absolute trajectory error
 * - Relative pose error (for varying segment lengths)
 * - Pose to text file recorder
 * - Timing of system components
 *
 * The absolute and relative error scripts have been implemented in C++ to allow for fast computation on multiple runs.
 * We recommend that people look at the [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) toolbox provided
 * by Zhang and Scaramuzza. For a background we recommend reading their [A Tutorial on Quantitative Trajectory Evaluation for
 * Visual(-Inertial) Odometry](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) @cite Zhang2018IROS and its use in [A Benchmark Comparison of
 * Monocular Visual-Inertial Odometry Algorithms for Flying Robots](http://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf) @cite
 * Delmerico2018ICRA.
 *
 *
 */
namespace ov_eval {}
