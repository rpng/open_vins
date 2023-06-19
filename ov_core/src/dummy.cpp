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
 * @namespace ov_core
 * @brief Core algorithms for OpenVINS
 *
 * This has the core algorithms that all projects within the OpenVINS ecosystem leverage.
 * The purpose is to allow for the reuse of code that could be shared between different localization systems (i.e. msckf-based, batch-based,
 * etc.). These algorithms are the foundation which is necessary before we can even write an estimator that can perform localization. The
 * key components of the ov_core codebase are the following:
 *
 * - 3d feature initialization (see @ref ov_core::FeatureInitializer)
 * - SE(3) b-spline (see @ref ov_core::BsplineSE3)
 * - KLT, descriptor, aruco, and simulation feature trackers
 * - Groundtruth dataset reader (see @ref ov_core::DatasetReader)
 * - Quaternion and other manifold math operations
 * - Generic type system and their implementations (see @ref ov_type namespace)
 * - Closed-form preintegration @cite Eckenhoff2019IJRR
 *
 * Please take a look at classes that we offer for the user to leverage as each has its own documentation.
 * If you are looking for the estimator please take a look at the ov_msckf project which leverages these algorithms.
 * If you are looking for the different types please take a look at the ov_type namespace for the ones we have.
 *
 */
namespace ov_core {}

/**
 * @namespace ov_type
 * @brief Dynamic type system types
 *
 * Types leveraged by the EKF system for covariance management.
 * These types store where they are in the covariance along with their current estimate.
 * Each also has an update function that takes a vector delta and updates their manifold representation.
 * Please see each type for details on what they represent, but their names should be straightforward.
 * See @ref dev-index for high level details on how the type system and covariance management works.
 * Each type is described by the following:
 *
 * @code{.cpp}
 * class Type {
 * protected:
 *   // Current best estimate
 *   Eigen::MatrixXd _value;
 *   // Location of error state in covariance
 *   int _id = -1;
 *   // Dimension of error state
 *   int _size = -1;
 *   // Update eq. taking vector to their rep.
 *   void update(const Eigen::VectorXd dx);
 * };
 * @endcode
 *
 * When deriving Jacobians, it is important to ensure that the error state used matches the type.
 * Each type updates an update function, and thus directly defines the error state it has.
 * A type with non-trivial error states is the @ref ov_type::JPLQuat which has equivalent quaterion and SO(3) error.
 * For rotations and on-manifold representations, [State Estimation for Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf)
 * by Timothy D. Barfoot @cite Barfoot2017 covers a nice range of examples.
 *
 */
namespace ov_type {}
