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

#ifndef OV_INIT_CERES_IMAGEREPROJCALIB_H
#define OV_INIT_CERES_IMAGEREPROJCALIB_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <deque>
#include <iostream>
#include <map>

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "utils/quat_ops.h"

namespace ov_init {

/**
 * @brief Factor of feature bearing observation (raw) with calibration
 */
class Factor_ImageReprojCalib : public ceres::CostFunction {
public:
  // Measurement observation of the feature (raw pixel coordinates)
  Eigen::Vector2d uv_meas;

  // Measurement noise
  double pix_sigma = 1.0;
  Eigen::Matrix<double, 2, 2> sqrtQ;

  // If distortion model is fisheye or radtan
  bool is_fisheye = false;

  // If value of 1 then this residual adds to the problem, otherwise if zero it is "gated"
  double gate = 1.0;

  /**
   * @brief Default constructor
   * @param uv_meas_ Raw pixel uv measurement of a environmental feature
   * @param pix_sigma_ Raw pixel measurement uncertainty (typically 1)
   * @param is_fisheye_ If this raw pixel camera uses fisheye distortion
   */
  Factor_ImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_);

  virtual ~Factor_ImageReprojCalib() {}

  /**
   * @brief Error residual and Jacobian calculation
   *
   * This computes the Jacobians and residual of the feature projection model.
   * This is a function of the observing pose, feature in global, and calibration parameters.
   * The normalized pixel coordinates are found and then distorted using the camera distortion model.
   * See the @ref update-feat page for more details.
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_IMAGEREPROJCALIB_H