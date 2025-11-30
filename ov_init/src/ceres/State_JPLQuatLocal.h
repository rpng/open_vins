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

#ifndef OV_INIT_CERES_JPLQUATLOCAL_H
#define OV_INIT_CERES_JPLQUATLOCAL_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief JPL quaternion CERES state parameterization
 */
class State_JPLQuatLocal : public ceres::Manifold {
public:
  /**
   * @brief State update function for a JPL quaternion representation.
   *
   * Implements update operation by left-multiplying the current
   * quaternion with a quaternion built from a small axis-angle perturbation.
   *
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   */
  int AmbientSize() const override {
    return 4; // For example, if this represents a 4D state (quaternion)
}

int TangentSize() const override {
    return 3; // Local perturbation size, typically 3 for a rotation
}

bool PlusJacobian(const double* x, double* jacobian) const override {
    // Implement the jacobian computation for the Plus operation
    return true;
}
bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
bool ComputeJacobian(const double* x, double* jacobian) const;


bool Minus(const double* y, const double* x, double* delta) const override {
    // Implement the minus operation
    return true;
}

bool MinusJacobian(const double* x, double* jacobian) const override {
    // Implement the jacobian for the Minus operation
    return true;
}

};

} // namespace ov_init

#endif // OV_INIT_CERES_JPLQUATLOCAL_H