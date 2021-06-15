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


#ifndef OV_TYPE_TYPE_JPLQUAT_H
#define OV_TYPE_TYPE_JPLQUAT_H

#include "Type.h"
#include "utils/quat_ops.h"

namespace ov_type {

/**
 * @brief Derived Type class that implements JPL quaternion
 *
 * This quaternion uses a left-multiplicative error state and follows the "JPL convention".
 * Please checkout our utility functions in the quat_ops.h file.
 * We recommend that people new quaternions check out the following resources:
 * - http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 * - ftp://naif.jpl.nasa.gov/pub/naif/misc/Quaternion_White_Paper/Quaternions_White_Paper.pdf
 */
class JPLQuat : public Type {

public:
  JPLQuat() : Type(3) {
    Eigen::Vector4d q0 = Eigen::Vector4d::Zero();
    q0(3) = 1.0;
    set_value_internal(q0);
    set_fej_internal(q0);
  }

  ~JPLQuat() {}

  /**
   * @brief Implements update operation by left-multiplying the current
   * quaternion with a quaternion built from a small axis-angle perturbation.
   *
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   *
   * @param dx Axis-angle representation of the perturbing quaternion
   */
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    // Build perturbing quaternion
    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx, 1.0;
    dq = ov_core::quatnorm(dq);

    // Update estimate and recompute R
    set_value(ov_core::quat_multiply(dq, _value));
  }

  /**
   * @brief Sets the value of the estimate and recomputes the internal rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the fej value and recomputes the fej rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_fej(const Eigen::MatrixXd &new_value) override { set_fej_internal(new_value); }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<JPLQuat>(new JPLQuat());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  /// Rotation access
  Eigen::Matrix<double, 3, 3> Rot() const { return _R; }

  /// FEJ Rotation access
  Eigen::Matrix<double, 3, 3> Rot_fej() const { return _Rfej; }

protected:
  // Stores the rotation
  Eigen::Matrix<double, 3, 3> _R;

  // Stores the first-estimate rotation
  Eigen::Matrix<double, 3, 3> _Rfej;

  /**
   * @brief Sets the value of the estimate and recomputes the internal rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_value_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    _value = new_value;

    // compute associated rotation
    _R = ov_core::quat_2_Rot(new_value);
  }

  /**
   * @brief Sets the fej value and recomputes the fej rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_fej_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    _fej = new_value;

    // compute associated rotation
    _Rfej = ov_core::quat_2_Rot(new_value);
  }
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_JPLQUAT_H
