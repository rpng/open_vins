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
 *
 *
 * We need to take special care to handle edge cases when converting to and from other rotation formats.
 * All equations are based on the following tech report @cite Trawny2005TR :
 *
 * > Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman filter for 3D attitude estimation."
 * > University of Minnesota, Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 * > http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 *
 * @section jplquat_define JPL Quaternion Definition
 *
 * We define the quaternion as the following linear combination:
 * @f[
 *  \bar{q} = q_4+q_1\mathbf{i}+q_2\mathbf{j}+q_3\mathbf{k}
 * @f]
 * Where i,j,k are defined as the following:
 * @f[
 *  \mathbf{i}^2=-1~,~\mathbf{j}^2=-1~,~\mathbf{k}^2=-1
 * @f]
 * @f[
 *  -\mathbf{i}\mathbf{j}=\mathbf{j}\mathbf{i}=\mathbf{k}
 *  ~,~
 *  -\mathbf{j}\mathbf{k}=\mathbf{k}\mathbf{j}=\mathbf{i}
 *  ~,~
 *  -\mathbf{k}\mathbf{i}=\mathbf{i}\mathbf{k}=\mathbf{j}
 * @f]
 * As noted in @cite Trawny2005TR this does not correspond to the Hamilton notation, and follows the "JPL Proposed Standard Conventions".
 * The q_4 quantity is the "scalar" portion of the quaternion, while q_1, q_2, q_3 are part of the "vector" portion.
 * We split the 4x1 vector into the following convention:
 * @f[
 *  \bar{q} = \begin{bmatrix}q_1\\q_2\\q_3\\q_4\end{bmatrix} = \begin{bmatrix}\mathbf{q}\\q_4\end{bmatrix}
 * @f]
 * It is also important to note that the quaternion is constrained to the unit circle:
 * @f[
 *  |\bar{q}| = \sqrt{\bar{q}^\top\bar{q}} = \sqrt{|\mathbf{q}|^2+q_4^2} = 1
 * @f]
 *
 *
 * @section jplquat_errorstate Error State Definition
 *
 * It is important to note that one can prove that the left-multiplicative quaternion error is equivalent to the SO(3) error.
 * If one wishes to use the right-hand error, this would need to be implemented as a different type then this class!
 * This is noted in Eq. (71) in @cite Trawny2005TR .
 * Specifically we have the following:
 * \f{align*}{
 * {}^{I}_G\bar{q} &\simeq \begin{bmatrix} \frac{1}{2} \delta \boldsymbol{\theta} \\ 1 \end{bmatrix} \otimes {}^{I}_G\hat{\bar{q}}
 * \f}
 * which is the same as:
 * \f{align*}{
 * {}^{I}_G \mathbf{R} &\simeq \exp(-\delta \boldsymbol{\theta}) {}^{I}_G \hat{\mathbf{R}} \\
 * &\simeq (\mathbf{I} - \lfloor \delta \boldsymbol{\theta} \rfloor) {}^{I}_G \hat{\mathbf{R}} \\
 * \f}
 *
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
   * \bar{q}=norm\Big(\begin{bmatrix} \frac{1}{2} \delta \boldsymbol{\theta}_{dx} \\ 1 \end{bmatrix}\Big) \otimes \hat{\bar{q}}
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
   * @param new_value New value for the quaternion estimate (JPL quat as x,y,z,w)
   */
  void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the fej value and recomputes the fej rotation matrix
   * @param new_value New value for the quaternion estimate (JPL quat as x,y,z,w)
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
