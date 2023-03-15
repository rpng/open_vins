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

#ifndef OV_TYPE_TYPE_POSEJPL_H
#define OV_TYPE_TYPE_POSEJPL_H

#include "JPLQuat.h"
#include "Vec.h"
#include "utils/quat_ops.h"

namespace ov_type {

/**
 * @brief Derived Type class that implements a 6 d.o.f pose
 *
 * Internally we use a JPLQuat quaternion representation for the orientation and 3D Vec position.
 * Please see JPLQuat for details on its update procedure and its left multiplicative error.
 */
class PoseJPL : public Type {

public:
  PoseJPL() : Type(6) {

    // Initialize subvariables
    _q = std::shared_ptr<JPLQuat>(new JPLQuat());
    _p = std::shared_ptr<Vec>(new Vec(3));

    // Set our default state value
    Eigen::Matrix<double, 7, 1> pose0;
    pose0.setZero();
    pose0(3) = 1.0;
    set_value_internal(pose0);
    set_fej_internal(pose0);
  }

  ~PoseJPL() {}

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that we update the sub-variables also.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  void set_local_id(int new_id) override {
    _id = new_id;
    _q->set_local_id(new_id);
    _p->set_local_id(new_id + ((new_id != -1) ? _q->size() : 0));
  }

  /**
   * @brief Update q and p using a the JPLQuat update for orientation and vector update for position
   *
   * @param dx Correction vector (orientation then position)
   */
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    Eigen::Matrix<double, 7, 1> newX = _value;

    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx.block(0, 0, 3, 1), 1.0;
    dq = ov_core::quatnorm(dq);

    // Update orientation
    newX.block(0, 0, 4, 1) = ov_core::quat_multiply(dq, quat());

    // Update position
    newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

    set_value(newX);
  }

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej(const Eigen::MatrixXd &new_value) override { set_fej_internal(new_value); }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<PoseJPL>(new PoseJPL());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) override {
    if (check == _q) {
      return _q;
    } else if (check == _p) {
      return _p;
    }
    return nullptr;
  }

  /// Rotation access
  Eigen::Matrix<double, 3, 3> Rot() const { return _q->Rot(); }

  /// FEJ Rotation access
  Eigen::Matrix<double, 3, 3> Rot_fej() const { return _q->Rot_fej(); }

  /// Rotation access as quaternion
  Eigen::Matrix<double, 4, 1> quat() const { return _q->value(); }

  /// FEJ Rotation access as quaternion
  Eigen::Matrix<double, 4, 1> quat_fej() const { return _q->fej(); }

  /// Position access
  Eigen::Matrix<double, 3, 1> pos() const { return _p->value(); }

  // FEJ position access
  Eigen::Matrix<double, 3, 1> pos_fej() const { return _p->fej(); }

  // Quaternion type access
  std::shared_ptr<JPLQuat> q() { return _q; }

  // Position type access
  std::shared_ptr<Vec> p() { return _p; }

protected:
  /// Subvariable containing orientation
  std::shared_ptr<JPLQuat> _q;

  /// Subvariable containing position
  std::shared_ptr<Vec> _p;

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    // Set orientation value
    _q->set_value(new_value.block(0, 0, 4, 1));

    // Set position value
    _p->set_value(new_value.block(4, 0, 3, 1));

    _value = new_value;
  }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    // Set orientation fej value
    _q->set_fej(new_value.block(0, 0, 4, 1));

    // Set position fej value
    _p->set_fej(new_value.block(4, 0, 3, 1));

    _fej = new_value;
  }
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_POSEJPL_H