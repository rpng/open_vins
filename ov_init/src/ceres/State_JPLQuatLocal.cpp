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

#include "State_JPLQuatLocal.h"

#include "utils/quat_ops.h"

using namespace ov_init;

bool State_JPLQuatLocal::Plus(const double *x, const double *delta, double *x_plus_delta) const {

  // Apply the standard JPL update: q <-- [d_th/2; 1] (x) q
  Eigen::Map<const Eigen::Vector4d> q(x);

  // Get delta into eigen
  Eigen::Map<const Eigen::Vector3d> d_th(delta);
  Eigen::Matrix<double, 4, 1> d_q;
  double theta = d_th.norm();
  if (theta < 1e-8) {
    d_q << .5 * d_th, 1.0;
  } else {
    d_q.block(0, 0, 3, 1) = (d_th / theta) * std::sin(theta / 2);
    d_q(3, 0) = std::cos(theta / 2);
  }
  d_q = ov_core::quatnorm(d_q);

  // Do the update
  Eigen::Map<Eigen::Vector4d> q_plus(x_plus_delta);
  q_plus = ov_core::quat_multiply(d_q, q);
  return true;
}

#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2
bool State_JPLQuatLocal::PlusJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
  j.topRows<3>().setIdentity();
  j.bottomRows<1>().setZero();
  return true;
}

bool State_JPLQuatLocal::Minus(const double *y, const double *x, double *delta) const {
  Eigen::Map<const Eigen::Vector4d> q1(x);
  Eigen::Map<const Eigen::Vector4d> q2(y);
  Eigen::Vector4d q_rel = ov_core::quat_multiply(q2, ov_core::Inv(q1));
  Eigen::Vector3d omega;

  double qw = q_rel(3);
  Eigen::Vector3d qv = q_rel.head<3>();

  double norm_qv = qv.norm();
  if (norm_qv < 1e-8) {
    omega.setZero();
  } else {
    double angle = 2.0 * std::atan2(norm_qv, qw);
    omega = angle * (qv / norm_qv);
  }

  Eigen::Map<Eigen::Vector3d> d_out(delta);
  d_out = omega;
  return true;
}

bool State_JPLQuatLocal::MinusJacobian(const double *x, double *jacobian) const {
  // This is an approximation: ∂delta/∂x ≈ [I; 0]
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> j(jacobian);
  j.setZero();
  j.leftCols<3>().setIdentity();
  return true;
}

#else

bool State_JPLQuatLocal::ComputeJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
  j.topRows<3>().setIdentity();
  j.bottomRows<1>().setZero();
  return true;
}

#endif
