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

#include "Factor_ImageReprojCalib.h"

#include "utils/quat_ops.h"

using namespace ov_init;

Factor_ImageReprojCalib::Factor_ImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_)
    : uv_meas(uv_meas_), pix_sigma(pix_sigma_), is_fisheye(is_fisheye_) {

  // Square root information inverse
  sqrtQ = Eigen::Matrix<double, 2, 2>::Identity();
  sqrtQ(0, 0) *= 1.0 / pix_sigma;
  sqrtQ(1, 1) *= 1.0 / pix_sigma;

  // Parameters we are a function of
  set_num_residuals(2);
  mutable_parameter_block_sizes()->push_back(4); // q_GtoIi
  mutable_parameter_block_sizes()->push_back(3); // p_IiinG
  mutable_parameter_block_sizes()->push_back(3); // p_FinG
  mutable_parameter_block_sizes()->push_back(4); // q_ItoC
  mutable_parameter_block_sizes()->push_back(3); // p_IinC
  mutable_parameter_block_sizes()->push_back(8); // focal, center, distortion
}

bool Factor_ImageReprojCalib::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

  // Recover the current state from our parameters
  Eigen::Vector4d q_GtoIi = Eigen::Map<const Eigen::Vector4d>(parameters[0]);
  Eigen::Matrix3d R_GtoIi = ov_core::quat_2_Rot(q_GtoIi);
  Eigen::Vector3d p_IiinG = Eigen::Map<const Eigen::Vector3d>(parameters[1]);
  Eigen::Vector3d p_FinG = Eigen::Map<const Eigen::Vector3d>(parameters[2]);
  Eigen::Vector4d q_ItoC = Eigen::Map<const Eigen::Vector4d>(parameters[3]);
  Eigen::Matrix3d R_ItoC = ov_core::quat_2_Rot(q_ItoC);
  Eigen::Vector3d p_IinC = Eigen::Map<const Eigen::Vector3d>(parameters[4]);

  // order: f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 & k_4
  Eigen::Matrix<double, 8, 1> camera_vals = Eigen::Map<const Eigen::Matrix<double, 8, 1>>(parameters[5]);

  // Transform the feature into the current camera frame of reference
  Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_IiinG);
  Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;

  // Normalized projected feature bearing
  Eigen::Vector2d uv_norm;
  uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

  // Square-root information and gate
  Eigen::Matrix<double, 2, 2> sqrtQ_gate = gate * sqrtQ;

  // Get the distorted raw image coordinate using the camera model
  // Also if jacobians are requested, then compute derivatives
  Eigen::Vector2d uv_dist;
  Eigen::MatrixXd H_dz_dzn, H_dz_dzeta;
  if (is_fisheye) {
    ov_core::CamEqui cam(0, 0);
    cam.set_value(camera_vals);
    uv_dist = cam.distort_d(uv_norm);
    if (jacobians) {
      cam.compute_distort_jacobian(uv_norm, H_dz_dzn, H_dz_dzeta);
      H_dz_dzn = sqrtQ_gate * H_dz_dzn;
      H_dz_dzeta = sqrtQ_gate * H_dz_dzeta;
    }
  } else {
    ov_core::CamRadtan cam(0, 0);
    cam.set_value(camera_vals);
    uv_dist = cam.distort_d(uv_norm);
    if (jacobians) {
      cam.compute_distort_jacobian(uv_norm, H_dz_dzn, H_dz_dzeta);
      H_dz_dzn = sqrtQ_gate * H_dz_dzn;
      H_dz_dzeta = sqrtQ_gate * H_dz_dzeta;
    }
  }

  // Compute residual
  // NOTE: we make this negative ceres cost function is only min||f(x)||^2
  // NOTE: thus since we have found the derivative of uv_meas = f(x) + n
  // NOTE: we need to reformulate into a zero error constraint 0 = f(x) + n - uv_meas
  // NOTE: if it was the other way (0 = uv_meas - f(x) - n) all the Jacobians would need to be flipped
  Eigen::Vector2d res = uv_dist - uv_meas;
  res = sqrtQ_gate * res;
  residuals[0] = res(0);
  residuals[1] = res(1);

  // Compute jacobians if requested by ceres
  if (jacobians) {

    // Normalized coordinates in respect to projection function
    Eigen::MatrixXd H_dzn_dpfc = Eigen::MatrixXd::Zero(2, 3);
    H_dzn_dpfc << 1.0 / p_FinCi(2), 0, -p_FinCi(0) / std::pow(p_FinCi(2), 2), 0, 1.0 / p_FinCi(2), -p_FinCi(1) / std::pow(p_FinCi(2), 2);
    Eigen::MatrixXd H_dz_dpfc = H_dz_dzn * H_dzn_dpfc;

    // Jacobian wrt q_GtoIi
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian(jacobians[0]);
      jacobian.block(0, 0, 2, 3) = H_dz_dpfc * R_ItoC * ov_core::skew_x(p_FinIi);
      jacobian.block(0, 3, 2, 1).setZero();
    }

    // Jacobian wrt p_IiinG
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[1]);
      jacobian.block(0, 0, 2, 3) = -H_dz_dpfc * R_ItoC * R_GtoIi;
    }

    // Jacobian wrt feature p_FinG
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[2]);
      jacobian.block(0, 0, 2, 3) = H_dz_dpfc * R_ItoC * R_GtoIi;
    }

    // Jacbian wrt IMU-camera transform q_ItoC
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian(jacobians[3]);
      jacobian.block(0, 0, 2, 3) = H_dz_dpfc * ov_core::skew_x(R_ItoC * p_FinIi);
      jacobian.block(0, 3, 2, 1).setZero();
    }

    // Jacbian wrt IMU-camera transform p_IinC
    if (jacobians[4]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian(jacobians[4]);
      jacobian.block(0, 0, 2, 3) = H_dz_dpfc;
    }

    // Jacbian wrt camera intrinsic
    if (jacobians[5]) {
      Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> jacobian(jacobians[5]);
      jacobian.block(0, 0, 2, 8) = H_dz_dzeta;
    }
  }
  return true;
}