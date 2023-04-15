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

#include "Factor_ImuCPIv1.h"

#include "utils/quat_ops.h"

using namespace ov_init;

Factor_ImuCPIv1::Factor_ImuCPIv1(double deltatime, Eigen::Vector3d &grav, Eigen::Vector3d &alpha, Eigen::Vector3d &beta,
                                 Eigen::Vector4d &q_KtoK1, Eigen::Vector3d &ba_lin, Eigen::Vector3d &bg_lin, Eigen::Matrix3d &J_q,
                                 Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha, Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha,
                                 Eigen::Matrix<double, 15, 15> &covariance) {
  // Save measurements
  this->alpha = alpha;
  this->beta = beta;
  this->q_breve = q_KtoK1;
  this->dt = deltatime;
  this->grav_save = grav;

  // Linearization points
  this->b_a_lin_save = ba_lin;
  this->b_w_lin_save = bg_lin;

  // Save bias jacobians
  this->J_q = J_q;
  this->J_a = J_alpha;
  this->J_b = J_beta;
  this->H_a = H_alpha;
  this->H_b = H_beta;

  // Check that we have a valid covariance matrix that we can get the information of
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(covariance.rows(), covariance.rows());
  Eigen::MatrixXd information = covariance.llt().solve(I);
  if (std::isnan(information.norm())) {
    std::cerr << "P - " << std::endl << covariance << std::endl << std::endl;
    std::cerr << "Pinv - " << std::endl << covariance.inverse() << std::endl << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Get square-root of our information matrix
  Eigen::LLT<Eigen::MatrixXd> lltOfI(information);
  sqrtI_save = lltOfI.matrixL().transpose();

  // Set the number of measurements, and the block sized
  set_num_residuals(15);
  mutable_parameter_block_sizes()->push_back(4); // q_GtoI1
  mutable_parameter_block_sizes()->push_back(3); // bg_1
  mutable_parameter_block_sizes()->push_back(3); // v_I1inG
  mutable_parameter_block_sizes()->push_back(3); // ba_1
  mutable_parameter_block_sizes()->push_back(3); // p_I1inG
  mutable_parameter_block_sizes()->push_back(4); // q_GtoI2
  mutable_parameter_block_sizes()->push_back(3); // bg_2
  mutable_parameter_block_sizes()->push_back(3); // v_I2inG
  mutable_parameter_block_sizes()->push_back(3); // ba_2
  mutable_parameter_block_sizes()->push_back(3); // p_I2inG
}

bool Factor_ImuCPIv1::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

  // Get the local variables (these would be different if we relinearized)
  Eigen::Vector3d gravity = grav_save;
  Eigen::Matrix<double, 15, 15> sqrtI = sqrtI_save;
  Eigen::Vector3d b_w_lin = b_w_lin_save;
  Eigen::Vector3d b_a_lin = b_a_lin_save;

  // Get the state estimates from the ceres parameters.
  // Each clone is stored in the canonical VINS format: q, bw, v, ba, p
  Eigen::Vector4d q_1 = Eigen::Map<const Eigen::Vector4d>(parameters[0]);
  Eigen::Matrix3d R_1 = ov_core::quat_2_Rot(q_1);
  Eigen::Vector4d q_2 = Eigen::Map<const Eigen::Vector4d>(parameters[5]);

  // Get bias_w
  Eigen::Vector3d b_w1 = Eigen::Map<const Eigen::Vector3d>(parameters[1]);
  Eigen::Vector3d b_w2 = Eigen::Map<const Eigen::Vector3d>(parameters[6]);

  // Get bias_a
  Eigen::Vector3d b_a1 = Eigen::Map<const Eigen::Vector3d>(parameters[3]);
  Eigen::Vector3d b_a2 = Eigen::Map<const Eigen::Vector3d>(parameters[8]);

  // Get velocity
  Eigen::Vector3d v_1 = Eigen::Map<const Eigen::Vector3d>(parameters[2]);
  Eigen::Vector3d v_2 = Eigen::Map<const Eigen::Vector3d>(parameters[7]);

  // Get positions
  Eigen::Vector3d p_1 = Eigen::Map<const Eigen::Vector3d>(parameters[4]);
  Eigen::Vector3d p_2 = Eigen::Map<const Eigen::Vector3d>(parameters[9]);

  // Get the change in clone 1's biases from the linearization points
  Eigen::Vector3d dbw = b_w1 - b_w_lin;
  Eigen::Vector3d dba = b_a1 - b_a_lin;

  // Quaternion associated with the bias w correction
  // Eigen::Vector4d q_b= rot_2_quat(Exp(-J_q*dbw));
  Eigen::Vector4d q_b;
  q_b.block(0, 0, 3, 1) = 0.5 * J_q * dbw;
  q_b(3, 0) = 1.0;
  q_b = q_b / q_b.norm();

  // Relative orientation from state estimates
  Eigen::Vector4d q_1_to_2 = ov_core::quat_multiply(q_2, ov_core::Inv(q_1));

  // Intermediate quaternions for error/jacobian computation
  Eigen::Vector4d q_res_minus = ov_core::quat_multiply(q_1_to_2, ov_core::Inv(q_breve));
  Eigen::Vector4d q_res_plus = ov_core::quat_multiply(q_res_minus, q_b);

  //================================================================================
  //================================================================================
  //================================================================================

  // Computer residual
  Eigen::Matrix<double, 15, 1> res;
  res.block(0, 0, 3, 1) = 2 * q_res_plus.block(0, 0, 3, 1);
  res.block(3, 0, 3, 1) = b_w2 - b_w1;
  res.block(6, 0, 3, 1) = R_1 * (v_2 - v_1 + gravity * dt) - J_b * dbw - H_b * dba - beta;
  res.block(9, 0, 3, 1) = b_a2 - b_a1;
  res.block(12, 0, 3, 1) = R_1 * (p_2 - p_1 - v_1 * dt + .5 * gravity * std::pow(dt, 2)) - J_a * dbw - H_a * dba - alpha;
  res = sqrtI * res;

  // Store residuals
  for (int i = 0; i < res.rows(); i++) {
    residuals[i] = res(i, 0);
  }

  //================================================================================
  //================================================================================
  //================================================================================

  // Compute jacobians if asked
  if (jacobians) {

    // Stores the total preintegrated jacobian into one spot
    Eigen::Matrix<double, 15, 30> Jacobian = Eigen::Matrix<double, 15, 30>::Zero();

    // Quick identity
    Eigen::Matrix<double, 3, 3> eye = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 4, 1> q_meas_plus = ov_core::quat_multiply(ov_core::Inv(q_breve), q_b);

    // Dtheta wrt theta 1
    Jacobian.block(0, 0, 3, 3) = -((q_1_to_2(3, 0) * eye - ov_core::skew_x(q_1_to_2.block(0, 0, 3, 1))) *
                                       (q_meas_plus(3, 0) * eye + ov_core::skew_x(q_meas_plus.block(0, 0, 3, 1))) -
                                   q_1_to_2.block(0, 0, 3, 1) * q_meas_plus.block(0, 0, 3, 1).transpose());

    // Dtheta wrt theta 2
    Jacobian.block(0, 15, 3, 3) = q_res_plus(3, 0) * eye + ov_core::skew_x(q_res_plus.block(0, 0, 3, 1));

    // Dtheta wrt bw 1
    Jacobian.block(0, 3, 3, 3) = (q_res_minus(3, 0) * eye - ov_core::skew_x(q_res_minus.block(0, 0, 3, 1))) * J_q;

    // Dbw wrt bw1 and bw2
    Jacobian.block(3, 3, 3, 3) = -eye;
    Jacobian.block(3, 18, 3, 3) = eye;

    // Dvelocity wrt theta 1
    Jacobian.block(6, 0, 3, 3) = ov_core::skew_x(R_1 * (v_2 - v_1 + gravity * dt));

    // Dvelocity wrt v 1
    Jacobian.block(6, 6, 3, 3) = -R_1;

    // Dvelocity wrt v 2
    Jacobian.block(6, 21, 3, 3) = R_1;

    // Dvelocity wrt bw 1
    Jacobian.block(6, 3, 3, 3) = -J_b;

    // Dvelocity wrt ba 1
    Jacobian.block(6, 9, 3, 3) = -H_b;

    // Dbw wrt ba1 and ba2
    Jacobian.block(9, 9, 3, 3) = -eye;
    Jacobian.block(9, 24, 3, 3) = eye;

    // Dposition wrt theta 1
    Jacobian.block(12, 0, 3, 3) = ov_core::skew_x(R_1 * (p_2 - p_1 - v_1 * dt + .5 * gravity * std::pow(dt, 2)));
    // Dposition wrt v 1
    Jacobian.block(12, 6, 3, 3) = -R_1 * dt;
    // Dposition wrt p 1
    Jacobian.block(12, 12, 3, 3) = -R_1;

    // Dposition wrt p 2
    Jacobian.block(12, 27, 3, 3) = R_1;

    // Dposition wrt bw 1
    Jacobian.block(12, 3, 3, 3) = -J_a;
    // Dposition wrt ba 1
    Jacobian.block(12, 9, 3, 3) = -H_a;

    // Apply sqrt info
    Jacobian = sqrtI * Jacobian;

    // Now store jacobians
    // Th1
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_th1(jacobians[0], 15, 4);
      J_th1.block(0, 0, 15, 3) = Jacobian.block(0, 0, 15, 3);
      J_th1.block(0, 3, 15, 1).setZero();
    }
    // Th2
    if (jacobians[5]) {
      Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> J_th2(jacobians[5], 15, 4);
      J_th2.block(0, 0, 15, 3) = Jacobian.block(0, 15, 15, 3);
      J_th2.block(0, 3, 15, 1).setZero();
    }
    // bw1
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_bw1(jacobians[1], 15, 3);
      J_bw1.block(0, 0, 15, 3) = Jacobian.block(0, 3, 15, 3);
    }
    // bw2
    if (jacobians[6]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_bw2(jacobians[6], 15, 3);
      J_bw2.block(0, 0, 15, 3) = Jacobian.block(0, 18, 15, 3);
    }
    // v1
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_v1(jacobians[2], 15, 3);
      J_v1.block(0, 0, 15, 3) = Jacobian.block(0, 6, 15, 3);
    }
    // v2
    if (jacobians[7]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_v2(jacobians[7], 15, 3);
      J_v2.block(0, 0, 15, 3) = Jacobian.block(0, 21, 15, 3);
    }
    // ba1
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_ba1(jacobians[3], 15, 3);
      J_ba1.block(0, 0, 15, 3) = Jacobian.block(0, 9, 15, 3);
    }
    // ba2
    if (jacobians[8]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_ba2(jacobians[8], 15, 3);
      J_ba2.block(0, 0, 15, 3) = Jacobian.block(0, 24, 15, 3);
    }
    // p1
    if (jacobians[4]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_p1(jacobians[4], 15, 3);
      J_p1.block(0, 0, 15, 3) = Jacobian.block(0, 12, 15, 3);
    }
    // p2
    if (jacobians[9]) {
      Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> J_p2(jacobians[9], 15, 3);
      J_p2.block(0, 0, 15, 3) = Jacobian.block(0, 27, 15, 3);
    }
  }
  return true;
}
