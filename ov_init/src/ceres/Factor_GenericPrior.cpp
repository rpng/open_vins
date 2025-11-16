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

#include "Factor_GenericPrior.h"

#include "utils/quat_ops.h"

using namespace ov_init;

Factor_GenericPrior::Factor_GenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_,
                                         const Eigen::MatrixXd &prior_Info, const Eigen::MatrixXd &prior_grad)
    : x_lin(x_lin_), x_type(x_type_) {

  // First assert that our state and variables are of the correct size
  int state_size = 0;
  int state_error_size = 0;
  for (auto const &str : x_type_) {
    if (str == "quat") {
      state_size += 4;
      state_error_size += 3;
    } else if (str == "quat_yaw") {
      state_size += 4;
      state_error_size += 1;
    } else if (str == "vec1") {
      state_size += 1;
      state_error_size += 1;
    } else if (str == "vec3") {
      state_size += 3;
      state_error_size += 3;
    } else if (str == "vec8") {
      state_size += 8;
      state_error_size += 8;
    } else {
      std::cerr << "type - " << str << " not implemented in prior" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
  assert(x_lin.rows() == state_size);
  assert(x_lin.cols() == 1);
  assert(prior_Info.rows() == state_error_size);
  assert(prior_Info.cols() == state_error_size);
  assert(prior_grad.rows() == state_error_size);
  assert(prior_grad.cols() == 1);

  // Now lets base-compute the square-root information and constant term b
  // Comes from the form: cost = A * (x - x_lin) + b
  Eigen::LLT<Eigen::MatrixXd> lltOfI(prior_Info);
  sqrtI = lltOfI.matrixL().transpose();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(prior_Info.rows(), prior_Info.rows());
  b = sqrtI.triangularView<Eigen::Upper>().solve(I) * prior_grad;

  // Check that we have a valid matrix that we can get the information of
  if (std::isnan(prior_Info.norm()) || std::isnan(sqrtI.norm()) || std::isnan(b.norm())) {
    std::cerr << "prior_Info - " << std::endl << prior_Info << std::endl << std::endl;
    std::cerr << "prior_Info_inv - " << std::endl << prior_Info.inverse() << std::endl << std::endl;
    std::cerr << "b - " << std::endl << b << std::endl << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Set the number of measurements, and the block sized
  set_num_residuals(state_error_size);
  for (auto const &str : x_type_) {
    if (str == "quat")
      mutable_parameter_block_sizes()->push_back(4);
    if (str == "quat_yaw")
      mutable_parameter_block_sizes()->push_back(4);
    if (str == "vec1")
      mutable_parameter_block_sizes()->push_back(1);
    if (str == "vec3")
      mutable_parameter_block_sizes()->push_back(3);
    if (str == "vec8")
      mutable_parameter_block_sizes()->push_back(8);
  }
}

bool Factor_GenericPrior::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

  // Location in our state and output residual
  int local_it = 0;
  int global_it = 0;
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(num_residuals(), 1);

  // Loop through each state and calculate its residual and Jacobian
  for (size_t i = 0; i < x_type.size(); i++) {
    if (x_type[i] == "quat") {
      Eigen::Vector4d q_i = Eigen::Map<const Eigen::Vector4d>(parameters[i]);
      Eigen::Matrix3d R_i = ov_core::quat_2_Rot(q_i);
      Eigen::Matrix3d R_lin = ov_core::quat_2_Rot(x_lin.block(global_it, 0, 4, 1));
      Eigen::Vector3d theta_err = ov_core::log_so3(R_i.transpose() * R_lin);
      res.block(local_it, 0, 3, 1) = -theta_err;
      if (jacobians && jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 4);
        jacobian.setZero();
        Eigen::Matrix3d Jr_inv = ov_core::Jr_so3(theta_err).inverse();
        Eigen::Matrix3d H_theta = -Jr_inv * R_lin.transpose();
        jacobian.block(0, 0, num_residuals(), 3) = sqrtI.block(0, local_it, num_residuals(), 3) * H_theta;
      }
      global_it += 4;
      local_it += 3;
    } else if (x_type[i] == "quat_yaw") {
      Eigen::Vector3d ez = Eigen::Vector3d(0.0, 0.0, 1.0);
      Eigen::Vector4d q_i = Eigen::Map<const Eigen::Vector4d>(parameters[i]);
      Eigen::Matrix3d R_i = ov_core::quat_2_Rot(q_i);
      Eigen::Matrix3d R_lin = ov_core::quat_2_Rot(x_lin.block(global_it, 0, 4, 1));
      Eigen::Vector3d theta_err = ov_core::log_so3(R_i.transpose() * R_lin);
      res(local_it, 0) = -(ez.transpose() * theta_err)(0, 0);
      if (jacobians && jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 4);
        jacobian.setZero();
        Eigen::Matrix3d Jr_inv = ov_core::Jr_so3(theta_err).inverse();
        Eigen::Matrix<double, 1, 3> H_theta = -ez.transpose() * (Jr_inv * R_lin.transpose());
        jacobian.block(0, 0, num_residuals(), 3) = sqrtI.block(0, local_it, num_residuals(), 1) * H_theta;
      }
      global_it += 4;
      local_it += 1;
    } else if (x_type[i] == "vec1") {
      double x = parameters[i][0];
      res(local_it, 0) = x - x_lin(global_it, 0);
      if (jacobians && jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J_vi(jacobians[i], num_residuals(), 1);
        J_vi.block(0, 0, num_residuals(), 1) = sqrtI.block(0, local_it, num_residuals(), 1);
      }
      global_it += 1;
      local_it += 1;
    } else if (x_type[i] == "vec3") {
      Eigen::Matrix<double, 3, 1> p_i = Eigen::Map<const Eigen::Matrix<double, 3, 1>>(parameters[i]);
      res.block(local_it, 0, 3, 1) = p_i - x_lin.block(global_it, 0, 3, 1);
      if (jacobians && jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 3);
        jacobian.block(0, 0, num_residuals(), 3) = sqrtI.block(0, local_it, num_residuals(), 3);
      }
      global_it += 3;
      local_it += 3;
    } else if (x_type[i] == "vec8") {
      Eigen::Matrix<double, 8, 1> p_i = Eigen::Map<const Eigen::Matrix<double, 8, 1>>(parameters[i]);
      res.block(local_it, 0, 8, 1) = p_i - x_lin.block(global_it, 0, 8, 1);
      if (jacobians && jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], num_residuals(), 8);
        jacobian.block(0, 0, num_residuals(), 8) = sqrtI.block(0, local_it, num_residuals(), 8);
      }
      global_it += 8;
      local_it += 8;
    } else {
      std::cerr << "type - " << x_type[i] << " not implemented in prior" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  // Now that we have done x - x_lin we need to multiply by sqrtI and add b to get full cost
  // Jacobians will already have sqrtI applied to them...
  res = sqrtI * res;
  res += b;

  // Store the residuals into ceres
  for (int i = 0; i < res.rows(); i++) {
    residuals[i] = res(i, 0);
  }
  return true;
}