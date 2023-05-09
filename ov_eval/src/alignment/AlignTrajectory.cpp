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

#include "AlignTrajectory.h"

using namespace ov_eval;

void AlignTrajectory::align_trajectory(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                       const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t,
                                       double &s, std::string method, int n_aligned) {

  // Use the correct method
  if (method == "posyaw") {
    s = 1;
    align_posyaw(traj_es, traj_gt, R, t, n_aligned);
  } else if (method == "posyawsingle") {
    s = 1;
    align_posyaw_single(traj_es, traj_gt, R, t);
  } else if (method == "se3") {
    s = 1;
    align_se3(traj_es, traj_gt, R, t, n_aligned);
  } else if (method == "se3single") {
    s = 1;
    align_se3_single(traj_es, traj_gt, R, t);
  } else if (method == "sim3") {
    assert(n_aligned >= 2 || n_aligned == -1);
    align_sim3(traj_es, traj_gt, R, t, s, n_aligned);
  } else if (method == "none") {
    s = 1;
    R.setIdentity();
    t.setZero();
  } else {
    PRINT_ERROR(RED "[ALIGN]: Invalid alignment method!\n" RESET);
    PRINT_ERROR(RED "[ALIGN]: Possible options: posyaw, posyawsingle, se3, se3single, sim3, none\n" RESET);
    std::exit(EXIT_FAILURE);
  }
}

void AlignTrajectory::align_posyaw_single(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                          const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t) {

  // Get first ever poses
  Eigen::Vector4d q_es_0 = traj_es.at(0).block(3, 0, 4, 1);
  Eigen::Vector3d p_es_0 = traj_es.at(0).block(0, 0, 3, 1);

  Eigen::Vector4d q_gt_0 = traj_gt.at(0).block(3, 0, 4, 1);
  Eigen::Vector3d p_gt_0 = traj_gt.at(0).block(0, 0, 3, 1);

  // Get rotations from IMU frame to World (note JPL!)
  Eigen::Matrix3d g_rot = ov_core::quat_2_Rot(q_gt_0).transpose();
  Eigen::Matrix3d est_rot = ov_core::quat_2_Rot(q_es_0).transpose();

  // Data matrix for the Frobenius problem
  Eigen::Matrix3d C_R = est_rot * g_rot.transpose();

  // Recover yaw
  double theta = AlignUtils::get_best_yaw(C_R);

  // Compute rotation
  R = ov_core::rot_z(theta);

  // Compute translation
  t.noalias() = p_gt_0 - R * p_es_0;
}

void AlignTrajectory::align_posyaw(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                   const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t,
                                   int n_aligned) {

  // If we only have one, just use the single alignment
  if (n_aligned == 1) {
    align_posyaw_single(traj_es, traj_gt, R, t);
  } else {

    // Get just position vectors
    assert(!traj_es.empty());
    std::vector<Eigen::Vector3d> pos_est, pos_gt;
    for (size_t i = 0; i < traj_es.size() && i < traj_gt.size(); i++) {
      pos_est.push_back(traj_es.at(i).block(0, 0, 3, 1));
      pos_gt.push_back(traj_gt.at(i).block(0, 0, 3, 1));
    }

    // Align using the method of Umeyama
    double s;
    AlignUtils::align_umeyama(pos_est, pos_gt, R, t, s, true, true);
    assert(s == 1);
  }
}

void AlignTrajectory::align_se3_single(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                       const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t) {

  // Get first ever poses
  Eigen::Vector4d q_es_0 = traj_es.at(0).block(3, 0, 4, 1);
  Eigen::Vector3d p_es_0 = traj_es.at(0).block(0, 0, 3, 1);

  Eigen::Vector4d q_gt_0 = traj_gt.at(0).block(3, 0, 4, 1);
  Eigen::Vector3d p_gt_0 = traj_gt.at(0).block(0, 0, 3, 1);

  // Get rotations from IMU frame to World (note JPL!)
  Eigen::Matrix3d g_rot = ov_core::quat_2_Rot(q_gt_0).transpose();
  Eigen::Matrix3d est_rot = ov_core::quat_2_Rot(q_es_0).transpose();

  R.noalias() = g_rot * est_rot.transpose();
  t.noalias() = p_gt_0 - R * p_es_0;
}

void AlignTrajectory::align_se3(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t,
                                int n_aligned) {

  // If we only have one, just use the single alignment
  if (n_aligned == 1) {
    align_se3_single(traj_es, traj_gt, R, t);
  } else {

    // Get just position vectors
    assert(!traj_es.empty());
    std::vector<Eigen::Vector3d> pos_est, pos_gt;
    for (size_t i = 0; i < traj_es.size() && i < traj_gt.size(); i++) {
      pos_est.push_back(traj_es.at(i).block(0, 0, 3, 1));
      pos_gt.push_back(traj_gt.at(i).block(0, 0, 3, 1));
    }

    // Align using the method of Umeyama
    double s;
    AlignUtils::align_umeyama(pos_est, pos_gt, R, t, s, true, false);
  }
}

void AlignTrajectory::align_sim3(const std::vector<Eigen::Matrix<double, 7, 1>> &traj_es,
                                 const std::vector<Eigen::Matrix<double, 7, 1>> &traj_gt, Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s,
                                 int n_aligned) {

  // Need to have more than two to get
  assert(n_aligned >= 2 || n_aligned == -1);

  // Get just position vectors
  assert(!traj_es.empty());
  std::vector<Eigen::Vector3d> pos_est, pos_gt;
  for (size_t i = 0; i < traj_es.size() && i < traj_gt.size(); i++) {
    pos_est.push_back(traj_es.at(i).block(0, 0, 3, 1));
    pos_gt.push_back(traj_gt.at(i).block(0, 0, 3, 1));
  }

  // Align using the method of Umeyama
  AlignUtils::align_umeyama(pos_est, pos_gt, R, t, s, false, false);
}
