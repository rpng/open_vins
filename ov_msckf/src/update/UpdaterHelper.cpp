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


#include "UpdaterHelper.h"

using namespace ov_core;
using namespace ov_msckf;

void UpdaterHelper::get_feature_jacobian_representation(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                                        std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<Type>> &x_order) {

  // Global XYZ representation
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D) {
    H_f.resize(3, 3);
    H_f.setIdentity();
    return;
  }

  // Global inverse depth representation
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH) {

    // Get the feature linearization point
    Eigen::Matrix<double, 3, 1> p_FinG = (state->_options.do_fej) ? feature.p_FinG_fej : feature.p_FinG;

    // Get inverse depth representation (should match what is in Landmark.cpp)
    double g_rho = 1 / p_FinG.norm();
    double g_phi = std::acos(g_rho * p_FinG(2));
    // double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
    double g_theta = std::atan2(p_FinG(1), p_FinG(0));
    Eigen::Matrix<double, 3, 1> p_invFinG;
    p_invFinG(0) = g_theta;
    p_invFinG(1) = g_phi;
    p_invFinG(2) = g_rho;

    // Get inverse depth bearings
    double sin_th = std::sin(p_invFinG(0, 0));
    double cos_th = std::cos(p_invFinG(0, 0));
    double sin_phi = std::sin(p_invFinG(1, 0));
    double cos_phi = std::cos(p_invFinG(1, 0));
    double rho = p_invFinG(2, 0);

    // Construct the Jacobian
    H_f.resize(3, 3);
    H_f << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi,
        (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi, -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0,
        -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    return;
  }

  //======================================================================
  //======================================================================
  //======================================================================

  // Assert that we have an anchor pose for this feature
  assert(feature.anchor_cam_id != -1);

  // Anchor pose orientation and position, and camera calibration for our anchor camera
  Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->Rot();
  Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->pos();
  Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot();
  Eigen::Vector3d p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos();
  Eigen::Vector3d p_FinA = feature.p_FinA;

  // If I am doing FEJ, I should FEJ the anchor states (should we fej calibration???)
  // Also get the FEJ position of the feature if we are
  if (state->_options.do_fej) {
    // "Best" feature in the global frame
    Eigen::Vector3d p_FinG_best = R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) + p_IinG;
    // Transform the best into our anchor frame using FEJ
    R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot_fej();
    p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos_fej();
    p_FinA = (R_GtoI.transpose() * R_ItoC.transpose()).transpose() * (p_FinG_best - p_IinG) + p_IinC;
  }
  Eigen::Matrix3d R_CtoG = R_GtoI.transpose() * R_ItoC.transpose();

  // Jacobian for our anchor pose
  Eigen::Matrix<double, 3, 6> H_anc;
  H_anc.block(0, 0, 3, 3).noalias() = -R_GtoI.transpose() * skew_x(R_ItoC.transpose() * (p_FinA - p_IinC));
  H_anc.block(0, 3, 3, 3).setIdentity();

  // Add anchor Jacobians to our return vector
  x_order.push_back(state->_clones_IMU.at(feature.anchor_clone_timestamp));
  H_x.push_back(H_anc);

  // Get calibration Jacobians (for anchor clone)
  if (state->_options.do_calib_camera_pose) {
    Eigen::Matrix<double, 3, 6> H_calib;
    H_calib.block(0, 0, 3, 3).noalias() = -R_CtoG * skew_x(p_FinA - p_IinC);
    H_calib.block(0, 3, 3, 3) = -R_CtoG;
    x_order.push_back(state->_calib_IMUtoCAM.at(feature.anchor_cam_id));
    H_x.push_back(H_calib);
  }

  // If we are doing anchored XYZ feature
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_3D) {
    H_f = R_CtoG;
    return;
  }

  // If we are doing full inverse depth
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH) {

    // Get inverse depth representation (should match what is in Landmark.cpp)
    double a_rho = 1 / p_FinA.norm();
    double a_phi = std::acos(a_rho * p_FinA(2));
    double a_theta = std::atan2(p_FinA(1), p_FinA(0));
    Eigen::Matrix<double, 3, 1> p_invFinA;
    p_invFinA(0) = a_theta;
    p_invFinA(1) = a_phi;
    p_invFinA(2) = a_rho;

    // Using anchored inverse depth
    double sin_th = std::sin(p_invFinA(0, 0));
    double cos_th = std::cos(p_invFinA(0, 0));
    double sin_phi = std::sin(p_invFinA(1, 0));
    double cos_phi = std::cos(p_invFinA(1, 0));
    double rho = p_invFinA(2, 0);
    // assert(p_invFinA(2,0)>=0.0);

    // Jacobian of anchored 3D position wrt inverse depth parameters
    Eigen::Matrix<double, 3, 3> d_pfinA_dpinv;
    d_pfinA_dpinv << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi,
        (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi, -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0,
        -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  // If we are doing the MSCKF version of inverse depth
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH) {

    // Get inverse depth representation (should match what is in Landmark.cpp)
    Eigen::Matrix<double, 3, 1> p_invFinA_MSCKF;
    p_invFinA_MSCKF(0) = p_FinA(0) / p_FinA(2);
    p_invFinA_MSCKF(1) = p_FinA(1) / p_FinA(2);
    p_invFinA_MSCKF(2) = 1 / p_FinA(2);

    // Using the MSCKF version of inverse depth
    double alpha = p_invFinA_MSCKF(0, 0);
    double beta = p_invFinA_MSCKF(1, 0);
    double rho = p_invFinA_MSCKF(2, 0);

    // Jacobian of anchored 3D position wrt inverse depth parameters
    Eigen::Matrix<double, 3, 3> d_pfinA_dpinv;
    d_pfinA_dpinv << (1.0 / rho), 0.0, -(1.0 / (rho * rho)) * alpha, 0.0, (1.0 / rho), -(1.0 / (rho * rho)) * beta, 0.0, 0.0,
        -(1.0 / (rho * rho));
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  /// CASE: Estimate single depth of the feature using the initial bearing
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

    // Get inverse depth representation (should match what is in Landmark.cpp)
    double rho = 1.0 / p_FinA(2);
    Eigen::Vector3d bearing = rho * p_FinA;

    // Jacobian of anchored 3D position wrt inverse depth parameters
    Eigen::Vector3d d_pfinA_drho;
    d_pfinA_drho << -(1.0 / (rho * rho)) * bearing;
    H_f = R_CtoG * d_pfinA_drho;
    return;
  }

  // Failure, invalid representation that is not programmed
  assert(false);
}

void UpdaterHelper::get_feature_jacobian_intrinsics(std::shared_ptr<State> state, const Eigen::Vector2d &uv_norm, bool isfisheye,
                                                    Eigen::Matrix<double, 8, 1> cam_d, Eigen::Matrix<double, 2, 2> &dz_dzn,
                                                    Eigen::Matrix<double, 2, 8> &dz_dzeta) {

  // Calculate distortion uv and jacobian
  if (isfisheye) {

    // Calculate distorted coordinates for fisheye
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double theta = std::atan(r);
    double theta_d = theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
                     cam_d(7) * std::pow(theta, 9);

    // Handle when r is small (meaning our xy is near the camera center)
    double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    // Jacobian of distorted pixel to "normalized" pixel
    Eigen::Matrix<double, 2, 2> duv_dxy = Eigen::Matrix<double, 2, 2>::Zero();
    duv_dxy << cam_d(0), 0, 0, cam_d(1);

    // Jacobian of "normalized" pixel to normalized pixel
    Eigen::Matrix<double, 2, 2> dxy_dxyn = Eigen::Matrix<double, 2, 2>::Zero();
    dxy_dxyn << theta_d * inv_r, 0, 0, theta_d * inv_r;

    // Jacobian of "normalized" pixel to r
    Eigen::Matrix<double, 2, 1> dxy_dr = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dr << -uv_norm(0) * theta_d * inv_r * inv_r, -uv_norm(1) * theta_d * inv_r * inv_r;

    // Jacobian of r pixel to normalized xy
    Eigen::Matrix<double, 1, 2> dr_dxyn = Eigen::Matrix<double, 1, 2>::Zero();
    dr_dxyn << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of "normalized" pixel to theta_d
    Eigen::Matrix<double, 2, 1> dxy_dthd = Eigen::Matrix<double, 2, 1>::Zero();
    dxy_dthd << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of theta_d to theta
    double dthd_dth = 1 + 3 * cam_d(4) * std::pow(theta, 2) + 5 * cam_d(5) * std::pow(theta, 4) + 7 * cam_d(6) * std::pow(theta, 6) +
                      9 * cam_d(7) * std::pow(theta, 8);

    // Jacobian of theta to r
    double dth_dr = 1 / (r * r + 1);

    // Total Jacobian wrt normalized pixel coordinates
    dz_dzn = duv_dxy * (dxy_dxyn + (dxy_dr + dxy_dthd * dthd_dth * dth_dr) * dr_dxyn);

    // Compute the Jacobian in respect to the intrinsics if we are calibrating
    if (state->_options.do_calib_camera_intrinsics) {

      // Calculate distorted coordinates for fisheye
      double x1 = uv_norm(0) * cdist;
      double y1 = uv_norm(1) * cdist;

      // Jacobian
      dz_dzeta(0, 0) = x1;
      dz_dzeta(0, 2) = 1;
      dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 3);
      dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 5);
      dz_dzeta(0, 6) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 7);
      dz_dzeta(0, 7) = cam_d(0) * uv_norm(0) * inv_r * std::pow(theta, 9);
      dz_dzeta(1, 1) = y1;
      dz_dzeta(1, 3) = 1;
      dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 3);
      dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 5);
      dz_dzeta(1, 6) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 7);
      dz_dzeta(1, 7) = cam_d(1) * uv_norm(1) * inv_r * std::pow(theta, 9);
    }

  } else {

    // Calculate distorted coordinates for radial
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double r_2 = r * r;
    double r_4 = r_2 * r_2;

    // Jacobian of distorted pixel to normalized pixel
    double x = uv_norm(0);
    double y = uv_norm(1);
    double x_2 = uv_norm(0) * uv_norm(0);
    double y_2 = uv_norm(1) * uv_norm(1);
    double x_y = uv_norm(0) * uv_norm(1);
    dz_dzn(0, 0) = cam_d(0) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * x_2 + 4 * cam_d(5) * x_2 * r) + 2 * cam_d(6) * y +
                               (2 * cam_d(7) * x + 4 * cam_d(7) * x));
    dz_dzn(0, 1) = cam_d(0) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
    dz_dzn(1, 0) = cam_d(1) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
    dz_dzn(1, 1) = cam_d(1) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * y_2 + 4 * cam_d(5) * y_2 * r) + 2 * cam_d(7) * x +
                               (2 * cam_d(6) * y + 4 * cam_d(6) * y));

    // Compute the Jacobian in respect to the intrinsics if we are calibrating
    if (state->_options.do_calib_camera_intrinsics) {

      // Calculate distorted coordinates for radtan
      double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
                  cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
      double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
                  2 * cam_d(7) * uv_norm(0) * uv_norm(1);

      // Jacobian
      dz_dzeta(0, 0) = x1;
      dz_dzeta(0, 2) = 1;
      dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * r_2;
      dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * r_4;
      dz_dzeta(0, 6) = 2 * cam_d(0) * uv_norm(0) * uv_norm(1);
      dz_dzeta(0, 7) = cam_d(0) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
      dz_dzeta(1, 1) = y1;
      dz_dzeta(1, 3) = 1;
      dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * r_2;
      dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * r_4;
      dz_dzeta(1, 6) = cam_d(1) * (r_2 + 2 * uv_norm(1) * uv_norm(1));
      dz_dzeta(1, 7) = 2 * cam_d(1) * uv_norm(0) * uv_norm(1);
    }
  }
}

void UpdaterHelper::get_feature_jacobian_full(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                              Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<Type>> &x_order) {

  // Total number of measurements for this feature
  int total_meas = 0;
  for (auto const &pair : feature.timestamps) {
    total_meas += (int)pair.second.size();
  }

  // Compute the size of the states involved with this feature
  int total_hx = 0;
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  for (auto const &pair : feature.timestamps) {

    // Our extrinsics and intrinsics
    std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
    std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);

    // If doing calibration extrinsics
    if (state->_options.do_calib_camera_pose) {
      map_hx.insert({calibration, total_hx});
      x_order.push_back(calibration);
      total_hx += calibration->size();
    }

    // If doing calibration intrinsics
    if (state->_options.do_calib_camera_intrinsics) {
      map_hx.insert({distortion, total_hx});
      x_order.push_back(distortion);
      total_hx += distortion->size();
    }

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

      // Add this clone if it is not added already
      std::shared_ptr<PoseJPL> clone_Ci = state->_clones_IMU.at(feature.timestamps[pair.first].at(m));
      if (map_hx.find(clone_Ci) == map_hx.end()) {
        map_hx.insert({clone_Ci, total_hx});
        x_order.push_back(clone_Ci);
        total_hx += clone_Ci->size();
      }
    }
  }

  // If we are using an anchored representation, make sure that the anchor is also added
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {

    // Assert we have a clone
    assert(feature.anchor_cam_id != -1);

    // Add this anchor if it is not added already
    std::shared_ptr<PoseJPL> clone_Ai = state->_clones_IMU.at(feature.anchor_clone_timestamp);
    if (map_hx.find(clone_Ai) == map_hx.end()) {
      map_hx.insert({clone_Ai, total_hx});
      x_order.push_back(clone_Ai);
      total_hx += clone_Ai->size();
    }

    // Also add its calibration if we are doing calibration
    if (state->_options.do_calib_camera_pose) {
      // Add this anchor if it is not added already
      std::shared_ptr<PoseJPL> clone_calib = state->_calib_IMUtoCAM.at(feature.anchor_cam_id);
      if (map_hx.find(clone_calib) == map_hx.end()) {
        map_hx.insert({clone_calib, total_hx});
        x_order.push_back(clone_calib);
        total_hx += clone_calib->size();
      }
    }
  }

  //=========================================================================
  //=========================================================================

  // Calculate the position of this feature in the global frame
  // If anchored, then we need to calculate the position of the feature in the global
  Eigen::Vector3d p_FinG = feature.p_FinG;
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {
    // Assert that we have an anchor pose for this feature
    assert(feature.anchor_cam_id != -1);
    // Get calibration for our anchor camera
    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->pos();
    // Anchor pose orientation and position
    Eigen::Matrix<double, 3, 3> R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos();
    // Feature in the global frame
    p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) + p_IinG;
  }

  // Calculate the position of this feature in the global frame FEJ
  // If anchored, then we can use the "best" p_FinG since the value of p_FinA does not matter
  Eigen::Vector3d p_FinG_fej = feature.p_FinG_fej;
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {
    p_FinG_fej = p_FinG;
  }

  //=========================================================================
  //=========================================================================

  // Allocate our residual and Jacobians
  int c = 0;
  int jacobsize = (feature.feat_representation != LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 3 : 1;
  res = Eigen::VectorXd::Zero(2 * total_meas);
  H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobsize);
  H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

  // Derivative of p_FinG in respect to feature representation.
  // This only needs to be computed once and thus we pull it out of the loop
  Eigen::MatrixXd dpfg_dlambda;
  std::vector<Eigen::MatrixXd> dpfg_dx;
  std::vector<std::shared_ptr<Type>> dpfg_dx_order;
  UpdaterHelper::get_feature_jacobian_representation(state, feature, dpfg_dlambda, dpfg_dx, dpfg_dx_order);

  // Assert that all the ones in our order are already in our local jacobian mapping
  for (auto &type : dpfg_dx_order) {
    assert(map_hx.find(type) != map_hx.end());
  }

  // Loop through each camera for this feature
  for (auto const &pair : feature.timestamps) {

    // Our calibration between the IMU and CAMi frames
    std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);
    std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
    Eigen::Matrix<double, 3, 3> R_ItoC = calibration->Rot();
    Eigen::Matrix<double, 3, 1> p_IinC = calibration->pos();
    Eigen::Matrix<double, 8, 1> cam_d = distortion->value();

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

      //=========================================================================
      //=========================================================================

      // Get current IMU clone state
      std::shared_ptr<PoseJPL> clone_Ii = state->_clones_IMU.at(feature.timestamps[pair.first].at(m));
      Eigen::Matrix<double, 3, 3> R_GtoIi = clone_Ii->Rot();
      Eigen::Matrix<double, 3, 1> p_IiinG = clone_Ii->pos();

      // Get current feature in the IMU
      Eigen::Matrix<double, 3, 1> p_FinIi = R_GtoIi * (p_FinG - p_IiinG);

      // Project the current feature into the current frame of reference
      Eigen::Matrix<double, 3, 1> p_FinCi = R_ItoC * p_FinIi + p_IinC;
      Eigen::Matrix<double, 2, 1> uv_norm;
      uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

      // Distort the normalized coordinates (false=radtan, true=fisheye)
      Eigen::Matrix<double, 2, 1> uv_dist;

      // Calculate distortion uv and jacobian
      if (state->_cam_intrinsics_model.at(pair.first)) {

        // Calculate distorted coordinates for fisheye
        double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
        double theta = std::atan(r);
        double theta_d = theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
                         cam_d(7) * std::pow(theta, 9);

        // Handle when r is small (meaning our xy is near the camera center)
        double inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
        double cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

        // Calculate distorted coordinates for fisheye
        double x1 = uv_norm(0) * cdist;
        double y1 = uv_norm(1) * cdist;
        uv_dist(0) = cam_d(0) * x1 + cam_d(2);
        uv_dist(1) = cam_d(1) * y1 + cam_d(3);

      } else {

        // Calculate distorted coordinates for radial
        double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
        double r_2 = r * r;
        double r_4 = r_2 * r_2;
        double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
                    cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
        double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
                    2 * cam_d(7) * uv_norm(0) * uv_norm(1);
        uv_dist(0) = cam_d(0) * x1 + cam_d(2);
        uv_dist(1) = cam_d(1) * y1 + cam_d(3);
      }

      // Our residual
      Eigen::Matrix<double, 2, 1> uv_m;
      uv_m << (double)feature.uvs[pair.first].at(m)(0), (double)feature.uvs[pair.first].at(m)(1);
      res.block(2 * c, 0, 2, 1) = uv_m - uv_dist;

      //=========================================================================
      //=========================================================================

      // If we are doing first estimate Jacobians, then overwrite with the first estimates
      if (state->_options.do_fej) {
        R_GtoIi = clone_Ii->Rot_fej();
        p_IiinG = clone_Ii->pos_fej();
        // R_ItoC = calibration->Rot_fej();
        // p_IinC = calibration->pos_fej();
        p_FinIi = R_GtoIi * (p_FinG_fej - p_IiinG);
        p_FinCi = R_ItoC * p_FinIi + p_IinC;
        // uv_norm << p_FinCi(0)/p_FinCi(2),p_FinCi(1)/p_FinCi(2);
        // cam_d = state->get_intrinsics_CAM(pair.first)->fej();
      }

      // Compute Jacobians in respect to normalized image coordinates and possibly the camera intrinsics
      Eigen::Matrix<double, 2, 2> dz_dzn = Eigen::Matrix<double, 2, 2>::Zero();
      Eigen::Matrix<double, 2, 8> dz_dzeta = Eigen::Matrix<double, 2, 8>::Zero();
      UpdaterHelper::get_feature_jacobian_intrinsics(state, uv_norm, state->_cam_intrinsics_model.at(pair.first), cam_d, dz_dzn, dz_dzeta);

      // Normalized coordinates in respect to projection function
      Eigen::Matrix<double, 2, 3> dzn_dpfc = Eigen::Matrix<double, 2, 3>::Zero();
      dzn_dpfc << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0, 1 / p_FinCi(2), -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

      // Derivative of p_FinCi in respect to p_FinIi
      Eigen::Matrix<double, 3, 3> dpfc_dpfg = R_ItoC * R_GtoIi;

      // Derivative of p_FinCi in respect to camera clone state
      Eigen::Matrix<double, 3, 6> dpfc_dclone = Eigen::Matrix<double, 3, 6>::Zero();
      dpfc_dclone.block(0, 0, 3, 3).noalias() = R_ItoC * skew_x(p_FinIi);
      dpfc_dclone.block(0, 3, 3, 3) = -dpfc_dpfg;

      //=========================================================================
      //=========================================================================

      // Precompute some matrices
      Eigen::Matrix<double, 2, 3> dz_dpfc = dz_dzn * dzn_dpfc;
      Eigen::Matrix<double, 2, 3> dz_dpfg = dz_dpfc * dpfc_dpfg;

      // CHAINRULE: get the total feature Jacobian
      H_f.block(2 * c, 0, 2, H_f.cols()).noalias() = dz_dpfg * dpfg_dlambda;

      // CHAINRULE: get state clone Jacobian
      H_x.block(2 * c, map_hx[clone_Ii], 2, clone_Ii->size()).noalias() = dz_dpfc * dpfc_dclone;

      // CHAINRULE: loop through all extra states and add their
      // NOTE: we add the Jacobian here as we might be in the anchoring pose for this measurement
      for (size_t i = 0; i < dpfg_dx_order.size(); i++) {
        H_x.block(2 * c, map_hx[dpfg_dx_order.at(i)], 2, dpfg_dx_order.at(i)->size()).noalias() += dz_dpfg * dpfg_dx.at(i);
      }

      //=========================================================================
      //=========================================================================

      // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)
      if (state->_options.do_calib_camera_pose) {

        // Calculate the Jacobian
        Eigen::Matrix<double, 3, 6> dpfc_dcalib = Eigen::Matrix<double, 3, 6>::Zero();
        dpfc_dcalib.block(0, 0, 3, 3) = skew_x(p_FinCi - p_IinC);
        dpfc_dcalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

        // Chainrule it and add it to the big jacobian
        H_x.block(2 * c, map_hx[calibration], 2, calibration->size()).noalias() += dz_dpfc * dpfc_dcalib;
      }

      // Derivative of measurement in respect to distortion parameters
      if (state->_options.do_calib_camera_intrinsics) {
        H_x.block(2 * c, map_hx[distortion], 2, distortion->size()) = dz_dzeta;
      }

      // Move the Jacobian and residual index forward
      c++;
    }
  }
}

void UpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Apply the left nullspace of H_f to all variables
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = (int)H_f.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
  // NOTE: need to eigen3 eval here since this experiences aliasing!
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // Sanity check
  assert(H_x.rows() == res.rows());
}

void UpdaterHelper::measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Return if H_x is a fat matrix (there is no need to compress in this case)
  if (H_x.rows() <= H_x.cols())
    return;

  // Do measurement compression through givens rotations
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_x.cols(); n++) {
    for (int m = (int)H_x.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_x(m - 1, n), H_x(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // If H is a fat matrix, then use the rows
  // Else it should be same size as our state
  int r = std::min(H_x.rows(), H_x.cols());

  // Construct the smaller jacobian and residual after measurement compression
  assert(r <= H_x.rows());
  H_x.conservativeResize(r, H_x.cols());
  res.conservativeResize(r, res.cols());
}
