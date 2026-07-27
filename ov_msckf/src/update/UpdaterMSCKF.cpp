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

#include "UpdaterMSCKF.h"

#include <cmath>
#include <unordered_map>

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterMSCKF::UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {

  // Save our raw pixel noise squared
  _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

  // Save our feature initializer
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterMSCKF::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // Get current camera pose
      Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // Append to our map
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }

  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
  auto it1 = feature_vec.begin();
  while (it1 != feature_vec.end()) {

    // Triangulate the feature and remove if it fails
    bool success_tri = true;
    if (initializer_feat->config().triangulate_1d) {
      success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
    } else {
      success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
    }

    // Gauss-newton refine the feature
    bool success_refine = true;
    if (initializer_feat->config().refine_features) {
      success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
    }

    // Remove the feature if not a success
    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
      it1 = feature_vec.erase(it1);
      continue;
    }
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  size_t max_hx_size = state->max_covariance_size();
  for (auto &landmark : state->_features_SLAM) {
    max_hx_size -= landmark.second->size();
  }

  // Large Jacobian and residual of *all* features for this update
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // Nullspace project
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

    // Per-feature noise multiplier nm using triangulated p_FinG.
    // Dynamic features receive nm > 1 → down-weighted in the EKF update, not hard-rejected.
    // Three modular gates, each independently enabled via YAML (default: all disabled):
    //   SoftGate nm:   IMU reprojection residual → continuous nm inflation  (use_imu_residual)
    //   Depth gate:    suppress nm for far features with unreliable stereo   (imu_residual_max_depth)
    //   Tri gate:      suppress nm when triangulation itself is poor          (imu_residual_max_tri_error)
    double nm = 1.0;
    if (_use_imu_residual &&
        !LandmarkRepresentation::is_relative_representation(feat.feat_representation) &&
        feat.p_FinG.norm() > 0.01 &&
        state->_cam_intrinsics_cameras.count(0) && state->_calib_IMUtoCAM.count(0) &&
        feat.timestamps.count(0) && feat.uvs_norm.count(0)) {
      const auto &times0 = feat.timestamps.at(0);
      const auto &norms0 = feat.uvs_norm.at(0);
      if (times0.size() >= 2) {
        cv::Matx33d K0 = state->_cam_intrinsics_cameras.at(0)->get_K();
        double fx = K0(0, 0);
        Eigen::Matrix3d R_ItoC0 = state->_calib_IMUtoCAM.at(0)->Rot();
        Eigen::Vector3d p_C0inI = state->_calib_IMUtoCAM.at(0)->pos();

        // Triangulation quality gate.
        // Computes RMS reprojection error of p_FinG across all window clones.
        // High RMS means the triangulated position is unreliable (low-texture or
        // degenerate geometry) → nm inflation would punish good features, not bad ones.
        // Falls through (tri_gate_ok = true) when imu_residual_max_tri_error = 0 (default).
        bool tri_gate_ok = true;
        if (_imu_residual_max_tri_error > 0.0) {
          double sum_r2_tri = 0.0;
          int n_tri = 0;
          for (size_t k = 0; k < times0.size(); k++) {
            double t_k = times0[k];
            if (!state->_clones_IMU.count(t_k))
              continue;
            Eigen::Matrix3d R_GtoI_k  = state->_clones_IMU.at(t_k)->Rot();
            Eigen::Vector3d p_IinG_k  = state->_clones_IMU.at(t_k)->pos();
            Eigen::Matrix3d R_GtoC0_k = R_ItoC0 * R_GtoI_k;
            Eigen::Vector3d p_C0inG_k = p_IinG_k - R_GtoC0_k.transpose() * p_C0inI;
            Eigen::Vector3d X_k       = R_GtoC0_k * (feat.p_FinG - p_C0inG_k);
            if (X_k[2] <= 0.1)
              continue;
            double n_px = X_k[0] / X_k[2];
            double n_py = X_k[1] / X_k[2];
            double n_ax = static_cast<double>(norms0[k][0]);
            double n_ay = static_cast<double>(norms0[k][1]);
            double r_k  = fx * std::sqrt(std::pow(n_px - n_ax, 2) + std::pow(n_py - n_ay, 2));
            sum_r2_tri += r_k * r_k;
            n_tri++;
          }
          if (n_tri >= 2 && std::sqrt(sum_r2_tri / n_tri) > _imu_residual_max_tri_error)
            tri_gate_ok = false;
        }

        if (tri_gate_ok) {
          // Variance mode: std-dev of per-clone reprojection residuals.
          // Triangulation error adds a constant bias c to every r_k → cancels in variance.
          // A static feature has low std_r (consistent bias); a dynamic feature has high
          // std_r (true position changes, p_FinG stays fixed → different residual each clone).
          // Requires ≥3 valid clones; falls back to nm=1 for short tracks.
          if (_use_imu_residual && _use_residual_variance) {
            double sum_r = 0.0, sum_r2 = 0.0;
            int n_var = 0;
            for (size_t k = 0; k < times0.size(); k++) {
              double t_k = times0[k];
              if (!state->_clones_IMU.count(t_k))
                continue;
              Eigen::Matrix3d R_GtoI_k  = state->_clones_IMU.at(t_k)->Rot();
              Eigen::Vector3d p_IinG_k  = state->_clones_IMU.at(t_k)->pos();
              Eigen::Matrix3d R_GtoC0_k = R_ItoC0 * R_GtoI_k;
              Eigen::Vector3d p_C0inG_k = p_IinG_k - R_GtoC0_k.transpose() * p_C0inI;
              Eigen::Vector3d X_k       = R_GtoC0_k * (feat.p_FinG - p_C0inG_k);
              if (X_k[2] <= 0.1)
                continue;
              double n_px = X_k[0] / X_k[2];
              double n_py = X_k[1] / X_k[2];
              double n_ax = static_cast<double>(norms0[k][0]);
              double n_ay = static_cast<double>(norms0[k][1]);
              double r_k  = fx * std::sqrt(std::pow(n_px - n_ax, 2) + std::pow(n_py - n_ay, 2));
              sum_r  += r_k;
              sum_r2 += r_k * r_k;
              n_var++;
            }
            if (n_var >= 3) {
              double mean_r = sum_r / n_var;
              double var_r  = std::max(0.0, sum_r2 / n_var - mean_r * mean_r);
              double std_r  = std::sqrt(var_r);
              double s      = std::exp(-std_r / _imu_residual_sigma_px);
              nm = std::max(1.0, 1.0 + _imu_residual_alpha * (1.0 - s));
            }
            // n_var < 3: nm stays 1.0 (too few observations, trust the feature)
          }

          // Single-frame residual at newest clone.
          double t_new = times0.back();
          if (state->_clones_IMU.count(t_new)) {
            Eigen::Matrix3d R_GtoI_new  = state->_clones_IMU.at(t_new)->Rot();
            Eigen::Vector3d p_IinG_new  = state->_clones_IMU.at(t_new)->pos();
            Eigen::Matrix3d R_GtoC0_new = R_ItoC0 * R_GtoI_new;
            Eigen::Vector3d p_C0inG_new = p_IinG_new - R_GtoC0_new.transpose() * p_C0inI;
            Eigen::Vector3d X_Ct        = R_GtoC0_new * (feat.p_FinG - p_C0inG_new);
            if (X_Ct[2] > 0.1) {
              double n_pred_x = X_Ct[0] / X_Ct[2];
              double n_pred_y = X_Ct[1] / X_Ct[2];
              double n_act_x  = norms0.back()[0];
              double n_act_y  = norms0.back()[1];
              double r_px     = fx * std::sqrt(std::pow(n_pred_x - n_act_x, 2) + std::pow(n_pred_y - n_act_y, 2));

              if (!_use_residual_variance) {
                double noise_floor = _imu_residual_dead_zone * _options.sigma_pix;
                double effective_r = std::max(0.0, r_px - noise_floor);
                double s_imu       = std::exp(-effective_r / _imu_residual_sigma_px);
                nm = std::max(1.0, 1.0 + _imu_residual_alpha * (1.0 - s_imu));
              }

              // Depth gate: features beyond max_depth have low stereo disparity and
              // unreliable p_FinG → suppress nm to avoid false inflation.
              // Applies to both single-frame and variance modes (X_Ct is from newest clone).
              if (nm > 1.0 && _imu_residual_max_depth > 0.0 && X_Ct[2] > _imu_residual_max_depth) {
                nm = 1.0;
              }
            }
          }
        } // tri_gate_ok
      }
    }

    // Chi2 distance check — use inflated noise nm*sigma_pix_sq as expected model.
    // Dynamic features are gated at their actual expected noise level, not the
    // static baseline (prevents hard-rejection of slow-moving dynamic features).
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += (nm * _options.sigma_pix_sq) * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is more)
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we should delete or not
    if (chi2 > _options.chi2_multipler * chi2_check) {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Measurement whitening for per-feature noise: divide H_x and res by sqrt(nm).
    // After this scaling the global R_big = sigma_pix_sq*I is the correct noise model,
    // equivalent to nm*sigma_pix_sq per feature in the Kalman update.
    // Dynamic features (nm >> 1) are down-weighted; static features (nm = 1.0) unchanged.
    if (nm > 1.0) {
      double inv_sqrt_nm = 1.0 / std::sqrt(nm);
      H_x *= inv_sqrt_nm;
      res *= inv_sqrt_nm;
    }

    // We are good!!! Append to our large H vector
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {

      // Ensure that this variable is in our Jacobian
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }

      // Append to our large Jacobian
      Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
      ct_hx += var->size();
    }

    // Append our residual and move forward
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    it2++;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);

  // 5. Perform measurement compression
  UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
  if (Hx_big.rows() < 1) {
    return;
  }
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

  // 6. With all good features update the state
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds create system (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds compress system\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds update state (%d size)\n", (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}
