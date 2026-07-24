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
#include <algorithm>
#include <cmath>
#include <limits>

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
  const bool use_feature_sigmas = static_cast<bool>(sigma_provider);

  struct PreparedCandidate {
    std::shared_ptr<Feature> feature;
    size_t feature_id = 0;
    size_t track_measurements = 0;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd residual;
    std::vector<std::shared_ptr<Type>> Hx_order;
    Eigen::MatrixXd innovation_without_measurement_noise;
    double chi2_threshold = 0.0;
    double stock_chi2 = 0.0;
    MsckfVisualInput model_input{};
  };
  std::vector<PreparedCandidate> prepared;
  prepared.reserve(feature_vec.size());

  // 4a. Build all feature systems from causal pre-decision values. Net A runs
  // on this complete set before learned noise changes any gate decision.
  for (const auto &feature : feature_vec) {
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = feature->featid;
    feat.uvs = feature->uvs;
    feat.uvs_norm = feature->uvs_norm;
    feat.timestamps = feature->timestamps;
    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE)
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = feature->anchor_cam_id;
      feat.anchor_clone_timestamp = feature->anchor_clone_timestamp;
      feat.p_FinA = feature->p_FinA;
      feat.p_FinA_fej = feature->p_FinA;
    } else {
      feat.p_FinG = feature->p_FinG;
      feat.p_FinG_fej = feature->p_FinG;
    }

    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd residual;
    std::vector<std::shared_ptr<Type>> Hx_order;
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, residual, Hx_order);
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, residual);

    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd innovation_without_noise = H_x * P_marg * H_x.transpose();
    Eigen::MatrixXd stock_innovation = innovation_without_noise;
    stock_innovation.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(stock_innovation.rows());
    const double stock_chi2 = residual.dot(stock_innovation.llt().solve(residual));

    double chi2_check;
    if (residual.rows() < 500) {
      chi2_check = chi_squared_table[residual.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(residual.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)residual.rows());
    }
    const double chi2_threshold = _options.chi2_multipler * chi2_check;

    size_t track_measurements = 0;
    double last_camera = -1.0;
    double last_timestamp = -std::numeric_limits<double>::infinity();
    double last_u = 0.0;
    double last_v = 0.0;
    for (const auto &camera_timestamps : feature->timestamps) {
      track_measurements += camera_timestamps.second.size();
      const auto uv_it = feature->uvs.find(camera_timestamps.first);
      if (uv_it == feature->uvs.end())
        continue;
      const size_t count = std::min(camera_timestamps.second.size(), uv_it->second.size());
      for (size_t index = 0; index < count; ++index) {
        if (camera_timestamps.second[index] > last_timestamp) {
          last_timestamp = camera_timestamps.second[index];
          last_camera = static_cast<double>(camera_timestamps.first);
          last_u = uv_it->second[index](0);
          last_v = uv_it->second[index](1);
        }
      }
    }
    MsckfVisualInput model_input{{
        std::log1p(static_cast<double>(track_measurements)),
        std::min(1.0, std::max(last_camera, 0.0)),
        last_u / 752.0,
        last_v / 480.0,
        std::log1p(std::max(residual.norm(), 0.0)),
        std::log1p(std::max(stock_chi2, 0.0)),
        std::log1p(std::max(chi2_threshold, 0.0)),
        _options.sigma_pix,
    }};
    for (double value : model_input) {
      if (!std::isfinite(value))
        throw std::runtime_error("non-finite live Net-A input");
    }
    prepared.push_back(PreparedCandidate{
        feature,
        feat.featid,
        track_measurements,
        std::move(H_x),
        std::move(residual),
        std::move(Hx_order),
        std::move(innovation_without_noise),
        chi2_threshold,
        stock_chi2,
        model_input,
    });
  }

  std::vector<double> feature_sigmas(prepared.size(), _options.sigma_pix);
  if (use_feature_sigmas && !prepared.empty()) {
    std::vector<MsckfVisualInput> model_inputs;
    model_inputs.reserve(prepared.size());
    size_t stock_gate_passes = 0;
    for (const auto &candidate : prepared) {
      model_inputs.push_back(candidate.model_input);
      if (candidate.stock_chi2 <= candidate.chi2_threshold)
        ++stock_gate_passes;
    }
    const MsckfFrameContext frame_context{{
        std::log1p(std::max(live_frame_context[0], 0.0)),
        std::log1p(std::max(live_frame_context[1], 0.0)),
        live_frame_context[2] / 255.0,
        static_cast<double>(state->max_covariance_size()) / 1000.0,
        std::log1p(static_cast<double>(prepared.size())),
        static_cast<double>(stock_gate_passes) / static_cast<double>(prepared.size()),
    }};
    feature_sigmas = sigma_provider(model_inputs, frame_context);
    if (feature_sigmas.size() != prepared.size())
      throw std::runtime_error("live Net-A returned the wrong number of feature sigmas");
    for (double sigma : feature_sigmas) {
      if (!std::isfinite(sigma) || sigma <= 0.0)
        throw std::runtime_error("live Net-A returned an invalid feature sigma");
    }
  }

  // 4b. Apply the predicted sigmas to the actual gates and stacked update.
  feature_vec.clear();
  for (size_t candidate_index = 0; candidate_index < prepared.size(); ++candidate_index) {
    auto &candidate = prepared[candidate_index];
    const double sigma_pix = feature_sigmas[candidate_index];
    Eigen::MatrixXd innovation = candidate.innovation_without_measurement_noise;
    innovation.diagonal() += sigma_pix * sigma_pix * Eigen::VectorXd::Ones(innovation.rows());
    const double chi2 = candidate.residual.dot(innovation.llt().solve(candidate.residual));
    const bool passed_chi2_gate = chi2 <= candidate.chi2_threshold;

    if (diagnostic_callback) {
      MsckfFeatureDiagnostic diagnostic;
      diagnostic.timestamp = state->_timestamp;
      diagnostic.feature_id = candidate.feature_id;
      diagnostic.track_measurements = candidate.track_measurements;
      diagnostic.filter_residual_norm = candidate.residual.norm();
      diagnostic.chi2 = chi2;
      diagnostic.chi2_threshold = candidate.chi2_threshold;
      diagnostic.sigma_pix = sigma_pix;
      diagnostic.passed_chi2_gate = passed_chi2_gate;
      diagnostic.feature = candidate.feature;
      diagnostic_callback(diagnostic);
    }

    if (!passed_chi2_gate) {
      candidate.feature->to_delete = true;
      continue;
    }
    feature_vec.push_back(candidate.feature);

    if (use_feature_sigmas) {
      candidate.H_x /= sigma_pix;
      candidate.residual /= sigma_pix;
    }

    size_t ct_hx = 0;
    for (const auto &var : candidate.Hx_order) {
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }
      Hx_big.block(ct_meas, Hx_mapping[var], candidate.H_x.rows(), var->size()) =
          candidate.H_x.block(0, ct_hx, candidate.H_x.rows(), var->size());
      ct_hx += var->size();
    }
    res_big.block(ct_meas, 0, candidate.residual.rows(), 1) = candidate.residual;
    ct_meas += candidate.residual.rows();
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
  Eigen::MatrixXd R_big = (use_feature_sigmas ? 1.0 : _options.sigma_pix_sq) *
                          Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

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
