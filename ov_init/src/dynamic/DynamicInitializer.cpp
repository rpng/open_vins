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

#include "DynamicInitializer.h"

#include "ceres/Factor_GenericPrior.h"
#include "ceres/Factor_ImageReprojCalib.h"
#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "utils/helper.h"

#include "cpi/CpiV1.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

bool DynamicInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                    std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                                    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM) {

  // Get the newest and oldest timestamps we will try to initialize between!
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  double oldest_time = newest_cam_time - params.init_window_time;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }

  // Remove all measurements that are older than our initialization window
  // Then we will try to use all features that are in the feature database!
  _db->cleanup_measurements(oldest_time);
  bool have_old_imu_readings = false;
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    have_old_imu_readings = true;
    it_imu = imu_data->erase(it_imu);
  }
  if (_db->get_internal_data().size() < 0.75 * params.init_max_features) {
    PRINT_WARNING(RED "[init-d]: only %zu valid features of required (%.0f thresh)!!\n" RESET, _db->get_internal_data().size(),
                  0.95 * params.init_max_features);
    return false;
  }
  if (imu_data->size() < 2 || !have_old_imu_readings) {
    // PRINT_WARNING(RED "[init-d]: waiting for window to reach full size (%zu imu readings)!!\n" RESET, imu_data->size());
    return false;
  }

  // Now we will make a copy of our features here
  // We do this to ensure that the feature database can continue to have new
  // measurements appended to it in an async-manor so this initialization
  // can be performed in a secondary thread while feature tracking is still performed.
  std::unordered_map<size_t, std::shared_ptr<Feature>> features;
  for (const auto &feat : _db->get_internal_data()) {
    auto feat_new = std::make_shared<Feature>();
    feat_new->featid = feat.second->featid;
    feat_new->uvs = feat.second->uvs;
    feat_new->uvs_norm = feat.second->uvs_norm;
    feat_new->timestamps = feat.second->timestamps;
    features.insert({feat.first, feat_new});
  }

  // ======================================================
  // ======================================================

  // Settings
  const int min_num_meas_to_optimize = (int)params.init_window_time;
  const int min_valid_features = 8;

  // Validation information for features we can use
  bool have_stereo = false;
  int count_valid_features = 0;
  std::map<size_t, int> map_features_num_meas;
  int num_measurements = 0;
  double oldest_camera_time = INFINITY;
  std::map<double, bool> map_camera_times;
  map_camera_times[newest_cam_time] = true; // always insert final pose
  std::map<size_t, bool> map_camera_ids;
  double pose_dt_avg = params.init_window_time / (double)(params.init_dyn_num_pose + 1);
  for (auto const &feat : features) {

    // Loop through each timestamp and make sure it is a valid pose
    std::vector<double> times;
    std::map<size_t, bool> camids;
    for (auto const &camtime : feat.second->timestamps) {
      for (double time : camtime.second) {
        double time_dt = INFINITY;
        for (auto const &tmp : map_camera_times) {
          time_dt = std::min(time_dt, std::abs(time - tmp.first));
        }
        for (auto const &tmp : times) {
          time_dt = std::min(time_dt, std::abs(time - tmp));
        }
        // either this pose is a new one at the desired frequency
        // or it is a timestamp that we already have, thus can use for free
        if (time_dt >= pose_dt_avg || time_dt == 0.0) {
          times.push_back(time);
          camids[camtime.first] = true;
        }
      }
    }

    // This isn't a feature we should use if there are not enough measurements
    map_features_num_meas[feat.first] = (int)times.size();
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;

    // If we have enough measurements we should append this feature!
    for (auto const &tmp : times) {
      map_camera_times[tmp] = true;
      oldest_camera_time = std::min(oldest_camera_time, tmp);
      num_measurements += 2;
    }
    for (auto const &tmp : camids) {
      map_camera_ids[tmp.first] = true;
    }
    if (camids.size() > 1) {
      have_stereo = true;
    }
    count_valid_features++;
  }

  // Return if we do not have our full window or not enough measurements
  // Also check that we have enough features to initialize with
  if ((int)map_camera_times.size() < params.init_dyn_num_pose) {
    return false;
  }
  if (count_valid_features < min_valid_features) {
    PRINT_WARNING(RED "[init-d]: only %zu valid features of required %d!!\n" RESET, count_valid_features, min_valid_features);
    return false;
  }

  // Bias initial guesses specified by the launch file
  // We don't go through the effort to recover the biases right now since they should be
  // Semi-well known before launching or can be considered to be near zero...
  Eigen::Vector3d gyroscope_bias = params.init_dyn_bias_g;
  Eigen::Vector3d accelerometer_bias = params.init_dyn_bias_a;

  // Check that we have some angular velocity / orientation change
  double accel_inI_norm = 0.0;
  double theta_inI_norm = 0.0;
  double time0_in_imu = oldest_camera_time + params.calib_camimu_dt;
  double time1_in_imu = newest_cam_time + params.calib_camimu_dt;
  std::vector<ov_core::ImuData> readings = InitializerHelper::select_imu_readings(*imu_data, time0_in_imu, time1_in_imu);
  assert(readings.size() > 2);
  for (size_t k = 0; k < readings.size() - 1; k++) {
    auto imu0 = readings.at(k);
    auto imu1 = readings.at(k + 1);
    double dt = imu1.timestamp - imu0.timestamp;
    Eigen::Vector3d wm = 0.5 * (imu0.wm + imu1.wm) - gyroscope_bias;
    Eigen::Vector3d am = 0.5 * (imu0.am + imu1.am) - accelerometer_bias;
    theta_inI_norm += (-wm * dt).norm();
    accel_inI_norm += am.norm();
  }
  accel_inI_norm /= (double)(readings.size() - 1);
  if (180.0 / M_PI * theta_inI_norm < params.init_dyn_min_deg) {
    PRINT_WARNING(YELLOW "[init-d]: gyroscope only %.2f degree change (%.2f thresh)\n" RESET, 180.0 / M_PI * theta_inI_norm,
                  params.init_dyn_min_deg);
    return false;
  }
  PRINT_DEBUG("[init-d]: |theta_I| = %.4f deg and |accel| = %.4f\n", 180.0 / M_PI * theta_inI_norm, accel_inI_norm);

  //  // Create feature bearing vector in the first frame
  //  // This gives us: p_FinI0 = depth * bearing
  //  Eigen::Vector4d q_ItoC = data_ori.camera_q_ItoC.at(cam_id);
  //  Eigen::Vector3d p_IinC = data_init.camera_p_IinC.at(cam_id);
  //  Eigen::Matrix3d R_ItoC = quat_2_Rot(q_ItoC);
  //  std::map<size_t, Eigen::Vector3d> features_bearings;
  //  std::map<size_t, int> features_index;
  //  for (auto const &feat : features) {
  //  if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
  //    continue;
  //    assert(feat->timestamps.find(cam_id) != feat->timestamps.end());
  //    double timestamp = data_ori.timestamps_cam.at(cam_id).at(0);
  //    auto it0 = std::find(feat->timestamps.at(cam_id).begin(), feat->timestamps.at(cam_id).end(), timestamp);
  //    if (it0 == feat->timestamps.at(cam_id).end())
  //      continue;
  //    auto idx0 = std::distance(feat->timestamps.at(cam_id).begin(), it0);
  //    Eigen::Vector3d bearing;
  //    bearing << feat->uvs_norm.at(cam_id).at(idx0)(0), feat->uvs_norm.at(cam_id).at(idx0)(1), 1;
  //    bearing = bearing / bearing.norm();
  //    bearing = R_ItoC.transpose() * bearing;
  //    features_bearings.insert({feat->featid, bearing});
  //    features_index.insert({feat->featid, (int)features_index.size()});
  //  }
  auto rT2 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // We will recover position of feature, velocity, gravity
  // Based on Equation (14) in the following paper:
  // https://ieeexplore.ieee.org/abstract/document/6386235
  // State ordering is: [features, velocity, gravity]
  // Feature size of 1 will use the first ever bearing of the feature as true (depth only..)
  const bool use_single_depth = false;
  int size_feature = (use_single_depth) ? 1 : 3;
  int num_features = count_valid_features;
  int system_size = size_feature * num_features + 3 + 3;

  // Make sure we have enough measurements to fully constrain the system
  if (num_measurements < system_size) {
    PRINT_WARNING(YELLOW "[init-d]: not enough feature measurements (%d meas vs %d state size)!\n" RESET, num_measurements, system_size);
    return false;
  }

  // Now lets pre-integrate from the first time to the last
  assert(oldest_camera_time < newest_cam_time);
  double last_camera_timestamp = 0.0;
  std::map<double, std::shared_ptr<ov_core::CpiV1>> map_camera_cpi_I0toIi, map_camera_cpi_IitoIi1;
  for (auto const &timepair : map_camera_times) {

    // No preintegration at the first timestamp
    double current_time = timepair.first;
    if (current_time == oldest_camera_time) {
      map_camera_cpi_I0toIi.insert({current_time, nullptr});
      map_camera_cpi_IitoIi1.insert({current_time, nullptr});
      last_camera_timestamp = current_time;
      continue;
    }

    // Perform our preintegration from I0 to Ii (used in the linear system)
    double cpiI0toIi1_time0_in_imu = oldest_camera_time + params.calib_camimu_dt;
    double cpiI0toIi1_time1_in_imu = current_time + params.calib_camimu_dt;
    auto cpiI0toIi1 = std::make_shared<ov_core::CpiV1>(params.sigma_w, params.sigma_wb, params.sigma_a, params.sigma_ab, true);
    cpiI0toIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);
    std::vector<ov_core::ImuData> cpiI0toIi1_readings =
        InitializerHelper::select_imu_readings(*imu_data, cpiI0toIi1_time0_in_imu, cpiI0toIi1_time1_in_imu);
    if (cpiI0toIi1_readings.size() < 2) {
      PRINT_DEBUG(YELLOW "[init-d]: camera %.2f in has %zu IMU readings!\n" RESET, (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu),
                  cpiI0toIi1_readings.size());
      return false;
    }
    double cpiI0toIi1_dt_imu = cpiI0toIi1_readings.at(cpiI0toIi1_readings.size() - 1).timestamp - cpiI0toIi1_readings.at(0).timestamp;
    if (std::abs(cpiI0toIi1_dt_imu - (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu)) > 0.01) {
      PRINT_DEBUG(YELLOW "[init-d]: camera IMU was only propagated %.3f of %.3f\n" RESET, cpiI0toIi1_dt_imu,
                  (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu));
      return false;
    }
    for (size_t k = 0; k < cpiI0toIi1_readings.size() - 1; k++) {
      auto imu0 = cpiI0toIi1_readings.at(k);
      auto imu1 = cpiI0toIi1_readings.at(k + 1);
      cpiI0toIi1->feed_IMU(imu0.timestamp, imu1.timestamp, imu0.wm, imu0.am, imu1.wm, imu1.am);
    }

    // Perform our preintegration from Ii to Ii1 (used in the mle optimization)
    double cpiIitoIi1_time0_in_imu = last_camera_timestamp + params.calib_camimu_dt;
    double cpiIitoIi1_time1_in_imu = current_time + params.calib_camimu_dt;
    auto cpiIitoIi1 = std::make_shared<ov_core::CpiV1>(params.sigma_w, params.sigma_wb, params.sigma_a, params.sigma_ab, true);
    cpiIitoIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);
    std::vector<ov_core::ImuData> cpiIitoIi1_readings =
        InitializerHelper::select_imu_readings(*imu_data, cpiIitoIi1_time0_in_imu, cpiIitoIi1_time1_in_imu);
    if (cpiIitoIi1_readings.size() < 2) {
      PRINT_DEBUG(YELLOW "[init-d]: camera %.2f in has %zu IMU readings!\n" RESET, (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu),
                  cpiIitoIi1_readings.size());
      return false;
    }
    double cpiIitoIi1_dt_imu = cpiIitoIi1_readings.at(cpiIitoIi1_readings.size() - 1).timestamp - cpiIitoIi1_readings.at(0).timestamp;
    if (std::abs(cpiIitoIi1_dt_imu - (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu)) > 0.01) {
      PRINT_DEBUG(YELLOW "[init-d]: camera IMU was only propagated %.3f of %.3f\n" RESET, cpiIitoIi1_dt_imu,
                  (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu));
      return false;
    }
    for (size_t k = 0; k < cpiIitoIi1_readings.size() - 1; k++) {
      auto imu0 = cpiIitoIi1_readings.at(k);
      auto imu1 = cpiIitoIi1_readings.at(k + 1);
      cpiIitoIi1->feed_IMU(imu0.timestamp, imu1.timestamp, imu0.wm, imu0.am, imu1.wm, imu1.am);
    }

    // Finally push back our integrations!
    map_camera_cpi_I0toIi.insert({current_time, cpiI0toIi1});
    map_camera_cpi_IitoIi1.insert({current_time, cpiIitoIi1});
    last_camera_timestamp = current_time;
  }

  // Loop through each feature observation and append it!
  // State ordering is: [features, velocity, gravity]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_measurements, system_size);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_measurements);
  PRINT_DEBUG("[init-d]: system of %d measurement x %d states created (%d features, %s)\n", num_measurements, system_size, num_features,
              (have_stereo) ? "stereo" : "mono");
  int index_meas = 0;
  int idx_feat = 0;
  std::map<size_t, int> A_index_features;
  for (auto const &feat : features) {
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;
    if (A_index_features.find(feat.first) == A_index_features.end()) {
      A_index_features.insert({feat.first, idx_feat});
      idx_feat += 1;
    }
    for (auto const &camtime : feat.second->timestamps) {

      // This camera
      size_t cam_id = camtime.first;
      Eigen::Vector4d q_ItoC = params.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = params.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
      Eigen::Matrix3d R_ItoC = quat_2_Rot(q_ItoC);

      // Loop through each observation
      for (size_t i = 0; i < camtime.second.size(); i++) {

        // Skip measurements we don't have poses for
        double time = feat.second->timestamps.at(cam_id).at(i);
        if (map_camera_times.find(time) == map_camera_times.end())
          continue;

        // Our measurement
        Eigen::Vector2d uv_norm;
        uv_norm << (double)feat.second->uvs_norm.at(cam_id).at(i)(0), (double)feat.second->uvs_norm.at(cam_id).at(i)(1);

        // Preintegration values
        double DT = 0.0;
        Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
        if (map_camera_cpi_I0toIi.find(time) != map_camera_cpi_I0toIi.end() && map_camera_cpi_I0toIi.at(time) != nullptr) {
          DT = map_camera_cpi_I0toIi.at(time)->DT;
          R_I0toIk = map_camera_cpi_I0toIi.at(time)->R_k2tau;
          alpha_I0toIk = map_camera_cpi_I0toIi.at(time)->alpha_tau;
        }

        // Create the linear system based on the feature reprojection
        // [ 1 0 -u ] p_FinCi = [ 0 ]
        // [ 0 1 -v ]           [ 0 ]
        // where
        // p_FinCi = R_C0toCi * R_ItoC * (p_FinI0 - p_IiinI0) + p_IinC
        //         = R_C0toCi * R_ItoC * (p_FinI0 - v_I0inI0 * dt - 0.5 * grav_inI0 * dt^2 - alpha) + p_IinC
        Eigen::MatrixXd H_proj = Eigen::MatrixXd::Zero(2, 3);
        H_proj << 1, 0, -uv_norm(0), 0, 1, -uv_norm(1);
        Eigen::MatrixXd Y = H_proj * R_ItoC * R_I0toIk;
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(2, system_size);
        Eigen::MatrixXd b_i = Y * alpha_I0toIk - H_proj * p_IinC;
        if (size_feature == 1) {
          assert(false);
          // Substitute in p_FinI0 = z*bearing_inC0_rotI0 - R_ItoC^T*p_IinC
          // H_i.block(0, size_feature * A_index_features.at(feat.first), 2, 1) = Y * features_bearings.at(feat.first);
          // b_i += Y * R_ItoC.transpose() * p_IinC;
        } else {
          H_i.block(0, size_feature * A_index_features.at(feat.first), 2, 3) = Y; // feat
        }
        H_i.block(0, size_feature * num_features + 0, 2, 3) = -DT * Y;            // vel
        H_i.block(0, size_feature * num_features + 3, 2, 3) = 0.5 * DT * DT * Y;  // grav

        // Else lets append this to our system!
        A.block(index_meas, 0, 2, A.cols()) = H_i;
        b.block(index_meas, 0, 2, 1) = b_i;
        index_meas += 2;
      }
    }
  }
  auto rT3 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // Solve the linear system without constraint
  // Eigen::MatrixXd AtA = A.transpose() * A;
  // Eigen::MatrixXd Atb = A.transpose() * b;
  // Eigen::MatrixXd x_hat = AtA.colPivHouseholderQr().solve(Atb);

  // Constrained solving |g| = 9.81 constraint
  Eigen::MatrixXd A1 = A.block(0, 0, A.rows(), A.cols() - 3);
  // Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).inverse();
  Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).llt().solve(Eigen::MatrixXd::Identity(A1.cols(), A1.cols()));
  Eigen::MatrixXd A2 = A.block(0, A.cols() - 3, A.rows(), 3);
  Eigen::MatrixXd Temp = A2.transpose() * (Eigen::MatrixXd::Identity(A1.rows(), A1.rows()) - A1 * A1A1_inv * A1.transpose());
  Eigen::MatrixXd D = Temp * A2;
  Eigen::MatrixXd d = Temp * b;
  Eigen::Matrix<double, 7, 1> coeff = InitializerHelper::compute_dongsi_coeff(D, d, params.gravity_mag);

  // Create companion matrix of our polynomial
  // https://en.wikipedia.org/wiki/Companion_matrix
  assert(coeff(0) == 1);
  Eigen::Matrix<double, 6, 6> companion_matrix = Eigen::Matrix<double, 6, 6>::Zero(coeff.rows() - 1, coeff.rows() - 1);
  companion_matrix.diagonal(-1).setOnes();
  companion_matrix.col(companion_matrix.cols() - 1) = -coeff.reverse().head(coeff.rows() - 1);
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd0(companion_matrix);
  Eigen::MatrixXd singularValues0 = svd0.singularValues();
  double cond0 = singularValues0(0) / singularValues0(singularValues0.rows() - 1);
  PRINT_DEBUG("[init-d]: CM cond = %.3f | rank = %d of %d (%4.3e thresh)\n", cond0, (int)svd0.rank(), (int)companion_matrix.cols(),
              svd0.threshold());
  if (svd0.rank() != companion_matrix.rows()) {
    PRINT_ERROR(RED "[init-d]: eigenvalue decomposition not full rank!!\n" RESET);
    return false;
  }

  // Find its eigenvalues (can be complex)
  Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(companion_matrix, false);
  if (solver.info() != Eigen::Success) {
    PRINT_ERROR(RED "[init-d]: failed to compute the eigenvalue decomposition!!\n" RESET);
    return false;
  }

  // Find the smallest real eigenvalue
  // NOTE: we find the one that gives us minimal constraint cost
  // NOTE: not sure if the best, but one that gives the correct mag should be good?
  bool lambda_found = false;
  double lambda_min = -1;
  double cost_min = INFINITY;
  Eigen::MatrixXd I_dd = Eigen::MatrixXd::Identity(D.rows(), D.rows());
  // double g2 = params.gravity_mag * params.gravity_mag;
  // Eigen::MatrixXd ddt = d * d.transpose();
  for (int i = 0; i < solver.eigenvalues().size(); i++) {
    auto val = solver.eigenvalues()(i);
    if (val.imag() == 0) {
      double lambda = val.real();
      // Eigen::MatrixXd mat = (D - lambda * I_dd) * (D - lambda * I_dd) - 1 / g2 * ddt;
      // double cost = mat.determinant();
      Eigen::MatrixXd D_lambdaI_inv = (D - lambda * I_dd).llt().solve(I_dd);
      Eigen::VectorXd state_grav = D_lambdaI_inv * d;
      double cost = std::abs(state_grav.norm() - params.gravity_mag);
      // std::cout << lambda << " - " << cost << " -> " << state_grav.transpose() << std::endl;
      if (!lambda_found || cost < cost_min) {
        lambda_found = true;
        lambda_min = lambda;
        cost_min = cost;
      }
    }
  }
  if (!lambda_found) {
    PRINT_ERROR(RED "[init-d]: failed to find a real eigenvalue!!!\n" RESET);
    return false;
  }
  PRINT_DEBUG("[init-d]: smallest real eigenvalue = %.5f (cost of %f)\n", lambda_min, cost_min);

  // Recover our gravity from the constraint!
  // Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).inverse();
  Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).llt().solve(I_dd);
  Eigen::VectorXd state_grav = D_lambdaI_inv * d;

  // Overwrite our state: [features, velocity, gravity]
  Eigen::VectorXd state_feat_vel = -A1A1_inv * A1.transpose() * A2 * state_grav + A1A1_inv * A1.transpose() * b;
  Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(system_size, 1);
  x_hat.block(0, 0, size_feature * num_features + 3, 1) = state_feat_vel;
  x_hat.block(size_feature * num_features + 3, 0, 3, 1) = state_grav;
  Eigen::Vector3d v_I0inI0 = x_hat.block(size_feature * num_features + 0, 0, 3, 1);
  PRINT_INFO("[init-d]: velocity in I0 was %.3f,%.3f,%.3f and |v| = %.4f\n", v_I0inI0(0), v_I0inI0(1), v_I0inI0(2), v_I0inI0.norm());

  // Check gravity magnitude to see if converged
  Eigen::Vector3d gravity_inI0 = x_hat.block(size_feature * num_features + 3, 0, 3, 1);
  double init_max_grav_difference = 1e-3;
  if (std::abs(gravity_inI0.norm() - params.gravity_mag) > init_max_grav_difference) {
    PRINT_WARNING(YELLOW "[init-d]: gravity did not converge (%.3f > %.3f)\n" RESET, std::abs(gravity_inI0.norm() - params.gravity_mag),
                  init_max_grav_difference);
    return false;
  }
  PRINT_INFO("[init-d]: gravity in I0 was %.3f,%.3f,%.3f and |g| = %.4f\n", gravity_inI0(0), gravity_inI0(1), gravity_inI0(2),
             gravity_inI0.norm());
  auto rT4 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // Extract imu state elements
  std::map<double, Eigen::VectorXd> ori_I0toIi, pos_IiinI0, vel_IiinI0;
  for (auto const &timepair : map_camera_times) {

    // Timestamp of this pose
    double time = timepair.first;

    // Get our CPI integration values
    double DT = 0.0;
    Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd beta_I0toIk = Eigen::MatrixXd::Zero(3, 1);
    if (map_camera_cpi_I0toIi.find(time) != map_camera_cpi_I0toIi.end() && map_camera_cpi_I0toIi.at(time) != nullptr) {
      auto cpi = map_camera_cpi_I0toIi.at(time);
      DT = cpi->DT;
      R_I0toIk = cpi->R_k2tau;
      alpha_I0toIk = cpi->alpha_tau;
      beta_I0toIk = cpi->beta_tau;
    }

    // Integrate to get the relative to the current timestamp
    Eigen::Vector3d p_IkinI0 = v_I0inI0 * DT - 0.5 * gravity_inI0 * DT * DT + alpha_I0toIk;
    Eigen::Vector3d v_IkinI0 = v_I0inI0 - gravity_inI0 * DT + beta_I0toIk;

    // Record the values all transformed to the I0 frame
    ori_I0toIi.insert({time, rot_2_quat(R_I0toIk)});
    pos_IiinI0.insert({time, p_IkinI0});
    vel_IiinI0.insert({time, v_IkinI0});
  }

  // Recover the features in the first IMU frame
  count_valid_features = 0;
  std::map<size_t, Eigen::Vector3d> features_inI0;
  for (auto const &feat : features) {
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;
    Eigen::Vector3d p_FinI0;
    if (size_feature == 1) {
      assert(false);
      // double depth = x_hat(size_feature * A_index_features.at(feat.first), 0);
      // p_FinI0 = depth * features_bearings.at(feat.first) - R_ItoC.transpose() * p_IinC;
    } else {
      p_FinI0 = x_hat.block(size_feature * A_index_features.at(feat.first), 0, 3, 1);
    }
    bool is_behind = false;
    for (auto const &camtime : feat.second->timestamps) {
      size_t cam_id = camtime.first;
      Eigen::Vector4d q_ItoC = params.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = params.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
      Eigen::Vector3d p_FinC0 = quat_2_Rot(q_ItoC) * p_FinI0 + p_IinC;
      if (p_FinC0(2) < 0) {
        is_behind = true;
      }
    }
    if (!is_behind) {
      features_inI0.insert({feat.first, p_FinI0});
      count_valid_features++;
    }
  }
  if (count_valid_features < min_valid_features) {
    PRINT_ERROR(YELLOW "[init-d]: not enough features for our mle (%zu < %d)!\n" RESET, count_valid_features, min_valid_features);
    return false;
  }

  // Convert our states to be a gravity aligned global frame of reference
  // Here we say that the I0 frame is at 0,0,0 and shared the global origin
  Eigen::Matrix3d R_GtoI0;
  InitializerHelper::gram_schmidt(gravity_inI0, R_GtoI0);
  Eigen::Vector4d q_GtoI0 = rot_2_quat(R_GtoI0);
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, params.gravity_mag;
  std::map<double, Eigen::VectorXd> ori_GtoIi, pos_IiinG, vel_IiinG;
  std::map<size_t, Eigen::Vector3d> features_inG;
  for (auto const &timepair : map_camera_times) {
    ori_GtoIi[timepair.first] = quat_multiply(ori_I0toIi.at(timepair.first), q_GtoI0);
    pos_IiinG[timepair.first] = R_GtoI0.transpose() * pos_IiinI0.at(timepair.first);
    vel_IiinG[timepair.first] = R_GtoI0.transpose() * vel_IiinI0.at(timepair.first);
  }
  for (auto const &feat : features_inI0) {
    features_inG[feat.first] = R_GtoI0.transpose() * feat.second;
  }

  // ======================================================
  // ======================================================

  // Ceres problem stuff
  // NOTE: By default the problem takes ownership of the memory
  ceres::Problem problem;

  // Our system states (map from time to index)
  std::map<double, int> map_states;
  std::vector<double *> ceres_vars_ori;
  std::vector<double *> ceres_vars_pos;
  std::vector<double *> ceres_vars_vel;
  std::vector<double *> ceres_vars_bias_g;
  std::vector<double *> ceres_vars_bias_a;

  // Feature states (3dof p_FinG)
  std::map<size_t, int> map_features;
  std::vector<double *> ceres_vars_feat;

  // Setup extrinsic calibration q_ItoC, p_IinC (map from camera id to index)
  std::map<size_t, int> map_calib_cam2imu;
  std::vector<double *> ceres_vars_calib_cam2imu_ori;
  std::vector<double *> ceres_vars_calib_cam2imu_pos;

  // Setup intrinsic calibration focal, center, distortion (map from camera id to index)
  std::map<size_t, int> map_calib_cam;
  std::vector<double *> ceres_vars_calib_cam_intrinsics;

  // Helper lambda that will free any memory we have allocated
  auto free_state_memory = [&]() {
    for (auto const &ptr : ceres_vars_ori)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_pos)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_vel)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_bias_g)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_bias_a)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_feat)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam2imu_ori)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam2imu_pos)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam_intrinsics)
      delete[] ptr;
  };

  // Set the optimization settings
  // NOTE: We use dense schur since after eliminating features we have a dense problem
  // NOTE: http://ceres-solver.org/solving_faqs.html#solving
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // options.preconditioner_type = ceres::SCHUR_JACOBI;
  // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = params.init_dyn_mle_max_threads;
  options.max_solver_time_in_seconds = params.init_dyn_mle_max_time;
  options.max_num_iterations = params.init_dyn_mle_max_iter;
  // options.minimizer_progress_to_stdout = true;
  // options.linear_solver_ordering = ordering;
  options.function_tolerance = 1e-5;
  options.gradient_tolerance = 1e-4 * options.function_tolerance;

  // Loop through each CPI integration and add its measurement to the problem
  double timestamp_k = -1;
  for (auto const &timepair : map_camera_times) {

    // Get our predicted state at the requested camera timestep
    double timestamp_k1 = timepair.first;
    std::shared_ptr<ov_core::CpiV1> cpi = map_camera_cpi_IitoIi1.at(timestamp_k1);
    Eigen::Matrix<double, 16, 1> state_k1;
    state_k1.block(0, 0, 4, 1) = ori_GtoIi.at(timestamp_k1);
    state_k1.block(4, 0, 3, 1) = pos_IiinG.at(timestamp_k1);
    state_k1.block(7, 0, 3, 1) = vel_IiinG.at(timestamp_k1);
    state_k1.block(10, 0, 3, 1) = gyroscope_bias;
    state_k1.block(13, 0, 3, 1) = accelerometer_bias;

    // ================================================================
    //  ADDING GRAPH STATE / ESTIMATES!
    // ================================================================

    // Load our state variables into our allocated state pointers
    auto *var_ori = new double[4];
    for (int j = 0; j < 4; j++) {
      var_ori[j] = state_k1(0 + j, 0);
    }
    auto *var_pos = new double[3];
    auto *var_vel = new double[3];
    auto *var_bias_g = new double[3];
    auto *var_bias_a = new double[3];
    for (int j = 0; j < 3; j++) {
      var_pos[j] = state_k1(4 + j, 0);
      var_vel[j] = state_k1(7 + j, 0);
      var_bias_g[j] = state_k1(10 + j, 0);
      var_bias_a[j] = state_k1(13 + j, 0);
    }

    // Now actually create the parameter block in the ceres problem
    auto ceres_jplquat = new State_JPLQuatLocal();
    problem.AddParameterBlock(var_ori, 4, ceres_jplquat);
    problem.AddParameterBlock(var_pos, 3);
    problem.AddParameterBlock(var_vel, 3);
    problem.AddParameterBlock(var_bias_g, 3);
    problem.AddParameterBlock(var_bias_a, 3);

    // Fix this first ever pose to constrain the problem
    // NOTE: If we don't do this, then the problem won't be full rank
    // NOTE: Since init is over a small window, we are likely to be degenerate
    // NOTE: Thus we need to fix these parameters
    if (map_states.empty()) {

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(13, 1);
      for (int j = 0; j < 4; j++) {
        x_lin(0 + j) = var_ori[j];
      }
      for (int j = 0; j < 3; j++) {
        x_lin(4 + j) = var_pos[j];
        x_lin(7 + j) = var_bias_g[j];
        x_lin(10 + j) = var_bias_a[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(10, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(10, 10);
      prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1e-5, 2); // 4dof unobservable yaw and position
      prior_Info.block(4, 4, 3, 3) *= 1.0 / std::pow(0.05, 2); // bias_g prior
      prior_Info.block(7, 7, 3, 3) *= 1.0 / std::pow(0.10, 2); // bias_a prior

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_ori);
      x_types.emplace_back("quat_yaw");
      factor_params.push_back(var_pos);
      x_types.emplace_back("vec3");
      factor_params.push_back(var_bias_g);
      x_types.emplace_back("vec3");
      factor_params.push_back(var_bias_a);
      x_types.emplace_back("vec3");

      // Append it to the problem
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
    }

    // Append to our historical vector of states
    map_states.insert({timestamp_k1, (int)ceres_vars_ori.size()});
    ceres_vars_ori.push_back(var_ori);
    ceres_vars_pos.push_back(var_pos);
    ceres_vars_vel.push_back(var_vel);
    ceres_vars_bias_g.push_back(var_bias_g);
    ceres_vars_bias_a.push_back(var_bias_a);

    // ================================================================
    //  ADDING GRAPH FACTORS!
    // ================================================================

    // Append the new IMU factor
    if (cpi != nullptr) {
      assert(timestamp_k != -1);
      std::vector<double *> factor_params;
      factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k1)));
      auto *factor_imu = new Factor_ImuCPIv1(cpi->DT, gravity, cpi->alpha_tau, cpi->beta_tau, cpi->q_k2tau, cpi->b_a_lin, cpi->b_w_lin,
                                             cpi->J_q, cpi->J_b, cpi->J_a, cpi->H_b, cpi->H_a, cpi->P_meas);
      problem.AddResidualBlock(factor_imu, nullptr, factor_params);
    }

    // Move time forward
    timestamp_k = timestamp_k1;
  }

  // First make sure we have calibration states added
  for (auto const &idpair : map_camera_ids) {
    size_t cam_id = idpair.first;
    if (map_calib_cam2imu.find(cam_id) == map_calib_cam2imu.end()) {
      auto *var_calib_ori = new double[4];
      for (int j = 0; j < 4; j++) {
        var_calib_ori[j] = params.camera_extrinsics.at(cam_id)(0 + j, 0);
      }
      auto *var_calib_pos = new double[3];
      for (int j = 0; j < 3; j++) {
        var_calib_pos[j] = params.camera_extrinsics.at(cam_id)(4 + j, 0);
      }
      auto ceres_calib_jplquat = new State_JPLQuatLocal();
      problem.AddParameterBlock(var_calib_ori, 4, ceres_calib_jplquat);
      problem.AddParameterBlock(var_calib_pos, 3);
      map_calib_cam2imu.insert({cam_id, (int)ceres_vars_calib_cam2imu_ori.size()});
      ceres_vars_calib_cam2imu_ori.push_back(var_calib_ori);
      ceres_vars_calib_cam2imu_pos.push_back(var_calib_pos);

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(7, 1);
      for (int j = 0; j < 4; j++) {
        x_lin(0 + j) = var_calib_ori[j];
      }
      for (int j = 0; j < 3; j++) {
        x_lin(4 + j) = var_calib_pos[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(6, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(6, 6);
      prior_Info.block(0, 0, 3, 3) *= 1.0 / std::pow(0.001, 2);
      prior_Info.block(3, 3, 3, 3) *= 1.0 / std::pow(0.01, 2);

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_calib_ori);
      x_types.emplace_back("quat");
      factor_params.push_back(var_calib_pos);
      x_types.emplace_back("vec3");
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
      if (!params.init_dyn_mle_opt_calib) {
        problem.SetParameterBlockConstant(var_calib_ori);
        problem.SetParameterBlockConstant(var_calib_pos);
      }
    }
    if (map_calib_cam.find(cam_id) == map_calib_cam.end()) {
      auto *var_calib_cam = new double[8];
      for (int j = 0; j < 8; j++) {
        var_calib_cam[j] = params.camera_intrinsics.at(cam_id)->get_value()(j, 0);
      }
      problem.AddParameterBlock(var_calib_cam, 8);
      map_calib_cam.insert({cam_id, (int)ceres_vars_calib_cam_intrinsics.size()});
      ceres_vars_calib_cam_intrinsics.push_back(var_calib_cam);

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(8, 1);
      for (int j = 0; j < 8; j++) {
        x_lin(0 + j) = var_calib_cam[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(8, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(8, 8);
      prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1.0, 2);
      prior_Info.block(4, 4, 4, 4) *= 1.0 / std::pow(0.005, 2);

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_calib_cam);
      x_types.emplace_back("vec8");
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
      if (!params.init_dyn_mle_opt_calib) {
        problem.SetParameterBlockConstant(var_calib_cam);
      }
    }
  }
  assert(map_calib_cam2imu.size() == map_calib_cam.size());

  // Then, append new feature observations factors seen from all cameras
  for (auto const &feat : features) {
    // Skip features that don't have enough measurements
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;
    // Features can be removed if behind the camera!
    if (features_inG.find(feat.first) == features_inG.end())
      continue;
    // Finally loop through each raw uv observation and append it as a factor
    for (auto const &camtime : feat.second->timestamps) {

      // Get our ids and if the camera is a fisheye or not
      size_t feat_id = feat.first;
      size_t cam_id = camtime.first;
      bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(params.camera_intrinsics.at(cam_id)) != nullptr);

      // Loop through each observation
      for (size_t i = 0; i < camtime.second.size(); i++) {

        // Skip measurements we don't have poses for
        double time = feat.second->timestamps.at(cam_id).at(i);
        if (map_camera_times.find(time) == map_camera_times.end())
          continue;

        // Our measurement
        Eigen::Vector2d uv_raw = feat.second->uvs.at(cam_id).at(i).block(0, 0, 2, 1).cast<double>();

        // If we don't have the feature state we should create that parameter block
        // The initial guess of the features are the scaled feature map from the SFM
        if (map_features.find(feat_id) == map_features.end()) {
          auto *var_feat = new double[3];
          for (int j = 0; j < 3; j++) {
            var_feat[j] = features_inG.at(feat_id)(j);
          }
          problem.AddParameterBlock(var_feat, 3);
          map_features.insert({feat_id, (int)ceres_vars_feat.size()});
          ceres_vars_feat.push_back(var_feat);
        }

        // Then lets add the factors
        std::vector<double *> factor_params;
        factor_params.push_back(ceres_vars_ori.at(map_states.at(time)));
        factor_params.push_back(ceres_vars_pos.at(map_states.at(time)));
        factor_params.push_back(ceres_vars_feat.at(map_features.at(feat_id)));
        factor_params.push_back(ceres_vars_calib_cam2imu_ori.at(map_calib_cam2imu.at(cam_id)));
        factor_params.push_back(ceres_vars_calib_cam2imu_pos.at(map_calib_cam2imu.at(cam_id)));
        factor_params.push_back(ceres_vars_calib_cam_intrinsics.at(map_calib_cam.at(cam_id)));
        auto *factor_pinhole = new Factor_ImageReprojCalib(uv_raw, params.sigma_pix, is_fisheye);
        // ceres::LossFunction *loss_function = nullptr;
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
        problem.AddResidualBlock(factor_pinhole, loss_function, factor_params);
      }
    }
  }
  assert(ceres_vars_ori.size() == ceres_vars_bias_g.size());
  assert(ceres_vars_ori.size() == ceres_vars_vel.size());
  assert(ceres_vars_ori.size() == ceres_vars_bias_a.size());
  assert(ceres_vars_ori.size() == ceres_vars_pos.size());
  auto rT5 = boost::posix_time::microsec_clock::local_time();

  // Optimize the ceres graph
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  PRINT_INFO("[init-d]: %d iterations | %zu states, %zu feats (%zu valid) | %d param and %d res | cost %.4e => %.4e\n",
             (int)summary.iterations.size(), map_states.size(), map_features.size(), count_valid_features, summary.num_parameters,
             summary.num_residuals, summary.initial_cost, summary.final_cost);
  auto rT6 = boost::posix_time::microsec_clock::local_time();

  // Return if we have failed!
  timestamp = newest_cam_time;
  if (params.init_dyn_mle_max_iter != 0 && summary.termination_type != ceres::CONVERGENCE) {
    PRINT_WARNING(YELLOW "[init-d]: opt failed: %s!\n" RESET, summary.message.c_str());
    free_state_memory();
    return false;
  }
  PRINT_DEBUG("[init-d]: %s\n", summary.message.c_str());

  //======================================================
  //======================================================

  // Helper function to get the IMU pose value from our ceres problem
  auto get_pose = [&](double timestamp) {
    Eigen::VectorXd state_imu = Eigen::VectorXd::Zero(16);
    for (int i = 0; i < 4; i++) {
      state_imu(0 + i) = ceres_vars_ori[map_states[timestamp]][i];
    }
    for (int i = 0; i < 3; i++) {
      state_imu(4 + i) = ceres_vars_pos[map_states[timestamp]][i];
      state_imu(7 + i) = ceres_vars_vel[map_states[timestamp]][i];
      state_imu(10 + i) = ceres_vars_bias_g[map_states[timestamp]][i];
      state_imu(13 + i) = ceres_vars_bias_a[map_states[timestamp]][i];
    }
    return state_imu;
  };

  // Our most recent state is the IMU state!
  assert(map_states.find(newest_cam_time) != map_states.end());
  if (_imu == nullptr) {
    _imu = std::make_shared<ov_type::IMU>();
  }
  Eigen::VectorXd imu_state = get_pose(newest_cam_time);
  _imu->set_value(imu_state);
  _imu->set_fej(imu_state);

  // Append our IMU clones (includes most recent)
  for (auto const &statepair : map_states) {
    Eigen::VectorXd pose = get_pose(statepair.first);
    if (_clones_IMU.find(statepair.first) == _clones_IMU.end()) {
      auto _pose = std::make_shared<ov_type::PoseJPL>();
      _pose->set_value(pose.block(0, 0, 7, 1));
      _pose->set_fej(pose.block(0, 0, 7, 1));
      _clones_IMU.insert({statepair.first, _pose});
    } else {
      _clones_IMU.at(statepair.first)->set_value(pose.block(0, 0, 7, 1));
      _clones_IMU.at(statepair.first)->set_fej(pose.block(0, 0, 7, 1));
    }
  }

  // Append features as SLAM features!
  for (auto const &featpair : map_features) {
    Eigen::Vector3d feature;
    feature << ceres_vars_feat[featpair.second][0], ceres_vars_feat[featpair.second][1], ceres_vars_feat[featpair.second][2];
    if (_features_SLAM.find(featpair.first) == _features_SLAM.end()) {
      auto _feature = std::make_shared<ov_type::Landmark>(3);
      _feature->_featid = featpair.first;
      _feature->_feat_representation = LandmarkRepresentation::Representation::GLOBAL_3D;
      _feature->set_from_xyz(feature, false);
      _feature->set_from_xyz(feature, true);
      _features_SLAM.insert({featpair.first, _feature});
    } else {
      _features_SLAM.at(featpair.first)->_featid = featpair.first;
      _features_SLAM.at(featpair.first)->_feat_representation = LandmarkRepresentation::Representation::GLOBAL_3D;
      _features_SLAM.at(featpair.first)->set_from_xyz(feature, false);
      _features_SLAM.at(featpair.first)->set_from_xyz(feature, true);
    }
  }

  // If we optimized calibration, we should also save it to our state
  if (params.init_dyn_mle_opt_calib) {
    // TODO: append our calibration states too if we are doing calibration!
    // TODO: (if we are not doing calibration do not calibrate them....)
    // TODO: std::shared_ptr<ov_type::Vec> _calib_dt_CAMtoIMU,
    // TODO: std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> &_calib_IMUtoCAM,
    // TODO: std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> &_cam_intrinsics
  }

  // Recover the covariance here of the optimized states
  // NOTE: for now just the IMU state is recovered, but we should be able to do everything
  // NOTE: maybe having features / clones will make it more stable?
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  int state_index = map_states[newest_cam_time];
  // diagonals
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_ori[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_pos[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index]));
  // orientation
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_pos[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index]));
  // position
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index]));
  // velocity
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index]));
  // bias_g
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index]));

  // Finally, compute the covariance
  ceres::Covariance::Options options_cov;
  options_cov.null_space_rank = (!params.init_dyn_mle_opt_calib) * ((int)map_calib_cam2imu.size() * (6 + 8));
  options_cov.min_reciprocal_condition_number = params.init_dyn_min_rec_cond;
  // options_cov.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
  options_cov.apply_loss_function = true; // Better consistency if we use this
  options_cov.num_threads = params.init_dyn_mle_max_threads;
  ceres::Covariance problem_cov(options_cov);
  bool success = problem_cov.Compute(covariance_blocks, &problem);
  if (!success) {
    PRINT_WARNING(YELLOW "[init-d]: covariance recovery failed...\n" RESET);
    free_state_memory();
    return false;
  }

  // construct the covariance we will return
  order.clear();
  order.push_back(_imu);
  covariance = Eigen::MatrixXd::Zero(_imu->size(), _imu->size());
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covtmp = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

  // block diagonal
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_ori[state_index], covtmp.data()));
  covariance.block(0, 0, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_pos[state_index], covtmp.data()));
  covariance.block(3, 3, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(6, 6, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(9, 9, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(12, 12, 3, 3) = covtmp.eval();

  // orientation
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_pos[state_index], covtmp.data()));
  covariance.block(0, 3, 3, 3) = covtmp.eval();
  covariance.block(3, 0, 3, 3) = covtmp.transpose();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(0, 6, 3, 3) = covtmp.eval();
  covariance.block(6, 0, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(0, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 0, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(0, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 0, 3, 3) = covtmp.transpose().eval();

  // position
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(3, 6, 3, 3) = covtmp.eval();
  covariance.block(6, 3, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(3, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 3, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(3, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 3, 3, 3) = covtmp.transpose().eval();

  // velocity
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(6, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 6, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(6, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 6, 3, 3) = covtmp.transpose().eval();

  // bias_g
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(9, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 9, 3, 3) = covtmp.transpose().eval();

  // inflate as needed
  covariance.block(0, 0, 3, 3) *= params.init_dyn_inflation_orientation;
  covariance.block(6, 6, 3, 3) *= params.init_dyn_inflation_velocity;
  covariance.block(9, 9, 3, 3) *= params.init_dyn_inflation_bias_gyro;
  covariance.block(12, 12, 3, 3) *= params.init_dyn_inflation_bias_accel;

  // we are done >:D
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::Vector3d sigmas_vel = covariance.block(6, 6, 3, 3).diagonal().transpose().cwiseSqrt();
  Eigen::Vector3d sigmas_bg = covariance.block(9, 9, 3, 3).diagonal().transpose().cwiseSqrt();
  Eigen::Vector3d sigmas_ba = covariance.block(12, 12, 3, 3).diagonal().transpose().cwiseSqrt();
  PRINT_DEBUG("[init-d]: vel priors = %.3f, %.3f, %.3f\n", sigmas_vel(0), sigmas_vel(1), sigmas_vel(2));
  PRINT_DEBUG("[init-d]: bg priors = %.3f, %.3f, %.3f\n", sigmas_bg(0), sigmas_bg(1), sigmas_bg(2));
  PRINT_DEBUG("[init-d]: ba priors = %.3f, %.3f, %.3f\n", sigmas_ba(0), sigmas_ba(1), sigmas_ba(2));

  // Set our position to be zero
  Eigen::MatrixXd x = _imu->value();
  x.block(4, 0, 3, 1).setZero();
  _imu->set_value(x);
  _imu->set_fej(x);

  // Debug timing information about how long it took to initialize!!
  auto rT7 = boost::posix_time::microsec_clock::local_time();
  PRINT_DEBUG("[TIME]: %.4f sec for prelim tests\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec for linsys setup\n", (rT3 - rT2).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec for linsys\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec for ceres opt setup\n", (rT5 - rT4).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec for ceres opt\n", (rT6 - rT5).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec for ceres covariance\n", (rT7 - rT6).total_microseconds() * 1e-6);
  PRINT_DEBUG("[TIME]: %.4f sec total for initialization\n", (rT7 - rT1).total_microseconds() * 1e-6);
  free_state_memory();
  return true;
}
