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

#include "UpdaterZeroVelocity.h"

using namespace ov_msckf;

bool UpdaterZeroVelocity::try_update(std::shared_ptr<State> state, double timestamp) {

  // Return if we don't have any imu data yet
  if (imu_data.empty()) {
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // Return if the state is already at the desired time
  if (state->_timestamp == timestamp) {
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  // assert that the time we are requesting is in the future
  // assert(timestamp > state->_timestamp);

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  // double time0 = state->_timestamp+t_off_new;
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;

  // Select bounding inertial measurements
  std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);

  // Move forward in time
  last_prop_time_offset = t_off_new;

  // Check that we have at least one measurement to propagate with
  if (imu_recent.size() < 2) {
    printf(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // If we should integrate the acceleration and say the velocity should be zero
  // Also if we should still inflate the bias based on their random walk noises
  bool integrated_accel_constraint = false;
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  if (integrated_accel_constraint) {
    Hx_order.push_back(state->_imu->v());
  }

  // Large final matrices used for update
  int h_size = (integrated_accel_constraint) ? 12 : 9;
  int m_size = 6 * ((int)imu_recent.size() - 1);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size, m_size);

  // Loop through all our IMU and construct the residual and Jacobian
  // State order is: [q_GtoI, bg, ba, v_IinG]
  // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) {

    // Precomputed values
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Eigen::Vector3d a_hat = imu_recent.at(i).am - state->_imu->bias_a();

    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) = -(imu_recent.at(i).wm - state->_imu->bias_g());
    if (!integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) = -(a_hat - state->_imu->Rot() * _gravity);
    } else {
      res.block(6 * i + 3, 0, 3, 1) = -(state->_imu->vel() - _gravity * dt + state->_imu->Rot().transpose() * a_hat * dt);
    }

    // Measurement Jacobian
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -skew_x(R_GtoI_jacob * _gravity);
      H.block(6 * i + 3, 6, 3, 3) = -Eigen::Matrix3d::Identity();
    } else {
      H.block(6 * i + 3, 0, 3, 3) = -R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    }

    // Measurement noise (convert from continuous to discrete)
    // Note the dt time might be different if we have "cut" any imu measurements
    R.block(6 * i + 0, 6 * i + 0, 3, 3) *= _noises.sigma_w_2 / dt;
    if (!integrated_accel_constraint) {
      R.block(6 * i + 3, 6 * i + 3, 3, 3) *= _noises.sigma_a_2 / dt;
    } else {
      R.block(6 * i + 3, 6 * i + 3, 3, 3) *= _noises.sigma_a_2 * dt;
    }
    dt_summed += dt;
  }

  // Multiply our noise matrix by a fixed amount
  // We typically need to treat the IMU as being "worst" to detect / not become over confident
  R *= _zupt_noise_multiplier;

  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab;

  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if this was to be accepted
  // NOTE: we don't propagate first since if we fail the chi2 then we just want to return and do normal logic
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  P_marg.block(3, 3, 6, 6) += Q_bias;
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));

  // Get our threshold (we precompute up to 1000 but handle the case that it is more)
  double chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    printf(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }

  // Check if the image disparity
  bool disparity_passed = false;
  if (override_with_disparity_check) {

    // Get features seen from this image, and the previous image
    double time0_cam = state->_timestamp;
    double time1_cam = timestamp;
    std::vector<std::shared_ptr<Feature>> feats0 = _db->features_containing(time0_cam, false, true);

    // Compute the disparity
    double average_disparity = 0.0;
    double num_features = 0.0;
    for (auto &feat : feats0) {

      // Get the two uvs for both times
      for (auto &campairs : feat->timestamps) {

        // First find the two timestamps
        size_t camid = campairs.first;
        auto it0 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time0_cam);
        auto it1 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time1_cam);
        if (it0 == feat->timestamps.at(camid).end() || it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        // Now lets calculate the disparity
        Eigen::Vector2f uv0 = feat->uvs.at(camid).at(idx0).block(0, 0, 2, 1);
        Eigen::Vector2f uv1 = feat->uvs.at(camid).at(idx1).block(0, 0, 2, 1);
        average_disparity += (uv1 - uv0).norm();
        num_features += 1;
      }
    }

    // Now check if we have passed the threshold
    if (num_features > 0) {
      average_disparity /= num_features;
    }
    disparity_passed = (average_disparity < _zupt_max_disparity && num_features > 20);
    if (disparity_passed) {
      printf(CYAN "[ZUPT]: passed disparity (%.3f < %.3f, %d features)\n" RESET, average_disparity, _zupt_max_disparity, (int)num_features);
    } else {
      printf(YELLOW "[ZUPT]: failed disparity (%.3f > %.3f, %d features)\n" RESET, average_disparity, _zupt_max_disparity,
             (int)num_features);
    }
  }

  // Check if we are currently zero velocity
  // We need to pass the chi2 and not be above our velocity threshold
  if (!disparity_passed && (chi2 > _options.chi2_multipler * chi2_check || state->_imu->vel().norm() > _zupt_max_velocity)) {
    last_zupt_state_timestamp = 0.0;
    printf(YELLOW "[ZUPT]: rejected |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
           _options.chi2_multipler * chi2_check);
    return false;
  }
  printf(CYAN "[ZUPT]: accepted |v_IinG| = %.3f (chi2 %.3f < %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
         _options.chi2_multipler * chi2_check);

  // Do our update, only do this update if we have previously detected
  // If we have succeeded, then we should remove the current timestamp feature tracks
  // This is because we will not clone at this timestep and instead do our zero velocity update
  // We want to keep the tracks from the previous timestep, thus only delete measurements from the current timestep
  if (last_zupt_state_timestamp > 0.0) {
    _db->cleanup_measurements_exact(last_zupt_state_timestamp);
  }

  // Else we are good, update the system
  // 1) update with our IMU measurements directly
  // 2) propagate and then explicitly say that our ori, pos, and vel should be zero
  if (!explicitly_enforce_zero_motion) {

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
    if (model_time_varying_bias) {
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg());
      Phi_order.push_back(state->_imu->ba());
      StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // Finally move the state time forward
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;

  } else {

    // Propagate the state forward in time
    double time0_cam = last_zupt_state_timestamp;
    double time1_cam = timestamp;
    _prop->propagate_and_clone(state, time1_cam);

    // Create the update system!
    H = Eigen::MatrixXd::Zero(9, 15);
    res = Eigen::VectorXd::Zero(9);
    R = Eigen::MatrixXd::Identity(9, 9);

    // residual (order is ori, pos, vel)
    Eigen::Matrix3d R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot();
    Eigen::Vector3d p_I0inG = state->_clones_IMU.at(time0_cam)->pos();
    Eigen::Matrix3d R_GtoI1 = state->_clones_IMU.at(time1_cam)->Rot();
    Eigen::Vector3d p_I1inG = state->_clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->_imu->vel();
    res *= -1;

    // jacobian (order is q0, p0, q1, p1, v0)
    Hx_order.clear();
    Hx_order.push_back(state->_clones_IMU.at(time0_cam));
    Hx_order.push_back(state->_clones_IMU.at(time1_cam));
    Hx_order.push_back(state->_imu->v());
    if (state->_options.do_fej) {
      R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot_fej();
    }
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, 6, 3, 3) = -R_GtoI0;
    H.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    H.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(6, 12, 3, 3) = Eigen::Matrix3d::Identity();

    // noise (order is ori, pos, vel)
    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    // finally update and remove the old clone
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(time1_cam));
    state->_clones_IMU.erase(time1_cam);
  }

  // Finally return
  last_zupt_state_timestamp = timestamp;
  return true;

}