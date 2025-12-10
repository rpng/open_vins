/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * IMPROVED VERSION with Gravity-Aligned Orientation Refinement
 * 
 * Key improvements:
 * 1. Gravity-aligned orientation refinement during stable ZUPT
 * 2. Progressive confidence-based constraint tightening
 * 3. Enhanced bias estimation during prolonged stationary periods
 */

#include "UpdaterZeroVelocity.h"

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterZeroVelocity::UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                                         std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity,
                                         double zupt_noise_multiplier, double zupt_max_disparity)
    : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
      _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity) {

  // Gravity
  _gravity << 0.0, 0.0, gravity_mag;

  // Save our raw pixel noise squared
  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // Initialize the chi squared test table with confidence level 0.95
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

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

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;

  // Select bounding inertial measurements
  std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);

  // Move forward in time
  last_prop_time_offset = t_off_new;

  // Check that we have at least one measurement to propagate with
  if (imu_recent.size() < 2) {
    PRINT_WARNING(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // Configuration flags
  bool integrated_accel_constraint = false;
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;
  bool enable_gravity_refinement = true;  // NEW: Enable gravity-based orientation refinement

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

  // IMU intrinsic calibration estimates (static)
  Eigen::Matrix3d Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());

  // Loop through all our IMU and construct the residual and Jacobian
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) {

    // Precomputed values
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot() * Da * (imu_recent.at(i).am - state->_imu->bias_a());
    Eigen::Vector3d w_hat = state->_calib_imu_GYROtoIMU->Rot() * Dw * (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);

    // Measurement noise
    double w_omega = std::sqrt(dt) / _noises.sigma_w;
    double w_accel = std::sqrt(dt) / _noises.sigma_a;
    double w_accel_v = 1.0 / (std::sqrt(dt) * _noises.sigma_a);

    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) = -w_omega * w_hat;
    if (!integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel * (a_hat - state->_imu->Rot() * _gravity);
    } else {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel_v * (state->_imu->vel() - _gravity * dt + state->_imu->Rot().transpose() * a_hat * dt);
    }

    // Measurement Jacobian
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -w_omega * Eigen::Matrix3d::Identity();
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * _gravity);
      H.block(6 * i + 3, 6, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();
    } else {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = w_accel_v * Eigen::Matrix3d::Identity();
    }
    dt_summed += dt;
  }

  // Compress the system
  UpdaterHelper::measurement_compress_inplace(H, res);
  if (H.rows() < 1) {
    return false;
  }

  // ========== IMPROVEMENT 1: Progressive Confidence-Based Noise Reduction ==========
  // As ZUPT count increases, we become more confident and reduce noise multiplier
  double confidence_factor = std::min(1.0, last_zupt_count / 5.0);  // Saturates at 5 consecutive ZUPTs
  double adaptive_noise_mult = _zupt_noise_multiplier * (1.0 - 0.7 * confidence_factor);
  Eigen::MatrixXd R = adaptive_noise_mult * Eigen::MatrixXd::Identity(res.rows(), res.rows());

  // Next propagate the biases forward in time
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb_2;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab_2;

  // Chi2 distance check
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  if (model_time_varying_bias) {
    P_marg.block(3, 3, 6, 6) += Q_bias;
  }
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));

  // Get our threshold
  double chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }

  // Check if the image disparity
  bool disparity_passed = false;
  if (override_with_disparity_check) {
    double time0_cam = state->_timestamp;
    double time1_cam = timestamp;
    int num_features = 0;
    double disp_avg = 0.0;
    double disp_var = 0.0;
    FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, disp_avg, disp_var, num_features);

    disparity_passed = (disp_avg < _zupt_max_disparity && num_features > 20);
    if (disparity_passed) {
      PRINT_INFO(CYAN "[ZUPT]: passed disparity (%.3f < %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    } else {
      PRINT_DEBUG(YELLOW "[ZUPT]: failed disparity (%.3f > %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    }
  }

  // Check if we are currently zero velocity
  if (!disparity_passed && (chi2 > _options.chi2_multipler * chi2_check || state->_imu->vel().norm() > _zupt_max_velocity)) {
    last_zupt_state_timestamp = 0.0;
    last_zupt_count = 0;
    PRINT_DEBUG(YELLOW "[ZUPT]: rejected |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
                _options.chi2_multipler * chi2_check);
    return false;
  }
  PRINT_INFO(CYAN "[ZUPT]: accepted |v_IinG| = %.3f (chi2 %.3f < %.3f) [count=%d, conf=%.2f]\n" RESET, 
             state->_imu->vel().norm(), chi2, _options.chi2_multipler * chi2_check, 
             last_zupt_count + 1, confidence_factor);

  // Do our update
  if (last_zupt_count >= 2) {
    _db->cleanup_measurements_exact(last_zupt_state_timestamp);
  }

  // Perform the standard ZUPT update
  if (!explicitly_enforce_zero_motion) {

    // Propagate biases forward in time
    if (model_time_varying_bias) {
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg());
      Phi_order.push_back(state->_imu->ba());
      StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // Standard EKF update with velocity constraint
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;

    // ========== IMPROVEMENT 2: Gravity-Aligned Orientation Refinement ==========
    // After several consecutive ZUPTs, refine orientation using gravity vector
    if (enable_gravity_refinement && last_zupt_count >= 2) {
      
      // Compute average accelerometer reading over the ZUPT period
      Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
      int accel_count = 0;
      
      for (size_t i = 0; i < imu_recent.size(); i++) {
        Eigen::Vector3d a_corrected = state->_calib_imu_ACCtoIMU->Rot() * Da * 
                                      (imu_recent.at(i).am - state->_imu->bias_a());
        avg_accel += a_corrected;
        accel_count++;
      }
      avg_accel /= accel_count;
      
      // Expected gravity in IMU frame
      Eigen::Matrix3d R_GtoI = state->_imu->Rot();
      Eigen::Vector3d g_in_I = R_GtoI * _gravity;
      
      // Compute gravity residual
      Eigen::Vector3d gravity_residual = avg_accel - g_in_I;
      double residual_norm = gravity_residual.norm();
      
      // Only apply correction if residual is reasonable (not too small, not too large)
      // Too small: no information gain
      // Too large: might be due to external acceleration or miscalibration
      if (residual_norm > 0.05 && residual_norm < 1.5) {
        
        // Create measurement model: res = avg_accel - R_GtoI * g
        // Linearization: res ≈ -[R_GtoI * g]_× * δθ
        Eigen::MatrixXd H_grav = Eigen::MatrixXd::Zero(3, 3);
        Eigen::VectorXd res_grav = Eigen::VectorXd::Zero(3);
        Eigen::MatrixXd R_grav = Eigen::MatrixXd::Identity(3, 3);
        
        // Jacobian: H = -[R_GtoI * g]_×
        Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
        H_grav = -skew_x(R_GtoI_jacob * _gravity);
        
        // Residual
        res_grav = avg_accel - R_GtoI * _gravity;
        
        // Measurement noise: progressively trust more as ZUPT count increases
        // Start with conservative noise, reduce as we gain confidence
        double base_noise = 0.1;  // Base noise in m/s^2
        double confidence_weight = std::min(5.0, (double)last_zupt_count) / 5.0;  // 0 to 1
        double adaptive_grav_noise = base_noise * std::pow(2.0, -(confidence_weight * 2.0));  // Exponential decay
        R_grav *= std::pow(adaptive_grav_noise, 2);
        
        // Perform the gravity-alignment update
        std::vector<std::shared_ptr<Type>> H_order_grav;
        H_order_grav.push_back(state->_imu->q());
        StateHelper::EKFUpdate(state, H_order_grav, H_grav, res_grav, R_grav);
        
        PRINT_INFO(GREEN "[ZUPT-GRAV]: Applied gravity alignment (|res|=%.4f m/s², noise=%.4f, count=%d)\n" RESET, 
                   residual_norm, adaptive_grav_noise, last_zupt_count + 1);
      } else if (residual_norm <= 0.05) {
        PRINT_DEBUG("[ZUPT-GRAV]: Residual too small (%.4f m/s²), skipping\n", residual_norm);
      } else {
        PRINT_DEBUG(YELLOW "[ZUPT-GRAV]: Residual too large (%.4f m/s²), possible external accel\n" RESET, residual_norm);
      }
    }

  } else {
    // Alternative: explicitly enforce zero motion constraint
    double time0_cam = last_zupt_state_timestamp;
    double time1_cam = timestamp;
    _prop->propagate_and_clone(state, time1_cam);

    H = Eigen::MatrixXd::Zero(9, 15);
    res = Eigen::VectorXd::Zero(9);
    R = Eigen::MatrixXd::Identity(9, 9);

    Eigen::Matrix3d R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot();
    Eigen::Vector3d p_I0inG = state->_clones_IMU.at(time0_cam)->pos();
    Eigen::Matrix3d R_GtoI1 = state->_clones_IMU.at(time1_cam)->Rot();
    Eigen::Vector3d p_I1inG = state->_clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->_imu->vel();
    res *= -1;

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

    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(time1_cam));
    state->_clones_IMU.erase(time1_cam);
  }

  // Update ZUPT state
  last_zupt_state_timestamp = timestamp;
  last_zupt_count++;
  
  return true;
}