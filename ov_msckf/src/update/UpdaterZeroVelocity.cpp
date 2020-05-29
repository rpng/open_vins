/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
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



bool UpdaterZeroVelocity::try_update(State *state, double timestamp) {

    // Return if we don't have any imu data yet
    if(imu_data.empty())
        return false;

    // Set the last time offset value if we have just started the system up
    if(!have_last_prop_time_offset) {
        last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
        have_last_prop_time_offset = true;
    }

    // assert that the time we are requesting is in the future
    assert(timestamp > state->_timestamp);

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

    // First lets construct an IMU vector of measurements we need
    //double time0 = state->_timestamp+t_off_new;
    double time0 = state->_timestamp+last_prop_time_offset;
    double time1 = timestamp+t_off_new;

    // Select bounding inertial measurements
    std::vector<Propagator::IMUDATA> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);

    // Move forward in time
    last_prop_time_offset = t_off_new;

    // Check that we have at least one measurement to propagate with
    if(imu_recent.empty() || imu_recent.size() < 2) {
        printf(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
        return false;
    }

    // If we should use velocity constraint
    bool velocity_constraint = false;

    // Order of our Jacobian
    std::vector<Type*> Hx_order;
    Hx_order.push_back(state->_imu->q());
    Hx_order.push_back(state->_imu->bg());
    Hx_order.push_back(state->_imu->ba());
    if(velocity_constraint) Hx_order.push_back(state->_imu->v());

    // Large final matrices used for update
    int h_size = (velocity_constraint)? 12 : 9;
    int m_size = (velocity_constraint)? 6*(imu_recent.size()-1)+3 : 6*(imu_recent.size()-1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size,h_size);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m_size,m_size);

    // Cached matrices that will be the same for all of them
    // Measurement order is: [w_true = 0, a_true = 0]
    // State order is: [q_GtoI, bg, ba]
    Eigen::Matrix<double,6,9> H_cached = Eigen::Matrix<double,6,9>::Zero();
    H_cached.block(0,3,3,3) = -Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej)? state->_imu->Rot_fej() : state->_imu->Rot();
    H_cached.block(3,0,3,3) = -skew_x(R_GtoI_jacob*_gravity);
    H_cached.block(3,6,3,3) = -Eigen::Matrix<double,3,3>::Identity();

    // Loop through all our IMU and construct the residual and Jacobian
    // w_true = w_m - bw - nw
    // a_true = a_m - ba - R*g - na
    for(size_t i=0; i<imu_recent.size()-1; i++) {

        // Measurement Jacobian
        H.block(6*i,0,6,9) = H_cached;

        // Measurement residual (true value is zero)
        res.block(6*i+0,0,3,1) = -(imu_recent.at(i).wm - state->_imu->bias_g());
        res.block(6*i+3,0,3,1) = -(imu_recent.at(i).am - state->_imu->bias_a() - state->_imu->Rot()*_gravity);

        // Measurement noise (convert from continuous to discrete)
        // Note the dt time might be different if we have "cut" any imu measurements
        double dt = imu_recent.at(i+1).timestamp - imu_recent.at(i).timestamp;
        R.block(6*i+0,6*i+0,3,3) *= _noises.sigma_w_2/dt;
        R.block(6*i+3,6*i+3,3,3) *= _noises.sigma_a_2/dt;

    }

    // If we are applying zero velocity, then add one more measurement
    // This says that our v_true = 0
    if(velocity_constraint) {
        H.block(H.rows()-3,9,3,3) = Eigen::Matrix<double,3,3>::Identity();
        res.block(res.rows()-3,0,3,1) = -(state->_imu->vel());
        R.block(R.rows()-3,R.rows()-3,3,3) *= std::pow(_zupt_max_velocity,3);
    }

    // Multiply our noise matrix by a fixed amount
    // We typically need to treat the IMU as being "worst" to detect / not become over confident
    R *= _zupt_noise_multiplier;

    /// Chi2 distance check
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H*P_marg*H.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 1000 but handle the case that it is more)
    double chi2_check;
    if(res.rows() < 1000) {
        chi2_check = chi_squared_table[res.rows()];
    } else {
        boost::math::chi_squared chi_squared_dist(res.rows());
        chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
        printf(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we are currently zero velocity
    // We need to pass the chi2 and not be above our velocity threshold
    if(chi2 > _options.chi2_multipler*chi2_check || state->_imu->vel().norm() > _zupt_max_velocity) {
        printf(YELLOW "[ZUPT]: rejected zero velocity |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET,state->_imu->vel().norm(),chi2,_options.chi2_multipler*chi2_check);
        return false;
    }

    // Else we are good, update the system
    // We can move our FEJ forward based on the idea that we are zero system dynamics
    // Thus our FEJ should move forward to the current timestep so the next propagation is correct
    printf(CYAN "[ZUPT]: accepted zero velocity |v_IinG| = %.3f (chi2 %.3f < %.3f)\n" RESET,state->_imu->vel().norm(),chi2,_options.chi2_multipler*chi2_check);
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;
    state->_imu->set_fej(state->_imu->value());
    return true;


}