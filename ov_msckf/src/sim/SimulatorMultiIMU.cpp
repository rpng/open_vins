/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
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
#include "SimulatorMultiIMU.h"

using namespace ov_msckf;


bool SimulatorMultiIMU::get_next_imu(double &time_imu, std::vector<int> &imuids, std::vector<Eigen::Vector3d> &wm, std::vector<Eigen::Vector3d> &am) {

    // Return if the camera measurement should go before us
    if(timestamp_last_cam+1.0/params.sim_freq_cam < timestamp_last_imu+1.0/params.sim_freq_imu)
        return false;

    // Else lets do a new measurement!!!
    timestamp_last_imu += 1.0/params.sim_freq_imu;
    timestamp = timestamp_last_imu;
    time_imu = timestamp_last_imu;

    // Current state values
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

    // Get the pose, velocity, and acceleration
    // NOTE: we get the acceleration between our two IMU
    // NOTE: this is because we are using a constant measurement model for integration
    //bool success_accel = spline.get_acceleration(timestamp+0.5/freq_imu, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
    bool success_accel = spline.get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

    // If failed, then that means we don't have any more spline
    // Thus we should stop the simulation
    if(!success_accel) {
        is_running = false;
        return false;
    }

    // Now add noise to these measurements
    double dt = 1.0/params.sim_freq_imu;
    std::normal_distribution<double> w(0,1);

    // Loop through each imu
    for(int imuid=0; imuid<params.num_imus; imuid++) {
        // Transform motion from base imu frame to current imu frame
        Eigen::Matrix<double,3,3> R_ItoU = quat_2_Rot(params.imu_extrinsics.at(imuid).block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_IinU = params.imu_extrinsics.at(imuid).block(4,0,3,1);
        
        Eigen::Matrix<double,3,1> p_UinI = - R_ItoU.transpose() * p_IinU;
        
        Eigen::Vector3d omega_inU = R_ItoU * w_IinI;    // rigid body's angular motion is uniform
        Eigen::Vector3d accel_inU = R_ItoU * R_GtoI * ((a_IinG+params.gravity) + alpha_IinI.cross(p_UinI) + w_IinI.cross(w_IinI.cross(p_UinI)));
        
        Eigen::Vector3d wm_curr, am_curr;

        wm_curr(0) = omega_inU(0) + true_bias_gyro[imuid](0) + params.imu_noises.sigma_w/std::sqrt(dt)*w(gen_meas_imu);
        wm_curr(1) = omega_inU(1) + true_bias_gyro[imuid](1) + params.imu_noises.sigma_w/std::sqrt(dt)*w(gen_meas_imu);
        wm_curr(2) = omega_inU(2) + true_bias_gyro[imuid](2) + params.imu_noises.sigma_w/std::sqrt(dt)*w(gen_meas_imu);
        am_curr(0) = accel_inU(0) + true_bias_accel[imuid](0) + params.imu_noises.sigma_a/std::sqrt(dt)*w(gen_meas_imu);
        am_curr(1) = accel_inU(1) + true_bias_accel[imuid](1) + params.imu_noises.sigma_a/std::sqrt(dt)*w(gen_meas_imu);
        am_curr(2) = accel_inU(2) + true_bias_accel[imuid](2) + params.imu_noises.sigma_a/std::sqrt(dt)*w(gen_meas_imu);

        wm.push_back(wm_curr);
        am.push_back(am_curr);
        imuids.push_back(imuid);

        // Move the biases forward in time
        true_bias_gyro[imuid](0) += params.imu_noises.sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
        true_bias_gyro[imuid](1) += params.imu_noises.sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
        true_bias_gyro[imuid](2) += params.imu_noises.sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
        true_bias_accel[imuid](0) += params.imu_noises.sigma_ab*std::sqrt(dt)*w(gen_meas_imu);
        true_bias_accel[imuid](1) += params.imu_noises.sigma_ab*std::sqrt(dt)*w(gen_meas_imu);
        true_bias_accel[imuid](2) += params.imu_noises.sigma_ab*std::sqrt(dt)*w(gen_meas_imu);

        // Append the current true bias to our history
        hist_true_bias_time[imuid].push_back(timestamp_last_imu);
        hist_true_bias_gyro[imuid].push_back(true_bias_gyro[imuid]);
        hist_true_bias_accel[imuid].push_back(true_bias_accel[imuid]);
    }

    // Return success
    return true;

}
