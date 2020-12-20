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
#include "InertialInitializer.h"


using namespace ov_core;


void InertialInitializer::feed_imu(const ImuData &message) {

    // Append it to our vector
    imu_data.push_back(message);

    // Delete all measurements older than three of our initialization windows
    auto it0 = imu_data.begin();
    while(it0 != imu_data.end() && it0->timestamp < message.timestamp-3*_window_length) {
        it0 = imu_data.erase(it0);
    }

}





bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                                              Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG, bool wait_for_jerk) {

    // Return if we don't have any measurements
    if(imu_data.empty()) {
        return false;
    }

    // Newest imu timestamp
    double newesttime = imu_data.at(imu_data.size()-1).timestamp;

    // First lets collect a window of IMU readings from the newest measurement to the oldest
    std::vector<ImuData> window_newest, window_secondnew;
    for(const ImuData& data : imu_data) {
        if(data.timestamp > newesttime-1*_window_length && data.timestamp <= newesttime-0*_window_length) {
            window_newest.push_back(data);
        }
        if(data.timestamp > newesttime-2*_window_length && data.timestamp <= newesttime-1*_window_length) {
            window_secondnew.push_back(data);
        }
    }

    // Return if both of these failed
    if(window_newest.empty() || window_secondnew.empty()) {
        //printf(YELLOW "InertialInitializer::initialize_with_imu(): unable to select window of IMU readings, not enough readings\n" RESET);
        return false;
    }

    // Calculate the sample variance for the newest one
    Eigen::Matrix<double,3,1> a_avg = Eigen::Matrix<double,3,1>::Zero();
    for(const ImuData& data : window_newest) {
        a_avg += data.am;
    }
    a_avg /= (int)window_newest.size();
    double a_var = 0;
    for(const ImuData& data : window_newest) {
        a_var += (data.am-a_avg).dot(data.am-a_avg);
    }
    a_var = std::sqrt(a_var/((int)window_newest.size()-1));

    // If it is below the threshold and we want to wait till we detect a jerk
    if(a_var < _imu_excite_threshold && wait_for_jerk) {
        printf(YELLOW "InertialInitializer::initialize_with_imu(): no IMU excitation, below threshold %.4f < %.4f\n" RESET,a_var,_imu_excite_threshold);
        return false;
    }

    // Return if we don't have any measurements
    //if(imu_data.size() < 200) {
    //    return false;
    //}

    // Sum up our current accelerations and velocities
    Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
    for(size_t i=0; i<window_secondnew.size(); i++) {
        linsum += window_secondnew.at(i).am;
        angsum += window_secondnew.at(i).wm;
    }

    // Calculate the mean of the linear acceleration and angular velocity
    Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
    linavg = linsum/window_secondnew.size();
    angavg = angsum/window_secondnew.size();

    // Calculate variance of the
    double a_var2 = 0;
    for(const ImuData& data : window_secondnew) {
        a_var2 += (data.am-linavg).dot(data.am-linavg);
    }
    a_var2 = std::sqrt(a_var2/((int)window_secondnew.size()-1));

    // If it is above the threshold and we are not waiting for a jerk
    // Then we are not stationary (i.e. moving) so we should wait till we are
    if((a_var > _imu_excite_threshold || a_var2 > _imu_excite_threshold) && !wait_for_jerk) {
        printf(YELLOW "InertialInitializer::initialize_with_imu(): to much IMU excitation, above threshold %.4f,%.4f > %.4f\n" RESET,a_var,a_var2,_imu_excite_threshold);
        return false;
    }

    // Get z axis, which alines with -g (z_in_G=0,0,1)
    Eigen::Vector3d z_axis = linavg/linavg.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1,0,0);

    // Make x_axis perpendicular to z
    Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
    x_axis= x_axis/x_axis.norm();

    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

    // From these axes get rotation
    Eigen::Matrix<double,3,3> Ro;
    Ro.block(0,0,3,1) = x_axis;
    Ro.block(0,1,3,1) = y_axis;
    Ro.block(0,2,3,1) = z_axis;

    // Create our state variables
    Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro);

    // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
    Eigen::Matrix<double,3,1> bg = angavg;
    Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI)*_gravity;

    // Set our state variables
    time0 = window_secondnew.at(window_secondnew.size()-1).timestamp;
    q_GtoI0 = q_GtoI;
    b_w0 = bg;
    v_I0inG = Eigen::Matrix<double,3,1>::Zero();
    b_a0 = ba;
    p_I0inG = Eigen::Matrix<double,3,1>::Zero();

    // Done!!!
    return true;

}




