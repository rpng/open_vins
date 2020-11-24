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



#include <csignal>

#include "core/CalibManager.h"

#include "sim/Simulator.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"

#include <ros/ros.h>
#include "core/RosVisualizer.h"
#include "utils/parse_ros.h"

#include <cmath>

using namespace ov_msckf;


Simulator* sim;
CalibManager* sys;
RosVisualizer* viz;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}

// Main function
int main(int argc, char** argv)
{
    // Read in our parameters
    VioManagerOptions params;
    ros::init(argc, argv, "run_calib");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);

    // Create our VIO system
    sim = new Simulator(params);
    sys = new CalibManager(params);
    // viz = new RosVisualizer(nh, sys, sim);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Get initial state
    Eigen::Matrix<double, 17, 1> imustate;
    bool success = sim->get_state(sim->current_timestamp(),imustate);
    if(!success) {
        printf(RED "[SIM]: Could not initialize the filter to the first state\n" RESET);
        printf(RED "[SIM]: Did the simulator load properly???\n" RESET);
        std::exit(EXIT_FAILURE);
    }

    // Since the state time is in the camera frame of reference
    // Subtract out the imu to camera time offset
    imustate(0,0) -= sim->get_true_paramters().calib_camimu_dt;

    // Initialize our filter with the groundtruth
    sys->initialize_with_gt(imustate);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Buffer our camera image
    double buffer_timecam = -1;
    std::vector<int> buffer_camids;
    std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> buffer_feats;

    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);

    int count = 0;
    Eigen::Matrix3d R_GtoI, R_GtoC, R_CtoI;
    R_CtoI << 0,-1, 0,
              1, 0, 0,
              0, 0, 1;

    while(sim->ok() && ros::ok()) {
        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim->get_next_imu(time_imu, wm, am);
        if(hasimu) {
            sys->feed_measurement_imu(time_imu, wm, am);
            // viz->visualize_odometry(time_imu);
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim->get_next_cam(time_cam, camids, feats);

        if(hascam) {
            if(buffer_timecam != -1) {
                sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
                // viz->visualize();
                
                count ++;
                if(count > 10) {
                    /*
                    // get relative rotation between the camera and IMU
                    Eigen::Matrix<double, 4, 1> q_IMUtoCAM = sys->get_state()->_calib_IMUtoCAM.at(0)->quat();
                    Eigen::Quaternion<double> q_ItoC;
                    q_ItoC.x() = q_IMUtoCAM(0); q_ItoC.y() = q_IMUtoCAM(1); q_ItoC.z() = q_IMUtoCAM(2); q_ItoC.w() = q_IMUtoCAM(3);
                    Eigen::Matrix3d R_ItoC = q_ItoC.toRotationMatrix();

                    // get ground truth angular velocity
                    Eigen::Matrix3d R_GtoI;
                    Eigen::Vector3d p_IinG, w_IinI, v_IinG;
                    bool success_vel = sim->get_spline()->get_velocity(buffer_timecam, R_GtoI, p_IinG, w_IinI, v_IinG);
                    Eigen::Matrix<double, 3, 1> wg_mat; wg_mat << w_IinI(0), w_IinI(1), w_IinI(2);

                    // get camera angular velocity
                    Eigen::Vector3d wc, ac;
                    Eigen::Matrix3d R_PrevtoCurr_c;
                    sys->get_estimation_camera(wc, ac, R_PrevtoCurr_c);
                    
                    // print results
                    std::cout << "w_gt : " << (R_ItoC * wg_mat).transpose() << " (norm: " << wg_mat.norm() << ")" << std::endl;
                    std::cout << "w_cam: " << wc.transpose() << " (norm: " << wc.norm() << ")" << std::endl;
                    */

                    // get ground truth rotation matrix
                    Eigen::Matrix3d R_GtoIprev, R_GtoIcurr, R_IprevtoIcurr;
                    Eigen::Vector3d p_IinG, w_IinI, v_IinG;
                    bool success_vel_prev = sim->get_spline()->get_velocity(buffer_timecam-0.1, R_GtoIprev, p_IinG, w_IinI, v_IinG);
                    bool success_vel_curr = sim->get_spline()->get_velocity(buffer_timecam, R_GtoIcurr, p_IinG, w_IinI, v_IinG);
                    R_IprevtoIcurr = R_GtoIprev.transpose() * R_GtoIcurr;

                    // get camera rotation matrix
                    Eigen::Vector3d wc, ac;
                    Eigen::Matrix3d R_CprevtoCcurr;
                    sys->get_estimation_camera(wc, ac, R_CprevtoCcurr);
                    
                    if (count == 11) {
                        R_GtoI = R_GtoIprev;
                        R_GtoC = R_GtoIprev;
                        // R_GtoC = R_GtoI * R_CtoI.transpose();
                    }
                    
                    R_GtoI = R_GtoI * R_IprevtoIcurr;
                    R_GtoC = R_GtoC * R_CprevtoCcurr;
                    
                    Eigen::Quaterniond q_GT(R_GtoIcurr);
                    Eigen::Quaterniond q_GtoI(R_GtoI);
                    Eigen::Quaterniond q_GtoC(R_GtoC);
                    double diff_GTtoI = q_GT.angularDistance(q_GtoI);   // should be the same
                    double diff_GTtoC = q_GT.angularDistance(q_GtoC);   // should be the same

                    std::cout << "q_GtoI: " << q_GtoI.coeffs().transpose() << ", Distance: " << diff_GTtoI << std::endl;
                    std::cout << "q_GtoC: " << q_GtoC.coeffs().transpose() << ", Distance: " << diff_GTtoC << std::endl;

                    
                }
            }
            buffer_timecam = time_cam;
            buffer_camids = camids;
            buffer_feats = feats;
        }
    

    }

    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Final visualization
    // viz->visualize_final();
    // delete viz;

    // Finally delete our system
    delete sim;
    delete sys;

    // Done!
    return EXIT_SUCCESS;

}














