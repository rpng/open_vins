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
#include <cmath>
#include <vector>
#include <deque>
#include <sstream>
#include <unistd.h>
#include <csignal>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#ifdef ROS_AVAILABLE
#include <ros/ros.h>
#endif

#include "sim/Simulator.h"
#include "core/VioManagerOptions.h"
#include "utils/parse_cmd.h"
#include "utils/parse_ros.h"


using namespace ov_msckf;


// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}


// Main function
int main(int argc, char** argv)
{

    // Register failure handler
    signal(SIGINT, signal_callback_handler);

    //===================================================
    //===================================================

    // Create the simulator
    VioManagerOptions params1;
#ifdef ROS_AVAILABLE
    ros::init(argc, argv, "test_sim_repeat");
    ros::NodeHandle nh1("~");
    params1 = parse_ros_nodehandler(nh1);
#else
    params1 = parse_command_line_arguments(argc, argv);
#endif
    Simulator sim1(params1);

    // Vector of stored measurements
    std::vector<double> vec_imutime;
    std::vector<double> vec_camtime;
    std::vector<Eigen::Vector3d> vec_am;
    std::vector<Eigen::Vector3d> vec_wm;
    std::vector<std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>>> vec_feats;

    // Continue to simulate until we have processed all the measurements
    while(sim1.ok()) {

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim1.get_next_imu(time_imu, wm, am);
        if(hasimu) {
            vec_imutime.push_back(time_imu);
            vec_am.push_back(am);
            vec_wm.push_back(wm);
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim1.get_next_cam(time_cam, camids, feats);
        if(hascam) {
            vec_camtime.push_back(time_cam);
            vec_feats.push_back(feats);
        }

    }


    //===================================================
    //===================================================


    // Create the simulator
    VioManagerOptions params2;
#ifdef ROS_AVAILABLE
    ros::NodeHandle nh2("~");
    params2 = parse_ros_nodehandler(nh2);
#else
    params2 = parse_command_line_arguments(argc, argv);
#endif
    Simulator sim2(params2);
    size_t ct_imu = 0;
    size_t ct_cam = 0;

    // Continue to simulate until we have processed all the measurements
    while(sim2.ok()) {

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim2.get_next_imu(time_imu, wm, am);
        if(hasimu) {
            assert(time_imu==vec_imutime.at(ct_imu));
            assert(wm(0)==vec_wm.at(ct_imu)(0));
            assert(wm(1)==vec_wm.at(ct_imu)(1));
            assert(wm(2)==vec_wm.at(ct_imu)(2));
            assert(am(0)==vec_am.at(ct_imu)(0));
            assert(am(1)==vec_am.at(ct_imu)(1));
            assert(am(2)==vec_am.at(ct_imu)(2));
            ct_imu++;
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim2.get_next_cam(time_cam, camids, feats);
        if(hascam) {
            assert(time_cam==vec_camtime.at(ct_cam));
            for(size_t camid=0; camid<feats.size(); camid++) {
                for(size_t i=0; i<feats.at(camid).size(); i++) {
                    assert(feats.at(camid).at(i).first==vec_feats.at(ct_cam).at(camid).at(i).first);
                    assert(feats.at(camid).at(i).second(0)==vec_feats.at(ct_cam).at(camid).at(i).second(0));
                    assert(feats.at(camid).at(i).second(1)==vec_feats.at(ct_cam).at(camid).at(i).second(1));
                }
            }
            ct_cam++;
        }

    }



    // Done!
    printf("success! they all are the same!\n");
    return EXIT_SUCCESS;


}




