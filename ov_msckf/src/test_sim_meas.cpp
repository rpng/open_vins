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

#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#if ROS_AVAILABLE
#include <ros/ros.h>
#endif

#include "core/VioManagerOptions.h"
#include "sim/Simulator.h"

using namespace ov_msckf;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

// Main function
int main(int argc, char **argv) {

#if ROS_AVAILABLE
  // Launch our ros node
  ros::init(argc, argv, "test_sim_meas");
  ros::NodeHandle nh("~");
#endif

  // Create the simulator
  VioManagerOptions params;
  Simulator sim(params);

  // Continue to simulate until we have processed all the measurements
  signal(SIGINT, signal_callback_handler);
  while (sim.ok()) {

    // IMU: get the next simulated IMU measurement if we have it
    double time_imu;
    Eigen::Vector3d wm, am;
    bool hasimu = sim.get_next_imu(time_imu, wm, am);
    if (hasimu) {
      PRINT_DEBUG("new imu measurement = %0.15g | w = %0.3g | a = %0.3g\n", time_imu, wm.norm(), am.norm());
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim.get_next_cam(time_cam, camids, feats);
    if (hascam) {
      PRINT_DEBUG("new cam measurement = %0.15g | %u cameras | uvs(0) = %u \n", time_cam, camids.size(), feats.at(0).size());
    }
  }

  // Done!
  return EXIT_SUCCESS;
};