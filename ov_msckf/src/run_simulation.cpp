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

#include <csignal>
#include <memory>

#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "utils/colors.h"
#include "utils/dataset_reader.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#if ROS_AVAILABLE
#include "ros/RosVisualizer.h"
#include <ros/ros.h>
#endif

using namespace ov_msckf;

std::shared_ptr<Simulator> sim;
std::shared_ptr<VioManager> sys;
#if ROS_AVAILABLE
std::shared_ptr<RosVisualizer> viz;
#endif

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE
  // Launch our ros node
  ros::init(argc, argv, "run_simulation");
  ros::NodeHandle nh("~");
  nh.param<std::string>("config_path", config_path, config_path);
#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE
  parser->set_node_handler(nh);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.print_and_load_simulation(parser);
  sim = std::make_shared<Simulator>(params);
  sys = std::make_shared<VioManager>(params);
#if ROS_AVAILABLE
  viz = std::make_shared<RosVisualizer>(nh, sys, sim);
#endif

  // Ensure we read in all parameters required
  if(!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Get initial state
  Eigen::Matrix<double, 17, 1> imustate;
  bool success = sim->get_state(sim->current_timestamp(), imustate);
  if (!success) {
    PRINT_ERROR(RED "[SIM]: Could not initialize the filter to the first state\n" RESET);
    PRINT_ERROR(RED "[SIM]: Did the simulator load properly???\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Since the state time is in the camera frame of reference
  // Subtract out the imu to camera time offset
  imustate(0, 0) -= sim->get_true_parameters().calib_camimu_dt;

  // Initialize our filter with the groundtruth
  sys->initialize_with_gt(imustate);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Buffer our camera image
  double buffer_timecam = -1;
  std::vector<int> buffer_camids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> buffer_feats;

  // Step through the rosbag
  signal(SIGINT, signal_callback_handler);
#if ROS_AVAILABLE
  while (sim->ok() && ros::ok()) {
#else
  while (sim->ok()) {
#endif

    // IMU: get the next simulated IMU measurement if we have it
    ov_core::ImuData message_imu;
    bool hasimu = sim->get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
    if (hasimu) {
      sys->feed_measurement_imu(message_imu);
#if ROS_AVAILABLE
      viz->visualize_odometry(message_imu.timestamp);
#endif
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim->get_next_cam(time_cam, camids, feats);
    if (hascam) {
      if (buffer_timecam != -1) {
        sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
#if ROS_AVAILABLE
        viz->visualize();
#endif
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
#if ROS_AVAILABLE
  viz->visualize_final();
#endif

  // Done!
  return EXIT_SUCCESS;
}
