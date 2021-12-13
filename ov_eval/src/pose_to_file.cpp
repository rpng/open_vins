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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "utils/Recorder.h"
#include "utils/print.h"

int main(int argc, char **argv) {

  // Create ros node
  ros::init(argc, argv, "pose_to_file");
  ros::NodeHandle nh("~");

  // Verbosity setting
  std::string verbosity;
  nh.param<std::string>("verbosity", verbosity, "INFO");
  ov_core::Printer::setPrintLevel(verbosity);

  // Get parameters to subscribe
  std::string topic, topic_type, fileoutput;
  nh.getParam("topic", topic);
  nh.getParam("topic_type", topic_type);
  nh.getParam("output", fileoutput);

  // Debug
  PRINT_DEBUG("Done reading config values");
  PRINT_DEBUG(" - topic = %s", topic.c_str());
  PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
  PRINT_DEBUG(" - file = %s", fileoutput.c_str());

  // Create the recorder object
  ov_eval::Recorder recorder(fileoutput);

  // Subscribe to topic
  ros::Subscriber sub;
  if (topic_type == std::string("PoseWithCovarianceStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_posecovariance, &recorder);
  } else if (topic_type == std::string("PoseStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_pose, &recorder);
  } else if (topic_type == std::string("TransformStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_transform, &recorder);
  } else if (topic_type == std::string("Odometry")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_odometry, &recorder);
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    std::exit(EXIT_FAILURE);
  }

  // Done!
  ros::spin();
  return EXIT_SUCCESS;
}
