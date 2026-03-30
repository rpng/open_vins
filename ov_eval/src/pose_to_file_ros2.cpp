/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "utils/Recorder.h"

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>

#include "utils/print.h"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pose_to_file", options);

  std::string verbosity;
  node->get_parameter_or<std::string>("verbosity", verbosity, "INFO");
  ov_core::Printer::setPrintLevel(verbosity);

  std::string topic;
  node->get_parameter_or<std::string>("topic", topic, "");
  std::string topic_type;
  node->get_parameter_or<std::string>("topic_type", topic_type, "");
  std::string fileoutput;
  node->get_parameter_or<std::string>("output", fileoutput, "");
  if (topic.empty() || topic_type.empty() || fileoutput.empty()) {
    PRINT_ERROR("Missing parameters: topic, topic_type, and output are required");
    return EXIT_FAILURE;
  }

  PRINT_DEBUG("Done reading config values");
  PRINT_DEBUG(" - topic = %s", topic.c_str());
  PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
  PRINT_DEBUG(" - file = %s", fileoutput.c_str());

  ov_eval::Recorder recorder(fileoutput);
  rclcpp::SubscriptionBase::SharedPtr sub;

  if (topic_type == "PoseWithCovarianceStamped") {
    sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, 9999, std::bind(&ov_eval::Recorder::callback_posecovariance, &recorder, std::placeholders::_1));
  } else if (topic_type == "PoseStamped") {
    sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 9999, std::bind(&ov_eval::Recorder::callback_pose, &recorder, std::placeholders::_1));
  } else if (topic_type == "TransformStamped") {
    sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
        topic, 9999, std::bind(&ov_eval::Recorder::callback_transform, &recorder, std::placeholders::_1));
  } else if (topic_type == "Odometry") {
    sub = node->create_subscription<nav_msgs::msg::Odometry>(
        topic, 9999, std::bind(&ov_eval::Recorder::callback_odometry, &recorder, std::placeholders::_1));
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    return EXIT_FAILURE;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
