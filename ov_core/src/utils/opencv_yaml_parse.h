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

#ifndef OPENCV_YAML_PARSER_H
#define OPENCV_YAML_PARSER_H

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#elif ROS_AVAILABLE == 2
#include <rclcpp/rclcpp.hpp>
#endif

#include "colors.h"
#include "print.h"
#include "quat_ops.h"

namespace ov_core {

/**
 * @brief Helper class to do OpenCV yaml parsing from both file and ROS.
 *
 * The logic is as follows:
 * - Given a path to the main config file we will load it into our cv::FileStorage object.
 * - From there the user can request for different parameters of different types from the config.
 * - If we have ROS, then we will also check to see if the user has overridden any config files via ROS.
 * - The ROS parameters always take priority over the ones in our config.
 *
 * NOTE: There are no "nested" yaml parameters. They are all under the "root" of the yaml file!!!
 * NOTE: The camera and imu have nested, but those are handled externally....
 */
class YamlParser {
public:
  /**
   * @brief Constructor that loads all three configuration files
   * @param config_path Path to the YAML file we will parse
   * @param fail_if_not_found If we should terminate the program if we can't open the config file
   */
  explicit YamlParser(const std::string &config_path, bool fail_if_not_found = true) : config_path_(config_path) {

    // Check if file exists
    if (!fail_if_not_found && !boost::filesystem::exists(config_path)) {
      config = nullptr;
      return;
    }
    if (!boost::filesystem::exists(config_path)) {
      PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET, config_path.c_str());
      std::exit(EXIT_FAILURE);
    }

    // Open the file, error if we can't
    config = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
    if (!fail_if_not_found && !config->isOpened()) {
      config = nullptr;
      return;
    }
    if (!config->isOpened()) {
      PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET, config_path.c_str());
      std::exit(EXIT_FAILURE);
    }
  }

#if ROS_AVAILABLE == 1
  /// Allows setting of the node handler if we have ROS to override parameters
  void set_node_handler(std::shared_ptr<ros::NodeHandle> nh_) { this->nh = nh_; }
#endif

#if ROS_AVAILABLE == 2
  /// Allows setting of the node if we have ROS to override parameters
  void set_node(std::shared_ptr<rclcpp::Node> &node_) { this->node = node_; }
#endif

  /**
   * @brief Will get the folder this config file is in
   * @return Config folder
   */
  std::string get_config_folder() { return config_path_.substr(0, config_path_.find_last_of('/')) + "/"; }

  /**
   * @brief Check to see if all parameters were read succesfully
   * @return True if we found all parameters
   */
  bool successful() const { return all_params_found_successfully; }

  /**
   * @brief Custom parser for the ESTIMATOR parameters.
   *
   * This will load the data from the main config file.
   * If it is unable it will give a warning to the user it couldn't be found.
   *
   * @tparam T Type of parameter we are looking for.
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  template <class T> void parse_config(const std::string &node_name, T &node_result, bool required = true) {

#if ROS_AVAILABLE == 1
    if (nh != nullptr && nh->getParam(node_name, node_result)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, node_name.c_str());
      nh->param<T>(node_name, node_result);
      return;
    }
#elif ROS_AVAILABLE == 2
    if (node != nullptr && node->has_parameter(node_name)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, node_name.c_str());
      node->get_parameter<T>(node_name, node_result);
      return;
    }
#endif

    // Else we just parse from the YAML file!
    parse_config_yaml(node_name, node_result, required);
  }

  /**
   * @brief Custom parser for the external parameter files with levels.
   *
   * This will first load the external file requested.
   * From there it will try to find the first level requested (e.g. imu0, cam0, cam1).
   * Then the requested node can be found under this sensor name.
   * ROS can override the config with `<sensor_name>_<node_name>` (e.g., cam0_distortion).
   *
   * @tparam T Type of parameter we are looking for.
   * @param external_node_name Name of the node we will get our relative path from
   * @param sensor_name The first level node we will try to get the requested node under
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  template <class T>
  void parse_external(const std::string &external_node_name, const std::string &sensor_name, const std::string &node_name, T &node_result,
                      bool required = true) {

#if ROS_AVAILABLE == 1
    std::string rosnode = sensor_name + "_" + node_name;
    if (nh != nullptr && nh->getParam(rosnode, node_result)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      nh->param<T>(rosnode, node_result);
      return;
    }
#elif ROS_AVAILABLE == 2
    std::string rosnode = sensor_name + "_" + node_name;
    if (node != nullptr && node->has_parameter(rosnode)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      node->get_parameter<T>(rosnode, node_result);
      return;
    }
#endif

    // Else we just parse from the YAML file!
    parse_external_yaml(external_node_name, sensor_name, node_name, node_result, required);
  }

  /**
   * @brief Custom parser for Matrix3d in the external parameter files with levels.
   *
   * This will first load the external file requested.
   * From there it will try to find the first level requested (e.g. imu0, cam0, cam1).
   * Then the requested node can be found under this sensor name.
   * ROS can override the config with `<sensor_name>_<node_name>` (e.g., cam0_distortion).
   *
   * @param external_node_name Name of the node we will get our relative path from
   * @param sensor_name The first level node we will try to get the requested node under
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse_external(const std::string &external_node_name, const std::string &sensor_name, const std::string &node_name,
                      Eigen::Matrix3d &node_result, bool required = true) {

#if ROS_AVAILABLE == 1
    // If we have the ROS parameter, we should just get that one
    // NOTE: for our 3x3 matrix we should read it as an array from ROS then covert it back into the 3x3
    std::string rosnode = sensor_name + "_" + node_name;
    std::vector<double> matrix_RCtoI;
    if (nh != nullptr && nh->getParam(rosnode, matrix_RCtoI)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      nh->param<std::vector<double>>(rosnode, matrix_RCtoI);
      node_result << matrix_RCtoI.at(0), matrix_RCtoI.at(1), matrix_RCtoI.at(2), matrix_RCtoI.at(3), matrix_RCtoI.at(4), matrix_RCtoI.at(5),
          matrix_RCtoI.at(6), matrix_RCtoI.at(7), matrix_RCtoI.at(8);
      return;
    }
#elif ROS_AVAILABLE == 2
    // If we have the ROS parameter, we should just get that one
    // NOTE: for our 3x3 matrix we should read it as an array from ROS then covert it back into the 4x4
    std::string rosnode = sensor_name + "_" + node_name;
    std::vector<double> matrix_RCtoI;
    if (node != nullptr && node->has_parameter(rosnode)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      node->get_parameter<std::vector<double>>(rosnode, matrix_RCtoI);
      node_result << matrix_RCtoI.at(0), matrix_RCtoI.at(1), matrix_RCtoI.at(2), matrix_RCtoI.at(3), matrix_RCtoI.at(4), matrix_RCtoI.at(5),
          matrix_RCtoI.at(6), matrix_RCtoI.at(7), matrix_RCtoI.at(8);
      return;
    }
#endif

    // Else we just parse from the YAML file!
    parse_external_yaml(external_node_name, sensor_name, node_name, node_result, required);
  }

  /**
   * @brief Custom parser for Matrix4d in the external parameter files with levels.
   *
   * This will first load the external file requested.
   * From there it will try to find the first level requested (e.g. imu0, cam0, cam1).
   * Then the requested node can be found under this sensor name.
   * ROS can override the config with `<sensor_name>_<node_name>` (e.g., cam0_distortion).
   *
   * @param external_node_name Name of the node we will get our relative path from
   * @param sensor_name The first level node we will try to get the requested node under
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse_external(const std::string &external_node_name, const std::string &sensor_name, const std::string &node_name,
                      Eigen::Matrix4d &node_result, bool required = true) {

#if ROS_AVAILABLE == 1
    // If we have the ROS parameter, we should just get that one
    // NOTE: for our 4x4 matrix we should read it as an array from ROS then covert it back into the 4x4
    std::string rosnode = sensor_name + "_" + node_name;
    std::vector<double> matrix_TCtoI;
    if (nh != nullptr && nh->getParam(rosnode, matrix_TCtoI)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      nh->param<std::vector<double>>(rosnode, matrix_TCtoI);
      node_result << matrix_TCtoI.at(0), matrix_TCtoI.at(1), matrix_TCtoI.at(2), matrix_TCtoI.at(3), matrix_TCtoI.at(4), matrix_TCtoI.at(5),
          matrix_TCtoI.at(6), matrix_TCtoI.at(7), matrix_TCtoI.at(8), matrix_TCtoI.at(9), matrix_TCtoI.at(10), matrix_TCtoI.at(11),
          matrix_TCtoI.at(12), matrix_TCtoI.at(13), matrix_TCtoI.at(14), matrix_TCtoI.at(15);
      return;
    }
#elif ROS_AVAILABLE == 2
    // If we have the ROS parameter, we should just get that one
    // NOTE: for our 4x4 matrix we should read it as an array from ROS then covert it back into the 4x4
    std::string rosnode = sensor_name + "_" + node_name;
    std::vector<double> matrix_TCtoI;
    if (node != nullptr && node->has_parameter(rosnode)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      node->get_parameter<std::vector<double>>(rosnode, matrix_TCtoI);
      node_result << matrix_TCtoI.at(0), matrix_TCtoI.at(1), matrix_TCtoI.at(2), matrix_TCtoI.at(3), matrix_TCtoI.at(4), matrix_TCtoI.at(5),
          matrix_TCtoI.at(6), matrix_TCtoI.at(7), matrix_TCtoI.at(8), matrix_TCtoI.at(9), matrix_TCtoI.at(10), matrix_TCtoI.at(11),
          matrix_TCtoI.at(12), matrix_TCtoI.at(13), matrix_TCtoI.at(14), matrix_TCtoI.at(15);
      return;
    }
#endif

    // Else we just parse from the YAML file!
    parse_external_yaml(external_node_name, sensor_name, node_name, node_result, required);
  }

#if ROS_AVAILABLE == 2
  /// For ROS2 we need to override the int since it seems to only support int64_t types
  /// https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_parameter.html
  void parse_config(const std::string &node_name, int &node_result, bool required = true) {
    int64_t val = node_result;
    if (node != nullptr && node->has_parameter(node_name)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, node_name.c_str());
      node->get_parameter<int64_t>(node_name, val);
      node_result = (int)val;
      return;
    }
    parse_config_yaml(node_name, node_result, required);
  }
  /// For ROS2 we need to override the int since it seems to only support int64_t types
  /// https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_parameter.html
  void parse_external(const std::string &external_node_name, const std::string &sensor_name, const std::string &node_name,
                      std::vector<int> &node_result, bool required = true) {
    std::vector<int64_t> val;
    for (auto tmp : node_result)
      val.push_back(tmp);
    std::string rosnode = sensor_name + "_" + node_name;
    if (node != nullptr && node->has_parameter(rosnode)) {
      PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, rosnode.c_str());
      node->get_parameter<std::vector<int64_t>>(rosnode, val);
      node_result.clear();
      for (auto tmp : val)
        node_result.push_back((int)tmp);
      return;
    }
    parse_external_yaml(external_node_name, sensor_name, node_name, node_result, required);
  }
#endif

private:
  /// Path to the config file
  std::string config_path_;

  /// Our config file with the data in it
  std::shared_ptr<cv::FileStorage> config;

  /// Record if all parameters were found
  bool all_params_found_successfully = true;

#if ROS_AVAILABLE == 1
  /// ROS1 node handler that will override values
  std::shared_ptr<ros::NodeHandle> nh;
#endif

#if ROS_AVAILABLE == 2
  /// Our ROS2 rclcpp node pointer
  std::shared_ptr<rclcpp::Node> node = nullptr;
#endif

  /**
   * @brief Given a YAML node object, this check to see if we have a valid key
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @return True if we can get the data
   */
  static bool node_found(const cv::FileNode &file_node, const std::string &node_name) {
    bool found_node = false;
    for (const auto &item : file_node) {
      if (item.name() == node_name) {
        found_node = true;
      }
    }
    return found_node;
  }

  /**
   * @brief This function will try to get the requested parameter from our config.
   *
   * If it is unable to find it, it will give a warning to the user it couldn't be found.
   *
   * @tparam T Type of parameter we are looking for.
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  template <class T> void parse(const cv::FileNode &file_node, const std::string &node_name, T &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    try {
      file_node[node_name] >> node_result;
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

  /**
   * @brief Custom parser for booleans (0,false,False,FALSE=>false and 1,true,True,TRUE=>false)
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, bool &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    try {
      if (file_node[node_name].isInt() && (int)file_node[node_name] == 1) {
        node_result = true;
        return;
      }
      if (file_node[node_name].isInt() && (int)file_node[node_name] == 0) {
        node_result = false;
        return;
      }
      // NOTE: we select the first bit of text as there can be a comment afterwards
      // NOTE: for example we could have "key: true #comment here..." where we only want "true"
      std::string value;
      file_node[node_name] >> value;
      value = value.substr(0, value.find_first_of('#'));
      value = value.substr(0, value.find_first_of(' '));
      if (value == "1" || value == "true" || value == "True" || value == "TRUE") {
        node_result = true;
      } else if (value == "0" || value == "false" || value == "False" || value == "FALSE") {
        node_result = false;
      } else {
        PRINT_WARNING(YELLOW "the node %s has an invalid boolean type of [%s]\n" RESET, node_name.c_str(), value.c_str());
        all_params_found_successfully = false;
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

  /**
   * @brief Custom parser for camera extrinsic 3x3 transformations
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, Eigen::Matrix3d &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    node_result = Eigen::Matrix3d::Identity();
    try {
      for (int r = 0; r < (int)file_node[node_name].size() && r < 3; r++) {
        for (int c = 0; c < (int)file_node[node_name][r].size() && c < 3; c++) {
          node_result(r, c) = (double)file_node[node_name][r][c];
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

  /**
   * @brief Custom parser for camera extrinsic 4x4 transformations
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, Eigen::Matrix4d &node_result, bool required = true) {

    // See if we need to flip the node name
    std::string node_name_local = node_name;
    if (node_name == "T_cam_imu" && !node_found(file_node, node_name)) {
      PRINT_INFO("parameter T_cam_imu not found, trying T_imu_cam instead (will return T_cam_imu still)!\n");
      node_name_local = "T_imu_cam";
    } else if (node_name == "T_imu_cam" && !node_found(file_node, node_name)) {
      PRINT_INFO("parameter T_imu_cam not found, trying T_cam_imu instead (will return T_imu_cam still)!\n");
      node_name_local = "T_cam_imu";
    }

    // Check that we have the requested node
    if (!node_found(file_node, node_name_local)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name_local.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name_local.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    node_result = Eigen::Matrix4d::Identity();
    try {
      for (int r = 0; r < (int)file_node[node_name_local].size() && r < 4; r++) {
        for (int c = 0; c < (int)file_node[node_name_local][r].size() && c < 4; c++) {
          node_result(r, c) = (double)file_node[node_name_local][r][c];
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }

    // Finally if we flipped the transform, get the correct value
    if (node_name_local != node_name) {
      Eigen::Matrix4d tmp(node_result);
      node_result = ov_core::Inv_se3(tmp);
    }
  }

  /**
   * @brief Custom parser for the ESTIMATOR parameters.
   *
   * This will load the data from the main config file.
   * If it is unable it will give a warning to the user it couldn't be found.
   *
   * @tparam T Type of parameter we are looking for.
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  template <class T> void parse_config_yaml(const std::string &node_name, T &node_result, bool required = true) {

    // Directly return if the config hasn't been opened
    if (config == nullptr)
      return;

    // Else lets get the one from the config
    try {
      parse(config->root(), node_name, node_result, required);
    } catch (...) {
      PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                    typeid(node_result).name());
      all_params_found_successfully = false;
    }
  }

  /**
   * @brief Custom parser for the EXTERNAL parameter files with levels.
   *
   * This will first load the external file requested.
   * From there it will try to find the first level requested (e.g. imu0, cam0, cam1).
   * Then the requested node can be found under this sensor name.
   * ROS can override the config with `<sensor_name>_<node_name>` (e.g., cam0_distortion).
   *
   * @tparam T Type of parameter we are looking for.
   * @param external_node_name Name of the node we will get our relative path from
   * @param sensor_name The first level node we will try to get the requested node under
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  template <class T>
  void parse_external_yaml(const std::string &external_node_name, const std::string &sensor_name, const std::string &node_name,
                           T &node_result, bool required = true) {
    // Directly return if the config hasn't been opened
    if (config == nullptr)
      return;

    // Create the path the external yaml file
    std::string path;
    if (!node_found(config->root(), external_node_name)) {
      PRINT_ERROR(RED "the external node %s could not be found!\n" RESET, external_node_name.c_str());
      std::exit(EXIT_FAILURE);
    }
    (*config)[external_node_name] >> path;
    std::string relative_folder = config_path_.substr(0, config_path_.find_last_of('/')) + "/";

    // Now actually try to load them from file!
    auto config_external = std::make_shared<cv::FileStorage>(relative_folder + path, cv::FileStorage::READ);
    if (!config_external->isOpened()) {
      PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET, (relative_folder + path).c_str());
      std::exit(EXIT_FAILURE);
    }

    // Check that we have the requested node
    if (!node_found(config_external->root(), sensor_name)) {
      PRINT_WARNING(YELLOW "the sensor %s of type [%s] was not found...\n" RESET, sensor_name.c_str(), typeid(node_result).name());
      all_params_found_successfully = false;
      return;
    }

    // Else lets get it!
    try {
      parse((*config_external)[sensor_name], node_name, node_result, required);
    } catch (...) {
      PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in [%s] in the external %s config file!\n" RESET, node_name.c_str(),
                    typeid(node_result).name(), sensor_name.c_str(), external_node_name.c_str());
      all_params_found_successfully = false;
    }
  }
};

} /* namespace ov_core */

#endif /* OPENCV_YAML_PARSER_H */
