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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "utils/dataset_reader.h"

/** one entry in the time-ordered bag message list (either IMU or image) */
struct BagMessage {
  std::string topic;
  double time_sec;
  sensor_msgs::msg::Imu::SharedPtr imu;
  sensor_msgs::msg::Image::SharedPtr image;
};

using namespace ov_msckf;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // Launch our ros node
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ros2_serial_msckf", options);
  node->get_parameter_or<std::string>("config_path", config_path, config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  //params.num_opencv_threads = 0; // uncomment if you want repeatability
  //params.use_multi_threading_pubs = 0; // uncomment if you want repeatability
  params.use_multi_threading_subs = false;
  auto sys = std::make_shared<VioManager>(params);
  auto viz = std::make_shared<ROS2Visualizer>(node, sys);

  // tear down ROS-owned objects before shutting down the ROS context.
  auto shutdown_and_cleanup = [&]() {
    viz.reset();
    sys.reset();
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  };

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    shutdown_and_cleanup();
    return EXIT_FAILURE;
  }

  if (params.state_options.num_cameras > 2) {
    PRINT_ERROR(RED "[SERIAL]: We currently only support 1 or 2 camera serial input....\n" RESET);
    shutdown_and_cleanup();
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic
  std::string topic_imu;
  node->get_parameter_or<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  PRINT_DEBUG("[SERIAL]: imu: %s\n", topic_imu.c_str());

  // Our camera topics
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic;
    node->get_parameter_or<std::string>(
        "topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag;
  node->get_parameter_or<std::string>(
      "path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // Load groundtruth if we have it
  // NOTE: needs to be a csv ASL format file
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  if (node->has_parameter("path_gt")) {
    std::string path_to_gt;
    node->get_parameter_or<std::string>("path_gt", path_to_gt, "");
    if (!path_to_gt.empty()) {
      ov_core::DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("[SERIAL]: gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start;
  node->get_parameter_or<double>("bag_start", bag_start, 0.0);
  double bag_durr;
  node->get_parameter_or<double>("bag_durr", bag_durr, -1.0);
  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // open bag with rosbag2 (uri = directory or path to bag; factory picks backend)
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = path_to_bag;

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;

  // collect all messages on our topics and deserialize
  std::vector<BagMessage> msgs;
  double min_time_sec = std::numeric_limits<double>::max();
  double max_time_sec = -std::numeric_limits<double>::max();
  double max_camera_time = -1;

  while (reader->has_next()) {
    if (!rclcpp::ok()) {
      break;
    }
    auto serialized = reader->read_next();
    const std::string & topic = serialized->topic_name;
    const double time_sec = static_cast<double>(serialized->time_stamp) / 1e9;

    if (topic == topic_imu) {
      auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::SerializedMessage extracted(*serialized->serialized_data);
      imu_serialization.deserialize_message(&extracted, imu_msg.get());
      msgs.push_back(BagMessage{topic, time_sec, imu_msg, nullptr});
      min_time_sec = std::min(min_time_sec, time_sec);
      max_time_sec = std::max(max_time_sec, time_sec);
    } else {
      for (int i = 0; i < params.state_options.num_cameras; i++) {
        if (topic == topic_cameras.at(i)) {
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          rclcpp::SerializedMessage extracted(*serialized->serialized_data);
          image_serialization.deserialize_message(&extracted, image_msg.get());
          msgs.push_back(BagMessage{topic, time_sec, nullptr, image_msg});
          min_time_sec = std::min(min_time_sec, time_sec);
          max_time_sec = std::max(max_time_sec, time_sec);
          max_camera_time = std::max(max_camera_time, time_sec);
          break;
        }
      }
    }
  }

  if (msgs.empty()) {
    PRINT_ERROR(RED "[SERIAL]: No messages to play on specified topics.  Exiting.\n" RESET);
    shutdown_and_cleanup();
    return EXIT_FAILURE;
  }

  // sort by time (same as ROS1 view order)
  std::sort(msgs.begin(), msgs.end(),
            [](const BagMessage & a, const BagMessage & b) { return a.time_sec < b.time_sec; });

  const double time_init = min_time_sec + bag_start;
  const double time_finish =
      (bag_durr < 0) ? max_time_sec : time_init + bag_durr;
  PRINT_DEBUG("time start = %.6f\n", time_init);
  PRINT_DEBUG("time end   = %.6f\n", time_finish);
  PRINT_DEBUG("[SERIAL]: total of %zu messages!\n", msgs.size());

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // loop through our message array and process them
  std::set<int> used_index;
  for (int m = 0; m < (int)msgs.size(); m++) {
    const BagMessage & bm = msgs.at(m);

    // end once we reach the last time, or skip if before beginning time
    if (!rclcpp::ok() || bm.time_sec > time_finish || bm.time_sec > max_camera_time) {
      break;
    }
    if (bm.time_sec < time_init) {
      continue;
    }

    // skip messages that we have already used
    if (used_index.find(m) != used_index.end()) {
      used_index.erase(m);
      continue;
    }

    // IMU processing
    if (bm.topic == topic_imu) {
      viz->callback_inertial(bm.imu);
      continue;
    }

    // camera processing
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {
      if (bm.topic != topic_cameras.at(cam_id)) {
        continue;
      }

      // find the other camera(s) for this time (within 0.02 s)
      std::map<int, int> camid_to_msg_index;
      double meas_time = bm.time_sec;
      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
        if (cam_idt == cam_id) {
          camid_to_msg_index.insert({cam_id, m});
          continue;
        }
        int cam_idt_idx = -1;
        for (int mt = m; mt < (int)msgs.size(); mt++) {
          if (msgs.at(mt).topic != topic_cameras.at(cam_idt)) {
            continue;
          }
          if (std::abs(msgs.at(mt).time_sec - meas_time) < 0.02) {
            cam_idt_idx = mt;
          }
          break;
        }
        if (cam_idt_idx != -1) {
          camid_to_msg_index.insert({cam_idt, cam_idt_idx});
        }
      }

      if ((int)camid_to_msg_index.size() != params.state_options.num_cameras) {
        PRINT_DEBUG(YELLOW "[SERIAL]: Unable to find stereo pair for message %d at %.2f into bag (will skip!)\n" RESET,
                    m, meas_time - time_init);
        continue;
      }

      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() &&
          ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
        sys->initialize_with_gt(imustate);
      }

      if (params.state_options.num_cameras == 1) {
        viz->callback_monocular(msgs.at(camid_to_msg_index.at(0)).image, 0);
      } else if (params.state_options.num_cameras == 2) {
        used_index.insert(camid_to_msg_index.at(0));
        used_index.insert(camid_to_msg_index.at(1));
        viz->callback_stereo(
            msgs.at(camid_to_msg_index.at(0)).image,
            msgs.at(camid_to_msg_index.at(1)).image,
            0, 1);
      }
      break;
    }
  }

  viz->visualize_final();
  shutdown_and_cleanup();

  return EXIT_SUCCESS;
}
