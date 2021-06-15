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



#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <memory>

#include "core/RosVisualizer.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

std::shared_ptr<VioManager> sys;
std::shared_ptr<RosVisualizer> viz;

// Main function
int main(int argc, char **argv) {

  // Launch our ros node
  ros::init(argc, argv, "run_serial_msckf");
  ros::NodeHandle nh("~");

  // Create our VIO system
  VioManagerOptions params = parse_ros_nodehandler(nh);
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<RosVisualizer>(nh, sys);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic
  std::string topic_imu;
  nh.param<std::string>("topic_imu", topic_imu, "/imu0");

  // Our camera topics (stereo pairs and non-stereo mono)
  std::vector<std::pair<size_t, std::string>> topic_cameras;
  std::vector<int> stereo_cam_ids;
  for (const auto &pair : params.stereo_pairs) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    nh.param<std::string>("topic_camera" + std::to_string(pair.first), cam_topic0, "/cam" + std::to_string(pair.first) + "/image_raw");
    nh.param<std::string>("topic_camera" + std::to_string(pair.second), cam_topic1, "/cam" + std::to_string(pair.second) + "/image_raw");
    topic_cameras.emplace_back(pair.first, cam_topic0);
    topic_cameras.emplace_back(pair.second, cam_topic1);
    stereo_cam_ids.push_back(pair.first);
    stereo_cam_ids.push_back(pair.second);
    ROS_INFO("serial cam (stereo): %s", cam_topic0.c_str());
    ROS_INFO("serial cam (stereo): %s", cam_topic1.c_str());
  }
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    // Skip if already have been added
    if (std::find(stereo_cam_ids.begin(), stereo_cam_ids.end(), i) != stereo_cam_ids.end())
      continue;
    // read in the topic
    std::string cam_topic;
    nh.param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
    topic_cameras.emplace_back(i, cam_topic);
    ROS_INFO("serial cam (mono): %s", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag;
  nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
  ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

  // Load groundtruth if we have it
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  if (nh.hasParam("path_gt")) {
    std::string path_to_gt;
    nh.param<std::string>("path_gt", path_to_gt, "");
    DatasetReader::load_gt_file(path_to_gt, gt_states);
    ROS_INFO("gt file path is: %s", path_to_gt.c_str());
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  ROS_INFO("bag start: %.1f", bag_start);
  ROS_INFO("bag duration: %.1f", bag_durr);

  // Read in what mode we should be processing in (1=mono, 2=stereo)
  int max_cameras;
  nh.param<int>("max_cameras", max_cameras, 1);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  ROS_INFO("time start = %.6f", time_init.toSec());
  ROS_INFO("time end   = %.6f", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Latest collection of image information
  std::map<size_t, std::pair<double, cv::Mat>> image_buffer;

  // Step through the rosbag
  for (const rosbag::MessageInstance &m : view) {

    // If ros is wants us to stop, break out
    if (!ros::ok())
      break;

    // Handle IMU measurement
    sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
    if (s2 != nullptr && m.getTopic() == topic_imu) {
      // convert into correct format
      ov_core::ImuData message;
      message.timestamp = (*s2).header.stamp.toSec();
      message.wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
      message.am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
      // send it to our VIO system
      sys->feed_measurement_imu(message);
      viz->visualize();
      viz->visualize_odometry(message.timestamp);
    }

    // Handle CAMERA measurement
    for (const auto &pair : topic_cameras) {
      sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
      if (s0 != nullptr && m.getTopic() == pair.second) {
        // Get the image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvShare(s0, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception &e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          continue;
        }
        // Save to our temp variable
        image_buffer[pair.first] = {cv_ptr->header.stamp.toSec(), cv_ptr->image.clone()};
      }
    }

    // MONO: Now loop through our buffer and see if we have have any we can send to the VIO
    auto it0 = image_buffer.begin();
    while (it0 != image_buffer.end()) {
      // skip if a stereo image
      if (std::find(stereo_cam_ids.begin(), stereo_cam_ids.end(), it0->first) != stereo_cam_ids.end()) {
        it0++;
        continue;
      }
      // process once we have initialized with the GT
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(it0->second.first, imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      } else if (gt_states.empty() || sys->initialized()) {
        ov_core::CameraData message;
        message.timestamp = it0->second.first;
        message.sensor_ids.push_back(it0->first);
        message.images.push_back(it0->second.second);
        sys->feed_measurement_camera(message);
      }
      it0 = image_buffer.erase(it0);
    }

    // STEREO: Now loop through our buffer and see if we have have any we can send to the VIO
    for (const auto &stereo : params.stereo_pairs) {
      // skip if we don't have both images
      if (image_buffer.find(stereo.first) == image_buffer.end() || image_buffer.find(stereo.second) == image_buffer.end())
        continue;
      // process once we have initialized with the GT
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() &&
          DatasetReader::get_gt_state(image_buffer.at(stereo.first).first, imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      } else if (gt_states.empty() || sys->initialized()) {
        ov_core::CameraData message;
        message.timestamp = image_buffer.at(stereo.first).first;
        message.sensor_ids.push_back(stereo.first);
        message.sensor_ids.push_back(stereo.second);
        message.images.push_back(image_buffer.at(stereo.first).second);
        message.images.push_back(image_buffer.at(stereo.second).second);
        sys->feed_measurement_camera(message);
      }
      image_buffer.erase(stereo.first);
      image_buffer.erase(stereo.second);
    }
  }

  // Final visualization
  viz->visualize_final();

  // Done!
  return EXIT_SUCCESS;
}
