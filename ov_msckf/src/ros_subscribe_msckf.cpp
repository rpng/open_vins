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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
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

// Callback functions
void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);
void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0);
void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1);

// Main function
int main(int argc, char **argv) {

  // Launch our ros node
  ros::init(argc, argv, "run_subscribe_msckf");
  ros::NodeHandle nh("~");

  // Create our VIO system
  VioManagerOptions params = parse_ros_nodehandler(nh);
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<RosVisualizer>(nh, sys);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Create imu subscriber
  std::string topic_imu;
  nh.param<std::string>("topic_imu", topic_imu, "/imu0");
  ros::Subscriber subimu = nh.subscribe(topic_imu, 9999, callback_inertial);

  // Create camera subscriber data vectors
  std::vector<int> added_cam_ids;
  std::vector<ros::Subscriber> subs_cam;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  std::vector<std::unique_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;
  std::vector<std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>> sync_subs_cam;

  // Logic for sync stereo subscriber
  // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
  for (const auto &pair : params.stereo_pairs) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    nh.param<std::string>("topic_camera" + std::to_string(pair.first), cam_topic0, "/cam" + std::to_string(pair.first) + "/image_raw");
    nh.param<std::string>("topic_camera" + std::to_string(pair.second), cam_topic1, "/cam" + std::to_string(pair.second) + "/image_raw");
    // Create sync filter (they have unique pointers internally, so we have to use move logic here...)
    auto image_sub0 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(nh, cam_topic0, 5));
    auto image_sub1 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(nh, cam_topic1, 5));
    auto sync = std::unique_ptr<message_filters::Synchronizer<sync_pol>>(
        new message_filters::Synchronizer<sync_pol>(sync_pol(5), *image_sub0, *image_sub1));
    sync->registerCallback(boost::bind(&callback_stereo, _1, _2, pair.first, pair.second));
    // Append to our vector of subscribers
    added_cam_ids.push_back(pair.first);
    added_cam_ids.push_back(pair.second);
    sync_cam.push_back(std::move(sync));
    sync_subs_cam.push_back(std::move(image_sub0));
    sync_subs_cam.push_back(std::move(image_sub1));
    ROS_INFO("subscribing to cam (stereo): %s", cam_topic0.c_str());
    ROS_INFO("subscribing to cam (stereo): %s", cam_topic1.c_str());
  }

  // Now we should add any non-stereo callbacks here
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    // Skip if already have been added
    if (std::find(added_cam_ids.begin(), added_cam_ids.end(), i) != added_cam_ids.end())
      continue;
    // read in the topic
    std::string cam_topic;
    nh.param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
    // create subscriber
    subs_cam.push_back(nh.subscribe<sensor_msgs::Image>(cam_topic, 5, boost::bind(callback_monocular, _1, i)));
    ROS_INFO("subscribing to cam (mono): %s", cam_topic.c_str());
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Spin off to ROS
  // TODO: maybe should use multi-thread spinner
  // TODO: but need to support multi-threaded calls to viomanager
  ROS_INFO("done...spinning to ros");
  ros::spin();

  // Final visualization
  viz->visualize_final();

  // Done!
  return EXIT_SUCCESS;
}

void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  sys->feed_measurement_imu(message);
  viz->visualize();
  viz->visualize_odometry(message.timestamp);
}

void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0) {

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.images.push_back(cv_ptr->image.clone());

  // send it to our VIO system
  sys->feed_measurement_camera(message);
}

void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1) {

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr0->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.sensor_ids.push_back(cam_id1);
  message.images.push_back(cv_ptr0->image.clone());
  message.images.push_back(cv_ptr1->image.clone());

  // send it to our VIO system
  sys->feed_measurement_camera(message);
}
