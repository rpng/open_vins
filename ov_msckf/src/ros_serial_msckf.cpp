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
  if (params.use_stereo && params.state_options.num_cameras == 2) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    nh.param<std::string>("topic_camera" + std::to_string(0), cam_topic0, "/cam" + std::to_string(0) + "/image_raw");
    nh.param<std::string>("topic_camera" + std::to_string(1), cam_topic1, "/cam" + std::to_string(1) + "/image_raw");
    topic_cameras.emplace_back(0, cam_topic0);
    topic_cameras.emplace_back(1, cam_topic1);
    ROS_INFO("serial cam (stereo): %s", cam_topic0.c_str());
    ROS_INFO("serial cam (stereo): %s", cam_topic1.c_str());
  } else {
    for (int i = 0; i < params.state_options.num_cameras; i++) {
      // read in the topic
      std::string cam_topic;
      nh.param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
      topic_cameras.emplace_back(i, cam_topic);
      ROS_INFO("serial cam (mono): %s", cam_topic.c_str());
    }
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
    if(!path_to_gt.empty()) {
      DatasetReader::load_gt_file(path_to_gt, gt_states);
      ROS_INFO("gt file path is: %s", path_to_gt.c_str());
    }
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  ROS_INFO("bag start: %.1f", bag_start);
  ROS_INFO("bag duration: %.1f", bag_durr);

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

  // Open our iterators
  auto view_imu = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(topic_imu), time_init, time_finish);
  auto view_imu_iter = view_imu->begin();
  std::vector<std::shared_ptr<rosbag::View>> view_cameras;
  std::vector<rosbag::View::iterator> view_cameras_iterators;
  for (const auto &topic : topic_cameras) {
    auto view_tmp = std::make_shared<rosbag::View>(bag, rosbag::TopicQuery(topic.second), time_init, time_finish);
    view_cameras.push_back(view_tmp);
    view_cameras_iterators.push_back(view_tmp->begin());
  }

  // Record the current measurement timestamps
  sensor_msgs::Imu::ConstPtr msg_imu_current;
  sensor_msgs::Imu::ConstPtr msg_imu_next;
  std::vector<sensor_msgs::Image::ConstPtr> msg_images_current;
  std::vector<sensor_msgs::Image::ConstPtr> msg_images_next;
  msg_imu_current = view_imu_iter->instantiate<sensor_msgs::Imu>();
  view_imu_iter++;
  msg_imu_next = view_imu_iter->instantiate<sensor_msgs::Imu>();
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    msg_images_current.emplace_back(view_cameras_iterators.at(i)->instantiate<sensor_msgs::Image>());
    view_cameras_iterators.at(i)++;
    msg_images_next.emplace_back(view_cameras_iterators.at(i)->instantiate<sensor_msgs::Image>());
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  while (ros::ok()) {

    // Check if we should end since we have run out of measurements
    bool should_stop = false;
    if (view_imu_iter == view_imu->end()) {
      should_stop = true;
    }
    for (int i = 0; i < params.state_options.num_cameras; i++) {
      if (view_cameras_iterators.at(i) == view_cameras.at(i)->end()) {
        should_stop = true;
      }
    }
    if (should_stop) {
      break;
    }

    // We should process the IMU if all the current cameras are greater then its time
    bool should_process_imu = false;
    for (int i = 0; i < params.state_options.num_cameras; i++) {
      double time_imu = msg_imu_current->header.stamp.toSec();
      double time_cam = msg_images_current.at(i)->header.stamp.toSec();
      if (time_imu <= time_cam) {
        should_process_imu = true;
      }
    }
    if (should_process_imu) {
      // convert into correct format
      ov_core::ImuData message;
      message.timestamp = msg_imu_current->header.stamp.toSec();
      message.wm << msg_imu_current->angular_velocity.x, msg_imu_current->angular_velocity.y, msg_imu_current->angular_velocity.z;
      message.am << msg_imu_current->linear_acceleration.x, msg_imu_current->linear_acceleration.y, msg_imu_current->linear_acceleration.z;
      // send it to our VIO system
      // ROS_ERROR("%.15f = imu time",msg_imu_current->header.stamp.toSec()-time_init.toSec());
      sys->feed_measurement_imu(message);
      viz->visualize();
      viz->visualize_odometry(message.timestamp);
      // move forward in time
      msg_imu_current = msg_imu_next;
      view_imu_iter++;
      msg_imu_next = view_imu_iter->instantiate<sensor_msgs::Imu>();
      continue;
    }

    // If we are stereo, then we should collect both the left and right
    if (params.state_options.num_cameras == 2) {

      // Now lets do some logic to find two images which are next to each other
      // We want to ensure that our stereo pair are very close to occurring at the same time
      bool have_found_pair = false;
      while (!have_found_pair && view_cameras_iterators.at(0) != view_cameras.at(0)->end() &&
             view_cameras_iterators.at(1) != view_cameras.at(1)->end()) {

        // Get left and right cameras
        auto msg_cam0 = msg_images_current.at(0);
        auto msg_cam1 = msg_images_current.at(1);
        auto msg_cam0_next = msg_images_next.at(0);
        auto msg_cam1_next = msg_images_next.at(1);

        // timestamps
        double time0 = msg_cam0->header.stamp.toSec();
        double time1 = msg_cam1->header.stamp.toSec();
        double time0_next = msg_cam0_next->header.stamp.toSec();
        double time1_next = msg_cam1_next->header.stamp.toSec();

        // We will have a match if the current left and right images are closer then the next one
        // Consider the case that we drop an image:
        //    (L1) L2 (R2) R3 <- current pointers are at L1 and R2
        //    In this case, we dropped the R1 image, thus we should skip the L1 image
        //    We can check to see that L1 is further away compared to L2 from R2
        //    Thus we should skip the L1 frame (advance the bag forward) and check this logic again!
        if (std::abs(time1 - time0) < std::abs(time1_next - time0) && std::abs(time0 - time1) < std::abs(time0_next - time1)) {
          have_found_pair = true;
        } else if (std::abs(time1 - time0) >= std::abs(time1_next - time0)) {
          // ROS_WARN("skipping cam1 (%.4f >= %.4f)",std::abs(time1-time0), std::abs(time1_next-time0));
          msg_images_current.at(1) = msg_images_next.at(1);
          view_cameras_iterators.at(1)++;
          msg_images_next.at(1) = view_cameras_iterators.at(1)->instantiate<sensor_msgs::Image>();
        } else {
          // ROS_WARN("skipping cam0 (%.4f >= %.4f)",std::abs(time0-time1), std::abs(time0_next-time1));
          msg_images_current.at(0) = msg_images_next.at(0);
          view_cameras_iterators.at(0)++;
          msg_images_next.at(0) = view_cameras_iterators.at(0)->instantiate<sensor_msgs::Image>();
        }
      }

      // Break out if we have ended
      if (view_cameras_iterators.at(0) == view_cameras.at(0)->end() || view_cameras_iterators.at(1) == view_cameras.at(1)->end()) {
        break;
      }

      // Check if we should initialize using the groundtruth (always use left)
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() &&
          DatasetReader::get_gt_state(msg_images_current.at(0)->header.stamp.toSec(), imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // Get the image
      cv_bridge::CvImageConstPtr cv_ptr0, cv_ptr1;
      try {
        cv_ptr0 = cv_bridge::toCvShare(msg_images_current.at(0), sensor_msgs::image_encodings::MONO8);
        cv_ptr1 = cv_bridge::toCvShare(msg_images_current.at(1), sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        msg_images_current.at(0) = msg_images_next.at(0);
        view_cameras_iterators.at(0)++;
        msg_images_next.at(0) = view_cameras_iterators.at(0)->instantiate<sensor_msgs::Image>();
        msg_images_current.at(1) = msg_images_next.at(1);
        view_cameras_iterators.at(1)++;
        msg_images_next.at(1) = view_cameras_iterators.at(1)->instantiate<sensor_msgs::Image>();
        continue;
      }

      // Create viomanager message datatype
      ov_core::CameraData message;
      message.timestamp = msg_images_current.at(0)->header.stamp.toSec();
      message.sensor_ids.push_back(0);
      message.sensor_ids.push_back(1);
      message.images.push_back(cv_ptr0->image.clone());
      message.images.push_back(cv_ptr1->image.clone());
      if (sys->get_params().use_mask) {
        message.masks.push_back(sys->get_params().masks.at(0));
        message.masks.push_back(sys->get_params().masks.at(1));
      } else {
        // message.masks.push_back(cv::Mat(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1, cv::Scalar(255)));
        message.masks.push_back(cv::Mat::zeros(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1));
        message.masks.push_back(cv::Mat::zeros(cv_ptr1->image.rows, cv_ptr1->image.cols, CV_8UC1));
      }
      // ROS_ERROR("%.15f = cam %d time",msg_images_current.at(0)->header.stamp.toSec()-time_init.toSec(), 0);
      // ROS_ERROR("%.15f = cam %d time",msg_images_current.at(1)->header.stamp.toSec()-time_init.toSec(), 1);
      // ROS_ERROR("(difference is %.15f)",msg_images_current.at(1)->header.stamp.toSec()-msg_images_current.at(0)->header.stamp.toSec());
      sys->feed_measurement_camera(message);

      // move forward in time
      msg_images_current.at(0) = msg_images_next.at(0);
      view_cameras_iterators.at(0)++;
      msg_images_next.at(0) = view_cameras_iterators.at(0)->instantiate<sensor_msgs::Image>();
      msg_images_current.at(1) = msg_images_next.at(1);
      view_cameras_iterators.at(1)++;
      msg_images_next.at(1) = view_cameras_iterators.at(1)->instantiate<sensor_msgs::Image>();

    } else {

      // Find the camera which should be processed (smallest time)
      int smallest_cam = 0;
      for (int i = 0; i < params.state_options.num_cameras; i++) {
        double time_cam0 = msg_images_current.at(smallest_cam)->header.stamp.toSec();
        double time_cam1 = msg_images_current.at(i)->header.stamp.toSec();
        if (time_cam1 < time_cam0) {
          smallest_cam = i;
        }
      }

      // Check if we should initialize using the groundtruth
      auto msg_camera = msg_images_current.at(smallest_cam);
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(msg_camera->header.stamp.toSec(), imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // Get the image
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(msg_camera, sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        msg_images_current.at(smallest_cam) = msg_images_next.at(smallest_cam);
        view_cameras_iterators.at(smallest_cam)++;
        msg_images_next.at(smallest_cam) = view_cameras_iterators.at(smallest_cam)->instantiate<sensor_msgs::Image>();
        continue;
      }

      // Create viomanager message datatype
      ov_core::CameraData message;
      message.timestamp = msg_camera->header.stamp.toSec();
      message.sensor_ids.push_back(smallest_cam);
      message.images.push_back(cv_ptr->image.clone());
      if (sys->get_params().use_mask) {
        message.masks.push_back(sys->get_params().masks.at(smallest_cam));
      } else {
        message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
      }
      // ROS_ERROR("%.15f = cam %d time",msg_camera->header.stamp.toSec()-time_init.toSec(), smallest_cam);
      sys->feed_measurement_camera(message);

      // move forward
      msg_images_current.at(smallest_cam) = msg_images_next.at(smallest_cam);
      view_cameras_iterators.at(smallest_cam)++;
      msg_images_next.at(smallest_cam) = view_cameras_iterators.at(smallest_cam)->instantiate<sensor_msgs::Image>();
    }
  }

  // Final visualization
  viz->visualize_final();

  // Done!
  return EXIT_SUCCESS;
}
