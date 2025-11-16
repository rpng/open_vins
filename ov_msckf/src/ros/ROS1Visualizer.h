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

#ifndef OV_MSCKF_ROS1VISUALIZER_H
#define OV_MSCKF_ROS1VISUALIZER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>

#include <Eigen/Eigen>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

namespace ov_core {
class YamlParser;
struct CameraData;
} // namespace ov_core

namespace ov_msckf {

class VioManager;
class Simulator;

/**
 * @brief Helper class that will publish results onto the ROS framework.
 *
 * Also save to file the current total state and covariance along with the groundtruth if we are simulating.
 * We visualize the following things:
 * - State of the system on TF, pose message, and path
 * - Image of our tracker
 * - Our different features (SLAM, MSCKF, ARUCO)
 * - Groundtruth trajectory if we have it
 */
class ROS1Visualizer {

public:
  /**
   * @brief Default constructor
   * @param nh ROS node handler
   * @param app Core estimator manager
   * @param sim Simulator if we are simulating
   */
  ROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim = nullptr);

  /**
   * @brief Will setup ROS subscribers and callbacks
   * @param parser Configuration file parser
   */
  void setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser);

  /**
   * @brief Will visualize the system if we have new things
   */
  void visualize();

  /**
   * @brief Will publish our odometry message for the current timestep.
   * This will take the current state estimate and get the propagated pose to the desired time.
   * This can be used to get pose estimates on systems which require high frequency pose estimates.
   */
  void visualize_odometry(double timestamp);

  /**
   * @brief After the run has ended, print results
   */
  void visualize_final();

  /// Callback for inertial information
  void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);

  /// Callback for monocular cameras information
  void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0);

  /// Callback for synchronized stereo camera information
  void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1);

protected:
  /// Publish the current state
  void publish_state();

  /// Publish the active tracking image
  void publish_images();

  /// Publish current features
  void publish_features();

  /// Publish groundtruth (if we have it)
  void publish_groundtruth();

  /// Publish loop-closure information of current pose and active track information
  void publish_loopclosure_information();

  /// Global node handler
  std::shared_ptr<ros::NodeHandle> _nh;

  /// Core application of the filter system
  std::shared_ptr<VioManager> _app;

  /// Simulator (is nullptr if we are not sim'ing)
  std::shared_ptr<Simulator> _sim;

  // Our publishers
  image_transport::Publisher it_pub_tracks, it_pub_loop_img_depth, it_pub_loop_img_depth_color;
  ros::Publisher pub_poseimu, pub_odomimu, pub_pathimu;
  ros::Publisher pub_points_msckf, pub_points_slam, pub_points_aruco, pub_points_sim;
  ros::Publisher pub_loop_pose, pub_loop_point, pub_loop_extrinsic, pub_loop_intrinsics;
  std::shared_ptr<tf::TransformBroadcaster> mTfBr;

  // Our subscribers and camera synchronizers
  ros::Subscriber sub_imu;
  std::vector<ros::Subscriber> subs_cam;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  std::vector<std::shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> sync_subs_cam;

  // For path viz
  unsigned int poses_seq_imu = 0;
  std::vector<geometry_msgs::PoseStamped> poses_imu;

  // Groundtruth infomation
  ros::Publisher pub_pathgt, pub_posegt;
  double summed_mse_ori = 0.0;
  double summed_mse_pos = 0.0;
  double summed_nees_ori = 0.0;
  double summed_nees_pos = 0.0;
  size_t summed_number = 0;

  // Start and end timestamps
  bool start_time_set = false;
  boost::posix_time::ptime rT1, rT2;

  // Thread atomics
  std::atomic<bool> thread_update_running;

  /// Queue up camera measurements sorted by time and trigger once we have
  /// exactly one IMU measurement with timestamp newer than the camera measurement
  /// This also handles out-of-order camera measurements, which is rare, but
  /// a nice feature to have for general robustness to bad camera drivers.
  std::deque<ov_core::CameraData> camera_queue;
  std::mutex camera_queue_mtx;

  // Last camera message timestamps we have received (mapped by cam id)
  std::map<int, double> camera_last_timestamp;

  // Last timestamp we visualized at
  double last_visualization_timestamp = 0;
  double last_visualization_timestamp_image = 0;

  // Our groundtruth states
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

  // For path viz
  unsigned int poses_seq_gt = 0;
  std::vector<geometry_msgs::PoseStamped> poses_gt;
  bool publish_global2imu_tf = true;
  bool publish_calibration_tf = true;

  // Files and if we should save total state
  bool save_total_state = false;
  std::ofstream of_state_est, of_state_std, of_state_gt;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS1VISUALIZER_H
