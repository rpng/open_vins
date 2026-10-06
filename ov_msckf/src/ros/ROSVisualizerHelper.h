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

#ifndef OV_MSCKF_ROSVISUALIZER_HELPER_H
#define OV_MSCKF_ROSVISUALIZER_HELPER_H

#include <Eigen/Eigen>

#if ROS_AVAILABLE == 1
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#elif ROS_AVAILABLE == 2
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace ov_type {
class PoseJPL;
}

namespace ov_msckf {

class State;
class VioManager;
class Simulator;

/**
 * @brief Helper class that handles some common versions into and out of ROS formats
 */
class ROSVisualizerHelper {

public:
#if ROS_AVAILABLE == 1
  /**
   * @brief Will visualize the system if we have new things
   * @param feats Vector of features we will convert into ros format
   * @return ROS pointcloud
   */
  static sensor_msgs::PointCloud2 get_ros_pointcloud(const std::vector<Eigen::Vector3d> &feats);

  /**
   * @brief Given a ov_type::PoseJPL this will convert into the ros format.
   *
   * NOTE: frame ids need to be handled externally!
   *
   * @param pose Pose with JPL quaternion (e.g. q_GtoI, p_IinG)
   * @param flip_trans If we should flip / inverse the translation
   * @return TF of our pose in global (e.g. q_ItoG, p_IinG)
   */
  static tf::StampedTransform get_stamped_transform_from_pose(const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);
#endif

#if ROS_AVAILABLE == 2
  /**
   * @brief Will visualize the system if we have new things
   * @param node ROS2 node pointer
   * @param feats Vector of features we will convert into ros format
   * @return ROS pointcloud
   */
  static sensor_msgs::msg::PointCloud2 get_ros_pointcloud(std::shared_ptr<rclcpp::Node> node, const std::vector<Eigen::Vector3d> &feats);

  /**
   * @brief Given a ov_type::PoseJPL this will convert into the ros format.
   *
   * NOTE: frame ids need to be handled externally!
   *
   * @param node ROS2 node pointer
   * @param pose Pose with JPL quaternion (e.g. q_GtoI, p_IinG)
   * @param flip_trans If we should flip / inverse the translation
   * @return TF of our pose in global (e.g. q_ItoG, p_IinG)
   */
  static geometry_msgs::msg::TransformStamped
  get_stamped_transform_from_pose(std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);

  /**
   * @brief Helper function that converts time in seconds to our rclcpp timestamp
   * @param seconds Time in seconds
   * @return Return ROS2 header timestamp
   */
  static rclcpp::Time get_time_from_seconds(double seconds) {

    // ROS2 time class has no double constructor
    // Extract compatible time from timestamp using ros1 implementation for now
    uint32_t sec, nsec;
    int64_t sec64 = static_cast<int64_t>(floor(seconds));
    if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
      throw std::runtime_error("Time is out of dual 32-bit range");
    sec = static_cast<uint32_t>(sec64);
    nsec = static_cast<uint32_t>(std::round((seconds - sec) * 1e9));

    // Avoid rounding errors
    sec += (nsec / 1000000000ul);
    nsec %= 1000000000ul;
    return rclcpp::Time(sec, nsec);
  }

#endif

  /**
   * @brief Save current estimate state and groundtruth including calibration
   * @param state Pointer to the state
   * @param sim Pointer to the simulator (or null)
   * @param of_state_est Output file for state estimate
   * @param of_state_std Output file for covariance
   * @param of_state_gt Output file for groundtruth (if we have it from sim)
   */
  static void sim_save_total_state_to_file(std::shared_ptr<State> state, std::shared_ptr<Simulator> sim, std::ofstream &of_state_est,
                                           std::ofstream &of_state_std, std::ofstream &of_state_gt);

private:
  // Cannot create this class
  ROSVisualizerHelper() = default;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROSVISUALIZER_HELPER_H
