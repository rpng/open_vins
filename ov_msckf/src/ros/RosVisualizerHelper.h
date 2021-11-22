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

#ifndef OV_MSCKF_ROSVISUALIZER_HELPER_H
#define OV_MSCKF_ROSVISUALIZER_HELPER_H

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "core/VioManager.h"
#include "sim/Simulator.h"

namespace ov_msckf {

/**
 * @brief Helper class that handles some common versions into and out of ROS formats
 */
class RosVisualizerHelper {

public:
#if ROS_AVAILABLE == 1
  /**
   * @brief Will visualize the system if we have new things
   * @param feats Vector of features we will convert into ros format
   * @return ROS pointcloud
   */
  static sensor_msgs::PointCloud2 get_ros_pointcloud(const std::vector<Eigen::Vector3d> &feats) {

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "global";
    cloud.header.stamp = ros::Time::now();
    cloud.width = 3 * feats.size();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(3 * feats.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    // Fill our iterators
    for (const auto &pt : feats) {
      *out_x = (float)pt(0);
      ++out_x;
      *out_y = (float)pt(1);
      ++out_y;
      *out_z = (float)pt(2);
      ++out_z;
    }

    return cloud;
  }

  /**
   * @brief Given a ov_type::PoseJPL this will convert into the ros format.
   *
   * NOTE: frame ids need to be handled externally!
   *
   * @param pose Pose with JPL quaternion (e.g. q_GtoI, p_IinG)
   * @param flip_trans If we should flip / inverse the translation
   * @return TF of our pose in global (e.g. q_ItoG, p_IinG)
   */
  static tf::StampedTransform get_stamped_transform_from_pose(const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

    // Need to flip the transform to the IMU frame
    Eigen::Vector4d q_ItoC = pose->quat();
    Eigen::Vector3d p_CinI = pose->pos();
    if (flip_trans) {
      p_CinI = -pose->Rot().transpose() * pose->pos();
    }

    // publish our transform on TF
    // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
    // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    tf::Quaternion quat(q_ItoC(0), q_ItoC(1), q_ItoC(2), q_ItoC(3));
    trans.setRotation(quat);
    tf::Vector3 orig(p_CinI(0), p_CinI(1), p_CinI(2));
    trans.setOrigin(orig);
    return trans;
  }
#endif

#if ROS_AVAILABLE == 2
  /**
   * @brief Will visualize the system if we have new things
   * @param node ROS2 node pointer
   * @param feats Vector of features we will convert into ros format
   * @return ROS pointcloud
   */
  static sensor_msgs::msg::PointCloud2 get_ros_pointcloud(std::shared_ptr<rclcpp::Node> node, const std::vector<Eigen::Vector3d> &feats) {

    // Declare message and sizes
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "global";
    cloud.header.stamp = node->now();
    cloud.width = 3 * feats.size();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(3 * feats.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    // Fill our iterators
    for (const auto &pt : feats) {
      *out_x = (float)pt(0);
      ++out_x;
      *out_y = (float)pt(1);
      ++out_y;
      *out_z = (float)pt(2);
      ++out_z;
    }

    return cloud;
  }

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
  get_stamped_transform_from_pose(std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

    // Need to flip the transform to the IMU frame
    Eigen::Vector4d q_ItoC = pose->quat();
    Eigen::Vector3d p_CinI = pose->pos();
    if (flip_trans) {
      p_CinI = -pose->Rot().transpose() * pose->pos();
    }

    // publish our transform on TF
    // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
    // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = node->now();
    trans.transform.rotation.x = q_ItoC(0);
    trans.transform.rotation.y = q_ItoC(1);
    trans.transform.rotation.z = q_ItoC(2);
    trans.transform.rotation.w = q_ItoC(3);
    trans.transform.translation.x = p_CinI(0);
    trans.transform.translation.y = p_CinI(1);
    trans.transform.translation.z = p_CinI(2);
    return trans;
  }

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
                                           std::ofstream &of_state_std, std::ofstream &of_state_gt) {

    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;

    // If we have our simulator, then save it to our groundtruth file
    if (sim != nullptr) {

      // Note that we get the true time in the IMU clock frame
      // NOTE: we record both the estimate and groundtruth with the same "true" timestamp if we are doing simulation
      Eigen::Matrix<double, 17, 1> state_gt;
      timestamp_inI = state->_timestamp + sim->get_true_parameters().calib_camimu_dt;
      if (sim->get_state(timestamp_inI, state_gt)) {
        // STATE: write current true state
        of_state_gt.precision(5);
        of_state_gt.setf(std::ios::fixed, std::ios::floatfield);
        of_state_gt << state_gt(0) << " ";
        of_state_gt.precision(6);
        of_state_gt << state_gt(1) << " " << state_gt(2) << " " << state_gt(3) << " " << state_gt(4) << " ";
        of_state_gt << state_gt(5) << " " << state_gt(6) << " " << state_gt(7) << " ";
        of_state_gt << state_gt(8) << " " << state_gt(9) << " " << state_gt(10) << " ";
        of_state_gt << state_gt(11) << " " << state_gt(12) << " " << state_gt(13) << " ";
        of_state_gt << state_gt(14) << " " << state_gt(15) << " " << state_gt(16) << " ";

        // TIMEOFF: Get the current true time offset
        of_state_gt.precision(7);
        of_state_gt << sim->get_true_parameters().calib_camimu_dt << " ";
        of_state_gt.precision(0);
        of_state_gt << state->_options.num_cameras << " ";
        of_state_gt.precision(6);

        // CALIBRATION: Write the camera values to file
        assert(state->_options.num_cameras == sim->get_true_parameters().state_options.num_cameras);
        for (int i = 0; i < state->_options.num_cameras; i++) {
          // Intrinsics values
          of_state_gt << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(0) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(1) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(2) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(3) << " ";
          of_state_gt << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(4) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(5) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(6) << " "
                      << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(7) << " ";
          // Rotation and position
          of_state_gt << sim->get_true_parameters().camera_extrinsics.at(i)(0) << " "
                      << sim->get_true_parameters().camera_extrinsics.at(i)(1) << " "
                      << sim->get_true_parameters().camera_extrinsics.at(i)(2) << " "
                      << sim->get_true_parameters().camera_extrinsics.at(i)(3) << " ";
          of_state_gt << sim->get_true_parameters().camera_extrinsics.at(i)(4) << " "
                      << sim->get_true_parameters().camera_extrinsics.at(i)(5) << " "
                      << sim->get_true_parameters().camera_extrinsics.at(i)(6) << " ";
        }

        // New line
        of_state_gt << endl;
      }
    }

    //==========================================================================
    //==========================================================================
    //==========================================================================

    // Get the covariance of the whole system
    Eigen::MatrixXd cov = StateHelper::get_full_covariance(state);

    // STATE: Write the current state to file
    of_state_est.precision(5);
    of_state_est.setf(std::ios::fixed, std::ios::floatfield);
    of_state_est << timestamp_inI << " ";
    of_state_est.precision(6);
    of_state_est << state->_imu->quat()(0) << " " << state->_imu->quat()(1) << " " << state->_imu->quat()(2) << " "
                 << state->_imu->quat()(3) << " ";
    of_state_est << state->_imu->pos()(0) << " " << state->_imu->pos()(1) << " " << state->_imu->pos()(2) << " ";
    of_state_est << state->_imu->vel()(0) << " " << state->_imu->vel()(1) << " " << state->_imu->vel()(2) << " ";
    of_state_est << state->_imu->bias_g()(0) << " " << state->_imu->bias_g()(1) << " " << state->_imu->bias_g()(2) << " ";
    of_state_est << state->_imu->bias_a()(0) << " " << state->_imu->bias_a()(1) << " " << state->_imu->bias_a()(2) << " ";

    // STATE: Write current uncertainty to file
    of_state_std.precision(5);
    of_state_std.setf(std::ios::fixed, std::ios::floatfield);
    of_state_std << timestamp_inI << " ";
    of_state_std.precision(6);
    int id = state->_imu->q()->id();
    of_state_std << std::sqrt(cov(id + 0, id + 0)) << " " << std::sqrt(cov(id + 1, id + 1)) << " " << std::sqrt(cov(id + 2, id + 2)) << " ";
    id = state->_imu->p()->id();
    of_state_std << std::sqrt(cov(id + 0, id + 0)) << " " << std::sqrt(cov(id + 1, id + 1)) << " " << std::sqrt(cov(id + 2, id + 2)) << " ";
    id = state->_imu->v()->id();
    of_state_std << std::sqrt(cov(id + 0, id + 0)) << " " << std::sqrt(cov(id + 1, id + 1)) << " " << std::sqrt(cov(id + 2, id + 2)) << " ";
    id = state->_imu->bg()->id();
    of_state_std << std::sqrt(cov(id + 0, id + 0)) << " " << std::sqrt(cov(id + 1, id + 1)) << " " << std::sqrt(cov(id + 2, id + 2)) << " ";
    id = state->_imu->ba()->id();
    of_state_std << std::sqrt(cov(id + 0, id + 0)) << " " << std::sqrt(cov(id + 1, id + 1)) << " " << std::sqrt(cov(id + 2, id + 2)) << " ";

    // TIMEOFF: Get the current estimate time offset
    of_state_est.precision(7);
    of_state_est << state->_calib_dt_CAMtoIMU->value()(0) << " ";
    of_state_est.precision(0);
    of_state_est << state->_options.num_cameras << " ";
    of_state_est.precision(6);

    // TIMEOFF: Get the current std values
    if (state->_options.do_calib_camera_timeoffset) {
      of_state_std << std::sqrt(cov(state->_calib_dt_CAMtoIMU->id(), state->_calib_dt_CAMtoIMU->id())) << " ";
    } else {
      of_state_std << 0.0 << " ";
    }
    of_state_std.precision(0);
    of_state_std << state->_options.num_cameras << " ";
    of_state_std.precision(6);

    // CALIBRATION: Write the camera values to file
    for (int i = 0; i < state->_options.num_cameras; i++) {
      // Intrinsics values
      of_state_est << state->_cam_intrinsics.at(i)->value()(0) << " " << state->_cam_intrinsics.at(i)->value()(1) << " "
                   << state->_cam_intrinsics.at(i)->value()(2) << " " << state->_cam_intrinsics.at(i)->value()(3) << " ";
      of_state_est << state->_cam_intrinsics.at(i)->value()(4) << " " << state->_cam_intrinsics.at(i)->value()(5) << " "
                   << state->_cam_intrinsics.at(i)->value()(6) << " " << state->_cam_intrinsics.at(i)->value()(7) << " ";
      // Rotation and position
      of_state_est << state->_calib_IMUtoCAM.at(i)->value()(0) << " " << state->_calib_IMUtoCAM.at(i)->value()(1) << " "
                   << state->_calib_IMUtoCAM.at(i)->value()(2) << " " << state->_calib_IMUtoCAM.at(i)->value()(3) << " ";
      of_state_est << state->_calib_IMUtoCAM.at(i)->value()(4) << " " << state->_calib_IMUtoCAM.at(i)->value()(5) << " "
                   << state->_calib_IMUtoCAM.at(i)->value()(6) << " ";
      // Covariance
      if (state->_options.do_calib_camera_intrinsics) {
        int index_in = state->_cam_intrinsics.at(i)->id();
        of_state_std << std::sqrt(cov(index_in + 0, index_in + 0)) << " " << std::sqrt(cov(index_in + 1, index_in + 1)) << " "
                     << std::sqrt(cov(index_in + 2, index_in + 2)) << " " << std::sqrt(cov(index_in + 3, index_in + 3)) << " ";
        of_state_std << std::sqrt(cov(index_in + 4, index_in + 4)) << " " << std::sqrt(cov(index_in + 5, index_in + 5)) << " "
                     << std::sqrt(cov(index_in + 6, index_in + 6)) << " " << std::sqrt(cov(index_in + 7, index_in + 7)) << " ";
      } else {
        of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
      }
      if (state->_options.do_calib_camera_pose) {
        int index_ex = state->_calib_IMUtoCAM.at(i)->id();
        of_state_std << std::sqrt(cov(index_ex + 0, index_ex + 0)) << " " << std::sqrt(cov(index_ex + 1, index_ex + 1)) << " "
                     << std::sqrt(cov(index_ex + 2, index_ex + 2)) << " ";
        of_state_std << std::sqrt(cov(index_ex + 3, index_ex + 3)) << " " << std::sqrt(cov(index_ex + 4, index_ex + 4)) << " "
                     << std::sqrt(cov(index_ex + 5, index_ex + 5)) << " ";
      } else {
        of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
      }
    }

    // Done with the estimates!
    of_state_est << endl;
    of_state_std << endl;
  }

private:
  // Cannot create this class
  RosVisualizerHelper() = default;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROSVISUALIZER_HELPER_H
