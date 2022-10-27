/*
 * ovmsckf_nodelet_class.h
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#ifndef ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_
#define ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

#if ROS_AVAILABLE == 1
#include "ros/ROS1Visualizer.h"
#include <ros/ros.h>
#elif ROS_AVAILABLE == 2
#include "ros/ROS2Visualizer.h"
#include <rclcpp/rclcpp.hpp>
#endif

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "../../ov_filter/include/butterworth/butterworth.h"

using namespace ov_msckf;
using namespace ov_core;
using namespace ov_type;
using namespace ov_init;
namespace ovmsckf_nodelet_ns
{
class OvmsckfNodeletClass : public nodelet::Nodelet
{
public:
    OvmsckfNodeletClass();
    ~OvmsckfNodeletClass();
    void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);
    virtual void onInit();
private:
    std::string config_path = "/home/chenyu/Desktop/ws_openvins/src/open_vins/ov_msckf/../config/zed_mini/estimator_config.yaml";
    std::shared_ptr<ov_core::YamlParser>  parser = std::make_shared<ov_core::YamlParser>(config_path);
    ov_core::ImuData imu_message;
	  ros::Subscriber sub_imu;
    std::shared_ptr<VioManager> sys;
	//   std:: string topic_imu = "/race4/zedm/zed_node/imu/data_raw";
	//   std:: string topic_image_left = "/zedm/zed_node/left/image_rect_gray";
	//   std:: string topic_image_right = "/zedm/zed_node/right/image_rect_gray";
    std::shared_ptr<VioManager> _app;
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>("~");
    #if ROS_AVAILABLE == 1
    std::shared_ptr<ROS1Visualizer> viz;
    #elif ROS_AVAILABLE == 2
    std::shared_ptr<ROS2Visualizer> viz;
    #endif
};
} // namespace ovmsckf_nodelet_ns

#endif /* ovmsckf_nodelet_CLASS_SRC_ovmsckf_nodelet_CLASS_H_ */
