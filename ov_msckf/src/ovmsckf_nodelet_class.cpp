/*
 * ovmsckf_nodelet_class.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#include "ovmsckf_nodelet_class.h"
#include <pluginlib/class_list_macros.h>
// #include "../../ov_core//src/utils/opencv_yaml_parse.h"
using namespace ov_core;
namespace ovmsckf_nodelet_ns
{
OvmsckfNodeletClass::OvmsckfNodeletClass()
{
  ROS_INFO("<<<OvmsckfNodeletClass Constructor");
  std::string config_path = "/home/chenyu/Desktop/OpenVINS/src/open_vins/ov_msckf/../config/zed_mini/estimator_config.yaml";
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#elif ROS_AVAILABLE == 2
  parser->set_node(node);
#endif
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys = std::make_shared<VioManager>(params);
  #if ROS_AVAILABLE == 1
  viz = std::make_shared<ROS1Visualizer>(nh, sys);
  viz->setup_subscribers(parser);
#elif ROS_AVAILABLE == 2
  viz = std::make_shared<ROS2Visualizer>(node, sys);
  viz->setup_subscribers(parser);
#endif
if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Spin off to ROS
  PRINT_DEBUG("done...spinning to ros\n");
  ROS_INFO("OvmsckfNodeletClass Constructor>>>");
}

OvmsckfNodeletClass::~OvmsckfNodeletClass()
{
  ROS_INFO("OvmsckfNodeletClass Destructor");
}

void OvmsckfNodeletClass::onInit()
{
  NODELET_INFO("CHENYU COMPUTER  %s", __FUNCTION__);
  NODELET_INFO("OvmsckfNodeletClass - %s", __FUNCTION__);
  // ros::NodeHandle nh(getPrivateNodeHandle());
  // sub_imu = nh.subscribe(topic_imu, 1000, &OvmsckfNodeletClass::callback_inertial, this);
  viz->visualize_final();
}


} // namespace ovmsckf_nodelet_ns

PLUGINLIB_EXPORT_CLASS(ovmsckf_nodelet_ns::OvmsckfNodeletClass, nodelet::Nodelet)
