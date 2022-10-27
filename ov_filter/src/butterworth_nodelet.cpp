#include "../include/butterworth/butterworth_nodelet.h"
#include <pluginlib/class_list_macros.h>
namespace ovfilter_nodelet_ns
{
OvfilterNodeletClass::OvfilterNodeletClass()
{
  ROS_INFO("<<<OvfilterNodeletClass Constructor");

  ROS_INFO("OvfilterNodeletClass Constructor>>>");
}

OvfilterNodeletClass::~OvfilterNodeletClass()
{
  ROS_INFO("OvfilterNodeletClass Destructor");
}

void OvfilterNodeletClass::onInit()
{
  NODELET_INFO("CHENYU COMPUTER  %s", __FUNCTION__);

  nh = nodelet::Nodelet::getMTNodeHandle();
  pnh = nodelet::Nodelet::getMTPrivateNodeHandle();
  NODELET_INFO("OvfilterNodeletClass - %s", __FUNCTION__);
  Filter *butterworth_node= new Filter(nh, pnh);
  butterworth_node->setup();
  
}


} // namespace ovfilter_nodelet_ns

PLUGINLIB_EXPORT_CLASS(ovfilter_nodelet_ns::OvfilterNodeletClass, nodelet::Nodelet)
