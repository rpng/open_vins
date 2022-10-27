#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include "butterworth.h"
namespace ovfilter_nodelet_ns
{
class OvfilterNodeletClass : public nodelet::Nodelet
{
public:
    OvfilterNodeletClass();
    ~OvfilterNodeletClass();
    virtual void onInit();
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
};
} // namespace ovmsckf_nodelet_ns
