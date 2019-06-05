

#include <ros/ros.h>

#include "core/VioManager.h"


using namespace ov_msckf;

VioManager* sys;





// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "test_msckf");
    ros::NodeHandle nh("~");

    // Create our VIO system
    sys = new VioManager(nh);


    ROS_INFO("done! :D");


}


















