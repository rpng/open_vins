

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include "utils/Recorder.h"



int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "pose_to_file");
    ros::NodeHandle nh("~");

    // Get parameters to subscribe
    std::string topic, topic_type, fileoutput;
    nh.getParam("topic", topic);
    nh.getParam("topic_type", topic_type);
    nh.getParam("output", fileoutput);

    // Debug
    ROS_INFO("Done reading config values");
    ROS_INFO(" - topic = %s", topic.c_str());
    ROS_INFO(" - topic_type = %s", topic_type.c_str());
    ROS_INFO(" - file = %s", fileoutput.c_str());

    // Create the recorder object
    ov_eval::Recorder recorder(fileoutput);

    // Subscribe to topic
    ros::Subscriber sub;
    if (topic_type == std::string("PoseWithCovarianceStamped")) {
        sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_posecovariance, &recorder);
    } else if (topic_type == std::string("PoseStamped")) {
        sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_pose, &recorder);
    } else if (topic_type == std::string("TransformStamped")) {
        sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_transform, &recorder);
    } else if(topic_type == std::string("Odometry")) {
        sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_odometry, &recorder);
    } else {
        ROS_ERROR("The specified topic type is not supported");
        ROS_ERROR("topic_type = %s", topic_type.c_str());
        ROS_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
        std::exit(EXIT_FAILURE);
    }

    // Done!
    ros::spin();
    return EXIT_SUCCESS;

}



