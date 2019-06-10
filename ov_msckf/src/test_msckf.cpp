

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "core/VioManager.h"
#include "core/RosVisualizer.h"


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;




// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "test_msckf");
    ros::NodeHandle nh("~");

    // Create our VIO system
    sys = new VioManager(nh);
    viz = new RosVisualizer(nh, sys);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our camera topics (left and right stereo)
    std::string topic_imu;
    std::string topic_camera0;
    std::string topic_camera1;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");

    // Location of the ROS bag we want to read in
    std::string path_to_bag, path_to_gt;
    //nhPrivate.param<std::string>("path_bag", path_to_bag, "/home/keck/catkin_ws/V1_01_easy.bag");
    nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
    nh.param<std::string>("path_gt", path_to_gt, "/home/patrick/datasets/eth/gt_files/V1_01_easy.csv");
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());
    ROS_INFO("gt file path is: %s", path_to_gt.c_str());

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f",bag_start);
    ROS_INFO("bag duration: %.1f",bag_durr);

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    nh.param<int>("max_cameras", max_cameras, 1);


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
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }


    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
    double time = time_init.toSec();
    double time_buffer = time_init.toSec();


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Step through the rosbag
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != NULL && m.getTopic() == topic_imu) {
            // convert into correct format
            double timem = (*s2).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
            am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
            // send it to our VIO system
            sys->feed_measurement_imu(timem, wm, am);
        }

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != NULL && m.getTopic() == topic_camera0) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s0, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable
            has_left = true;
            img0 = cv_ptr->image.clone();
            time = cv_ptr->header.stamp.toSec();
        }

        // Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != NULL && m.getTopic() == topic_camera1) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s1, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable
            has_right = true;
            img1 = cv_ptr->image.clone();
            time = cv_ptr->header.stamp.toSec();
        }


        // Fill our buffer if we have not
        if(has_left && img0_buffer.rows == 0) {
            has_left = false;
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // Fill our buffer if we have not
        if(has_right && img1_buffer.rows == 0) {
            has_right = false;
            time_buffer = time;
            img1_buffer = img1.clone();
        }


        // If we are in monocular mode, then we should process the left if we have it
        if(max_cameras==1 && has_left) {
            // process once we have initialized with the GT
            sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
        }


        // If we are in stereo mode and have both left and right, then process
        if(max_cameras==2 && has_left && has_right) {
            // process once we have initialized with the GT
            sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
            // visualize
            viz->visualize();
            // reset bools
            has_left = false;
            has_right = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
            img1_buffer = img1.clone();
        }

    }

    // Finally delete our system
    delete sys;


    // Done!
    return EXIT_SUCCESS;

}


















