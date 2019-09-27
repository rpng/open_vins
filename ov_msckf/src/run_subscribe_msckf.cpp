/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
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


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;


// Buffer data
double time_buffer = -1;
cv::Mat img0_buffer, img1_buffer;

// Callback functions
void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg);
void callback_monocular(const sensor_msgs::ImageConstPtr& msg0);
void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);



// Main function
int main(int argc, char** argv) {

    // Launch our ros node
    ros::init(argc, argv, "run_subscribe_msckf");
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

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    nh.param<int>("max_cameras", max_cameras, 1);

    // Logic for sync stereo subscriber
    // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
    message_filters::Subscriber<sensor_msgs::Image> image_sub0(nh,topic_camera0.c_str(),1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub1(nh,topic_camera1.c_str(),1);
    //message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(image_sub0,image_sub1,5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), image_sub0,image_sub1);

    // Create subscribers
    ros::Subscriber subimu = nh.subscribe(topic_imu.c_str(), 9999, callback_inertial);
    ros::Subscriber subcam;
    if(max_cameras == 1) {
        ROS_INFO("subscribing to: %s", topic_camera0.c_str());
        subcam = nh.subscribe(topic_camera0.c_str(), 1, callback_monocular);
    } else if(max_cameras == 2) {
        ROS_INFO("subscribing to: %s", topic_camera0.c_str());
        ROS_INFO("subscribing to: %s", topic_camera1.c_str());
        sync.registerCallback(boost::bind(&callback_stereo, _1, _2));
    } else {
        ROS_ERROR("INVALID MAX CAMERAS SELECTED!!!");
        std::exit(EXIT_FAILURE);
    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Spin off to ROS
    ROS_INFO("done...spinning to ros");
    ros::spin();

    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;


}


void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {

    // convert into correct format
    double timem = msg->header.stamp.toSec();
    Eigen::Vector3d wm, am;
    wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // send it to our VIO system
    sys->feed_measurement_imu(timem, wm, am);

}



void callback_monocular(const sensor_msgs::ImageConstPtr& msg0) {


    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Fill our buffer if we have not
    if(img0_buffer.rows == 0) {
        time_buffer = cv_ptr->header.stamp.toSec();
        img0_buffer = cv_ptr->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
    viz->visualize();

    // move buffer forward
    time_buffer = cv_ptr->header.stamp.toSec();
    img0_buffer = cv_ptr->image.clone();

}



void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr0;
    try {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr1;
    try {
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Fill our buffer if we have not
    if(img0_buffer.rows == 0 || img1_buffer.rows == 0) {
        time_buffer = cv_ptr0->header.stamp.toSec();
        img0_buffer = cv_ptr0->image.clone();
        time_buffer = cv_ptr1->header.stamp.toSec();
        img1_buffer = cv_ptr1->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
    viz->visualize();

    // move buffer forward
    time_buffer = cv_ptr0->header.stamp.toSec();
    img0_buffer = cv_ptr0->image.clone();
    time_buffer = cv_ptr1->header.stamp.toSec();
    img1_buffer = cv_ptr1->image.clone();

}


















