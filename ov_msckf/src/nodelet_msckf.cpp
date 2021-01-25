/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// This is a nodelet vresion of run_subscribe_msckf.cpp, using the
// stereo_image_proc nodelet as a template.


#include <memory>
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"

// using namespace ov_msckf;
using namespace message_filters::sync_policies;

namespace ov_msckf {

class MSCKFNodelet : public nodelet::Nodelet {
private:
    std::vector<boost::shared_ptr<image_transport::ImageTransport>> its_;

    // Subscriptions
    //image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
    // TODO Image info
    //message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_, sub_r_info_;
    std::vector<std::shared_ptr<image_transport::SubscriberFilter>> sub_image_;
    typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    std::vector<boost::shared_ptr<ExactSync>> exact_syncs_;
    std::vector<boost::shared_ptr<ApproximateSync>> approximate_syncs_;

    virtual void onInit();

    void connectCb();

    void stereoCb(const sensor_msgs::ImageConstPtr& msg0,
                const sensor_msgs::ImageConstPtr& msg1, 
                int cam_id0, int cam_id1);
    void monoCb(const sensor_msgs::ImageConstPtr& msg0, 
                int cam_id0);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

    std::shared_ptr<VioManager> sys;
    std::shared_ptr<RosVisualizer> viz;
    VioManagerOptions params; 

    std::string topic_imu;

    ros::Subscriber imu_sub;
    std::vector<image_transport::Subscriber> mono_image_subs;
};

void MSCKFNodelet::onInit() {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    // Synchronize inputs. Topic subscriptions happen on demand in the connection
    // callback. Optionally do approximate synchronization.

    // Create our VIO system
    params = parse_ros_nodehandler(private_nh);
    sys = std::make_shared<VioManager>(params);
    viz = std::make_shared<RosVisualizer>(nh, sys);

    // Our camera topics (left and right stereo)
    private_nh.param<std::string>("topic_imu", topic_imu, "/imu0");

    // imu
    imu_sub = nh.subscribe(topic_imu.c_str(), 9999, &MSCKFNodelet::imuCb, this);

    int queue_size;
    private_nh.param("queue_size", queue_size, 5);
    bool approx;
    private_nh.param("approximate_sync", approx, false);

    std::vector<int> added_cam_ids;
    
    // Logic for sync stereo subscriber
    // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
    for (const auto &pair : params.stereo_pairs) {
        // Read in the topics
        std::string cam_topic0, cam_topic1;
        nh.param<std::string>("topic_camera"+std::to_string(pair.first), cam_topic0, 
                "/cam"+std::to_string(pair.first)+"/image_raw");
        nh.param<std::string>("topic_camera"+std::to_string(pair.second), cam_topic1, 
                "/cam"+std::to_string(pair.second)+"/image_raw");

        auto image_sub0 = std::make_shared<image_transport::SubscriberFilter>();
        auto image_sub1 = std::make_shared<image_transport::SubscriberFilter>();

        if (approx) {
            boost::shared_ptr<ApproximateSync> approximate_sync(
                    new ApproximateSync(ApproximatePolicy(queue_size), 
                    *image_sub0, *image_sub1) );
            approximate_sync->registerCallback(
                    boost::bind(&MSCKFNodelet::stereoCb, this, _1, _2, 
                    pair.first, pair.second));
            approximate_syncs_.push_back(approximate_sync);
        } else {
            boost::shared_ptr<ExactSync> exact_sync(
                    new ExactSync(ExactPolicy(queue_size), 
                    *image_sub0, *image_sub1) );
            exact_sync->registerCallback(
                    boost::bind(&MSCKFNodelet::stereoCb, this, _1, _2,
                    pair.first, pair.second));
            exact_syncs_.push_back(exact_sync);
        }

        image_transport::TransportHints hints("raw", 
                ros::TransportHints(), getPrivateNodeHandle());

        its_.emplace_back(new image_transport::ImageTransport(nh));
        image_sub0->subscribe(*its_.back(), cam_topic0, 1, hints);
        image_sub1->subscribe(*its_.back(), cam_topic1, 1, hints);
        sub_image_.push_back(image_sub0);
        sub_image_.push_back(image_sub1);
        added_cam_ids.push_back(pair.first);
        added_cam_ids.push_back(pair.second);

        ROS_INFO("subscribing to cam (stereo): %s", cam_topic0.c_str());
        ROS_INFO("subscribing to cam (stereo): %s", cam_topic1.c_str());
    }

    // Now we should add any non-stereo callbacks here
    for (int i=0; i<params.state_options.num_cameras; i++) {
        // Skip if already have been added
        if (std::find(added_cam_ids.begin(),added_cam_ids.end(),i)==added_cam_ids.end()) {
            // read in the topic
            std::string cam_topic;
            nh.param<std::string>("topic_camera"+std::to_string(i), cam_topic, 
                    "/cam"+std::to_string(i)+"/image_raw");
            // TODO if this works without keeping it in the scope,
            // does it work also for the synced image transports above?
            image_transport::ImageTransport it(nh);
            image_transport::TransportHints hints("raw", 
                    ros::TransportHints(), getPrivateNodeHandle());
            auto sub = it.subscribe(cam_topic, 1, boost::bind(&MSCKFNodelet::monoCb,
                    this, _1, i), ros::VoidPtr(), hints);
            mono_image_subs.push_back(sub);

            ROS_INFO("subscribing to cam (mono): %s", cam_topic.c_str());
        }
    }
}

void MSCKFNodelet::imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    // convert into correct format
    ov_core::ImuData message;
    message.timestamp = msg->header.stamp.toSec();
    message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // send it to our VIO system
    sys->feed_measurement_imu(message);
    viz->visualize();
    viz->visualize_odometry(message.timestamp);
}

void MSCKFNodelet::monoCb(const sensor_msgs::ImageConstPtr& msg0, int cam_id0) {
    
    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Create the measurement
    ov_core::CameraData message;
    message.timestamp = cv_ptr->header.stamp.toSec();
    message.sensor_ids.push_back(cam_id0);
    message.images.push_back(cv_ptr->image.clone());

    // send it to our VIO system
    sys->feed_measurement_camera(message);
}

void MSCKFNodelet::stereoCb(const sensor_msgs::ImageConstPtr& msg0,
                            const sensor_msgs::ImageConstPtr& msg1,
                            int cam_id0, int cam_id1) {
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

    // Create the measurement
    ov_core::CameraData message;
    message.timestamp = cv_ptr0->header.stamp.toSec();
    message.sensor_ids.push_back(cam_id0);
    message.sensor_ids.push_back(cam_id1);
    message.images.push_back(cv_ptr0->image.clone());
    message.images.push_back(cv_ptr1->image.clone());

    // send it to our VIO system
    sys->feed_measurement_camera(message);
}

}   //namespace ov_msckf

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ov_msckf::MSCKFNodelet,nodelet::Nodelet)
