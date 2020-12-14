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
#ifndef OV_MSCKF_ROSTINYVISUALIZER_H
#define OV_MSCKF_ROSTINYVISUALIZER_H



#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>

#include "sim/SimulatorMultiIMU.h"
#include "utils/dataset_reader.h"


namespace ov_msckf {


/**
 * @brief Lightweight version of RosVisualizer. 
 * This does not carry out state estimation.
 */
class RosTinyVisualizer {

    public:
        RosTinyVisualizer(ros::NodeHandle &nh, Simulator* sim);
        void visualize(double timestamp);
    
    protected:
        void publish_groundtruth(double timestamp_inI);
        void publish_features();

        /// ROS node handle that we publish onto
        ros::NodeHandle _nh;
        /// Simulator (is nullptr if we are not sim'ing)
        Simulator* _sim;

        // Our publishers
        std::vector<ros::Publisher> pub_pathgt, pub_posegt;
        ros::Publisher pub_points_sim;
        tf::TransformBroadcaster *mTfBr;

        // Start and end timestamps
        bool start_time_set = false;
        boost::posix_time::ptime rT1, rT2;

        // Our groundtruth states
        std::map<double, Eigen::Matrix<double,17,1>> gt_states;

        // For path viz
        unsigned int poses_seq_gt = 0;
        std::vector<vector<geometry_msgs::PoseStamped>> poses_gt;
        bool publish_global2imu_tf = true;
        bool publish_calibration_tf = true;

};

}


#endif //OV_MSCKF_ROSTINYVISUALIZER_H
