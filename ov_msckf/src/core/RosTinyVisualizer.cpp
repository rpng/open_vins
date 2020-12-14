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
#include "RosTinyVisualizer.h"

using namespace ov_msckf;


RosTinyVisualizer::RosTinyVisualizer(ros::NodeHandle &nh, Simulator *sim) : _nh(nh), _sim(sim) {
    // Setup our transform broadcaster
    mTfBr = new tf::TransformBroadcaster();

    // 3D points publishing
    pub_points_sim = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_sim", 2);
    ROS_INFO("Publishing: %s", pub_points_sim.getTopic().c_str());
    
    // allocate size of std::vector variable
    poses_gt.resize(_sim->get_true_parameters().num_imus);

    // Groundtruth publishers
    // zero index for the base IMU (default), others for additionals
    for (int imuid=0; imuid<_sim->get_true_parameters().num_imus; imuid++) {
        std::string pub_posegt_name = "/ov_msckf/posegt";
        std::string pub_pathgt_name = "/ov_msckf/pathgt";
        if (imuid != 0) {
            pub_posegt_name.append("_U");
            pub_pathgt_name.append("_U");
            pub_posegt_name.append(std::to_string(imuid));
            pub_pathgt_name.append(std::to_string(imuid));
        }
        pub_posegt.push_back(nh.advertise<geometry_msgs::PoseStamped>(pub_posegt_name, 2));
        ROS_INFO("Publishing: %s", pub_posegt.at(imuid).getTopic().c_str());
        pub_pathgt.push_back(nh.advertise<nav_msgs::Path>(pub_pathgt_name, 2));
        ROS_INFO("Publishing: %s", pub_pathgt.at(imuid).getTopic().c_str());
    }
    
    // option to enable publishing of global to IMU transformation
    nh.param<bool>("publish_global_to_imu_tf", publish_global2imu_tf, true);
    nh.param<bool>("publish_calibration_tf", publish_calibration_tf, true);

    // Load groundtruth if we have it and are not doing simulation
    if (nh.hasParam("path_gt") && _sim==nullptr) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        DatasetReader::load_gt_file(path_to_gt, gt_states);
    }

}

void RosTinyVisualizer::visualize(double timestamp) {

    // Save the start time of this dataset
    if(!start_time_set) {
        rT1 =  boost::posix_time::microsec_clock::local_time();
        start_time_set = true;
    }

    // Publish gt if we have it
    publish_groundtruth(timestamp);
    publish_features();

}

void RosTinyVisualizer::publish_groundtruth(double timestamp_inI) {
    // Our groundtruth state
    Eigen::Matrix<double,17,1> state_gt;
    if(!_sim->get_state(timestamp_inI, state_gt))
        return;
    
    // base imu pose
    Eigen::Vector4d q_GtoI = state_gt.block(1,0,4,1);
    Eigen::Vector3d p_IinG = state_gt.block(5,0,3,1);
    Eigen::Matrix<double,3,3> R_GtoI = quat_2_Rot(q_GtoI);

    for (int imuid = 0; imuid < _sim->get_true_parameters().num_imus; imuid ++) {
        Eigen::Matrix<double,3,3> R_ItoU = quat_2_Rot(_sim->get_true_parameters().imu_extrinsics.at(imuid).block(0,0,4,1));
        Eigen::Matrix<double,3,1> p_IinU = _sim->get_true_parameters().imu_extrinsics.at(imuid).block(4,0,3,1);
        
        Eigen::Matrix<double,3,1> p_UinI = - R_ItoU.transpose() * p_IinU;

        // calculate each imu pose
        Eigen::Matrix3d R_GtoU = R_ItoU * R_GtoI;
        Eigen::Vector4d q_GtoU = rot_2_quat(R_GtoU);
        Eigen::Vector3d p_UinG = p_IinG + R_GtoI.transpose() * p_UinI;

        // Create pose of IMUs
        geometry_msgs::PoseStamped poseIinM;
        poseIinM.header.stamp = ros::Time(timestamp_inI);
        poseIinM.header.seq = poses_seq_gt;
        poseIinM.header.frame_id = "global";
        poseIinM.pose.orientation.x = q_GtoU(0);
        poseIinM.pose.orientation.y = q_GtoU(1);
        poseIinM.pose.orientation.z = q_GtoU(2);
        poseIinM.pose.orientation.w = q_GtoU(3);
        poseIinM.pose.position.x = p_UinG(0);
        poseIinM.pose.position.y = p_UinG(1);
        poseIinM.pose.position.z = p_UinG(2);
        pub_posegt.at(imuid).publish(poseIinM);

        // Append to our pose vector
        poses_gt.at(imuid).push_back(poseIinM);

        // Create our path (imu)
        // NOTE: We downsample the number of poses as needed to prevent rviz crashes
        // NOTE: https://github.com/ros-visualization/rviz/issues/1107
        nav_msgs::Path arrIMU;
        arrIMU.header.stamp = ros::Time::now();
        arrIMU.header.seq = poses_seq_gt;
        arrIMU.header.frame_id = "global";
        for(size_t i=0; i<poses_gt.at(imuid).size(); i+=std::floor(poses_gt.at(imuid).size()/16384.0)+1) {
            arrIMU.poses.push_back(poses_gt.at(imuid).at(i));
        }
        arrIMU.poses = poses_gt.at(imuid);
        pub_pathgt.at(imuid).publish(arrIMU);

        // Publish our transform on TF
        tf::StampedTransform trans;
        trans.stamp_ = ros::Time::now();
        trans.frame_id_ = "global";
        trans.child_frame_id_ = "truth";
        if (imuid != 0) trans.child_frame_id_ = trans.child_frame_id_ + "_U" + std::to_string(imuid);
        tf::Quaternion quat(q_GtoU(0), q_GtoU(1), q_GtoU(2), q_GtoU(3));
        trans.setRotation(quat);
        tf::Vector3 orig(p_UinG(0), p_UinG(1), p_UinG(2));
        trans.setOrigin(orig);
        if(publish_global2imu_tf) {
            mTfBr->sendTransform(trans);
        }
    }

    // Move them forward in time
    poses_seq_gt++;
}

void RosTinyVisualizer::publish_features() {
    // Get our good features
    std::unordered_map<size_t,Eigen::Vector3d> feats_sim = _sim->get_map();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SIM;
    cloud_SIM.header.frame_id = "global";
    cloud_SIM.header.stamp = ros::Time::now();
    cloud_SIM.width  = 3*feats_sim.size();
    cloud_SIM.height = 1;
    cloud_SIM.is_bigendian = false;
    cloud_SIM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SIM(cloud_SIM);
    modifier_SIM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SIM.resize(3*feats_sim.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SIM(cloud_SIM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SIM(cloud_SIM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SIM(cloud_SIM, "z");

    // Fill our iterators
    for(const auto &pt : feats_sim) {
        *out_x_SIM = pt.second(0); ++out_x_SIM;
        *out_y_SIM = pt.second(1); ++out_y_SIM;
        *out_z_SIM = pt.second(2); ++out_z_SIM;
    }

    // Publish
    pub_points_sim.publish(cloud_SIM);

}
