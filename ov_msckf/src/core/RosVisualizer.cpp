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
#include "RosVisualizer.h"


using namespace ov_msckf;



RosVisualizer::RosVisualizer(ros::NodeHandle &nh, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim) : _app(app), _sim(sim) {


    // Setup our transform broadcaster
    mTfBr = new tf::TransformBroadcaster();

    // Setup pose and path publisher
    pub_poseimu = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 2);
    ROS_INFO("Publishing: %s", pub_poseimu.getTopic().c_str());
    pub_odomimu = nh.advertise<nav_msgs::Odometry>("/ov_msckf/odomimu", 2);
    ROS_INFO("Publishing: %s", pub_odomimu.getTopic().c_str());
    pub_pathimu = nh.advertise<nav_msgs::Path>("/ov_msckf/pathimu", 2);
    ROS_INFO("Publishing: %s", pub_pathimu.getTopic().c_str());

    // 3D points publishing
    pub_points_msckf = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_msckf", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_slam = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_slam", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_aruco = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_aruco", 2);
    ROS_INFO("Publishing: %s", pub_points_aruco.getTopic().c_str());
    pub_points_sim = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_sim", 2);
    ROS_INFO("Publishing: %s", pub_points_sim.getTopic().c_str());

    // Our tracking image
    pub_tracks = nh.advertise<sensor_msgs::Image>("/ov_msckf/trackhist", 2);
    ROS_INFO("Publishing: %s", pub_tracks.getTopic().c_str());

    // Groundtruth publishers
    pub_posegt = nh.advertise<geometry_msgs::PoseStamped>("/ov_msckf/posegt", 2);
    ROS_INFO("Publishing: %s", pub_posegt.getTopic().c_str());
    pub_pathgt = nh.advertise<nav_msgs::Path>("/ov_msckf/pathgt", 2);
    ROS_INFO("Publishing: %s", pub_pathgt.getTopic().c_str());

    // Loop closure publishers
    pub_loop_pose = nh.advertise<nav_msgs::Odometry>("/ov_msckf/loop_pose", 2);
    pub_loop_point = nh.advertise<sensor_msgs::PointCloud>("/ov_msckf/loop_feats", 2);
    pub_loop_extrinsic = nh.advertise<nav_msgs::Odometry>("/ov_msckf/loop_extrinsic", 2);
    pub_loop_intrinsics = nh.advertise<sensor_msgs::CameraInfo>("/ov_msckf/loop_intrinsics", 2);
    pub_loop_img_depth = nh.advertise<sensor_msgs::Image>("/ov_msckf/loop_depth", 2);
    pub_loop_img_depth_color = nh.advertise<sensor_msgs::Image>("/ov_msckf/loop_depth_colored", 2);

    // option to enable publishing of global to IMU transformation
    nh.param<bool>("publish_global_to_imu_tf", publish_global2imu_tf, true);
    nh.param<bool>("publish_calibration_tf", publish_calibration_tf, true);

    // Load groundtruth if we have it and are not doing simulation
    if (nh.hasParam("path_gt") && _sim==nullptr) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        DatasetReader::load_gt_file(path_to_gt, gt_states);
    }

    // Load if we should save the total state to file
    nh.param<bool>("save_total_state", save_total_state, false);

    // If the file is not open, then open the file
    if(save_total_state) {

        // files we will open
        std::string filepath_est, filepath_std, filepath_gt;
        nh.param<std::string>("filepath_est", filepath_est, "state_estimate.txt");
        nh.param<std::string>("filepath_std", filepath_std, "state_deviation.txt");
        nh.param<std::string>("filepath_gt", filepath_gt, "state_groundtruth.txt");

        // If it exists, then delete it
        if(boost::filesystem::exists(filepath_est))
            boost::filesystem::remove(filepath_est);
        if(boost::filesystem::exists(filepath_std))
            boost::filesystem::remove(filepath_std);

        // Open the files
        of_state_est.open(filepath_est.c_str());
        of_state_std.open(filepath_std.c_str());
        of_state_est << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;
        of_state_std << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;

        // Groundtruth if we are simulating
        if(_sim != nullptr) {
            if(boost::filesystem::exists(filepath_gt))
                boost::filesystem::remove(filepath_gt);
            of_state_gt.open(filepath_gt.c_str());
            of_state_gt << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;
        }

    }

}



void RosVisualizer::visualize() {

    // Return if we have already visualized
    if(last_visualization_timestamp == _app->get_state()->_timestamp)
        return;
    last_visualization_timestamp = _app->get_state()->_timestamp;

    // Start timing
    boost::posix_time::ptime rT0_1, rT0_2;
    rT0_1 =  boost::posix_time::microsec_clock::local_time();

    // publish current image
    publish_images();

    // Return if we have not inited
    if(!_app->initialized())
        return;

    // Save the start time of this dataset
    if(!start_time_set) {
        rT1 =  boost::posix_time::microsec_clock::local_time();
        start_time_set = true;
    }

    // publish state
    publish_state();

    // publish points
    publish_features();

    // Publish gt if we have it
    publish_groundtruth();

    // Publish keyframe information
    publish_loopclosure_information();

    // save total state
    if(save_total_state)
        sim_save_total_state_to_file();

    // Print how much time it took to publish / displaying things
    rT0_2 =  boost::posix_time::microsec_clock::local_time();
    double time_total = (rT0_2-rT0_1).total_microseconds() * 1e-6;
    printf(BLUE "[TIME]: %.4f seconds for visualization\n" RESET, time_total);

}



void RosVisualizer::visualize_odometry(double timestamp) {

    // Check if we have subscribers
    if(pub_odomimu.getNumSubscribers()==0)
        return;

    // Return if we have not inited and a second has passes
    if(!_app->initialized() || (timestamp - _app->initialized_time()) < 1)
        return;

    // Get fast propagate state at the desired timestamp
    std::shared_ptr<State> state = _app->get_state();
    Eigen::Matrix<double,13,1> state_plus = Eigen::Matrix<double,13,1>::Zero();
    _app->get_propagator()->fast_state_propagate(state, timestamp, state_plus);

    // Our odometry message
    nav_msgs::Odometry odomIinM;
    odomIinM.header.stamp = ros::Time(timestamp);
    odomIinM.header.frame_id = "global";

    // The POSE component (orientation and position)
    odomIinM.pose.pose.orientation.x = state_plus(0);
    odomIinM.pose.pose.orientation.y = state_plus(1);
    odomIinM.pose.pose.orientation.z = state_plus(2);
    odomIinM.pose.pose.orientation.w = state_plus(3);
    odomIinM.pose.pose.position.x = state_plus(4);
    odomIinM.pose.pose.position.y = state_plus(5);
    odomIinM.pose.pose.position.z = state_plus(6);

    // Finally set the covariance in the message (in the order position then orientation as per ros convention)
    // TODO: this currently is an approximation since this should actually evolve over our propagation period
    // TODO: but to save time we only propagate the mean and not the uncertainty, but maybe we should try to prop the covariance?
    std::vector<std::shared_ptr<Type>> statevars;
    statevars.push_back(state->_imu->pose()->p());
    statevars.push_back(state->_imu->pose()->q());
    Eigen::Matrix<double,6,6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(),statevars);
    for(int r=0; r<6; r++) {
        for(int c=0; c<6; c++) {
            odomIinM.pose.covariance[6*r+c] = covariance_posori(r,c);
        }
    }

    // The TWIST component (angular and linear velocities)
    odomIinM.child_frame_id = "imu";
    odomIinM.twist.twist.linear.x = state_plus(7);
    odomIinM.twist.twist.linear.y = state_plus(8);
    odomIinM.twist.twist.linear.z = state_plus(9);
    odomIinM.twist.twist.angular.x = state_plus(10); // we do not estimate this...
    odomIinM.twist.twist.angular.y = state_plus(11); // we do not estimate this...
    odomIinM.twist.twist.angular.z = state_plus(12); // we do not estimate this...

    // Velocity covariance (linear then angular)
    // TODO: this currently is an approximation since this should actually evolve over our propagation period
    // TODO: but to save time we only propagate the mean and not the uncertainty, but maybe we should try to prop the covariance?
    // TODO: can we come up with an approx covariance for the omega based on the w_hat = w_m - b_w ??
    statevars.clear();
    statevars.push_back(state->_imu->v());
    Eigen::Matrix<double,6,6> covariance_linang = INFINITY*Eigen::Matrix<double,6,6>::Identity();
    covariance_linang.block(0,0,3,3) = StateHelper::get_marginal_covariance(_app->get_state(),statevars);
    for(int r=0; r<6; r++) {
        for(int c=0; c<6; c++) {
            odomIinM.twist.covariance[6*r+c] = (std::isnan(covariance_linang(r,c))) ? 0 : covariance_linang(r,c);
        }
    }

    // Finally, publish the resulting odometry message
    pub_odomimu.publish(odomIinM);


}



void RosVisualizer::visualize_final() {

    // Final time offset value
    if(_app->get_state()->_options.do_calib_camera_timeoffset) {
        printf(REDPURPLE "camera-imu timeoffset = %.5f\n\n" RESET,_app->get_state()->_calib_dt_CAMtoIMU->value()(0));
    }

    // Final camera intrinsics
    if(_app->get_state()->_options.do_calib_camera_intrinsics) {
        for(int i=0; i<_app->get_state()->_options.num_cameras; i++) {
            std::shared_ptr<Vec> calib = _app->get_state()->_cam_intrinsics.at(i);
            printf(REDPURPLE "cam%d intrinsics:\n" RESET, (int)i);
            printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n" RESET,calib->value()(0),calib->value()(1),calib->value()(2),calib->value()(3));
            printf(REDPURPLE "%.5f,%.5f,%.5f,%.5f\n\n" RESET,calib->value()(4),calib->value()(5),calib->value()(6),calib->value()(7));
        }
    }

    // Final camera extrinsics
    if(_app->get_state()->_options.do_calib_camera_pose) {
        for(int i=0; i<_app->get_state()->_options.num_cameras; i++) {
            std::shared_ptr<PoseJPL> calib = _app->get_state()->_calib_IMUtoCAM.at(i);
            Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
            T_CtoI.block(0,0,3,3) = quat_2_Rot(calib->quat()).transpose();
            T_CtoI.block(0,3,3,1) = -T_CtoI.block(0,0,3,3)*calib->pos();
            printf(REDPURPLE "T_C%dtoI:\n" RESET,i);
            printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET,T_CtoI(0,0),T_CtoI(0,1),T_CtoI(0,2),T_CtoI(0,3));
            printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET,T_CtoI(1,0),T_CtoI(1,1),T_CtoI(1,2),T_CtoI(1,3));
            printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET,T_CtoI(2,0),T_CtoI(2,1),T_CtoI(2,2),T_CtoI(2,3));
            printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n\n" RESET,T_CtoI(3,0),T_CtoI(3,1),T_CtoI(3,2),T_CtoI(3,3));
        }
    }

    // Publish RMSE if we have it
    if(!gt_states.empty()) {
        printf(REDPURPLE "RMSE average: %.3f (deg) orientation\n" RESET,summed_rmse_ori/summed_number);
        printf(REDPURPLE "RMSE average: %.3f (m) position\n\n" RESET,summed_rmse_pos/summed_number);
    }

    // Publish RMSE and NEES if doing simulation
    if(_sim != nullptr) {
        printf(REDPURPLE "RMSE average: %.3f (deg) orientation\n" RESET,summed_rmse_ori/summed_number);
        printf(REDPURPLE "RMSE average: %.3f (m) position\n\n" RESET,summed_rmse_pos/summed_number);
        printf(REDPURPLE "NEES average: %.3f (deg) orientation\n" RESET,summed_nees_ori/summed_number);
        printf(REDPURPLE "NEES average: %.3f (m) position\n\n" RESET,summed_nees_pos/summed_number);
    }

    // Print the total time
    rT2 =  boost::posix_time::microsec_clock::local_time();
    printf(REDPURPLE "TIME: %.3f seconds\n\n" RESET,(rT2-rT1).total_microseconds()*1e-6);

}



void RosVisualizer::publish_state() {

    // Get the current state
    std::shared_ptr<State> state = _app->get_state();

    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;

    // Create pose of IMU (note we use the bag time)
    geometry_msgs::PoseWithCovarianceStamped poseIinM;
    poseIinM.header.stamp = ros::Time(timestamp_inI);
    poseIinM.header.seq = poses_seq_imu;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.pose.orientation.x = state->_imu->quat()(0);
    poseIinM.pose.pose.orientation.y = state->_imu->quat()(1);
    poseIinM.pose.pose.orientation.z = state->_imu->quat()(2);
    poseIinM.pose.pose.orientation.w = state->_imu->quat()(3);
    poseIinM.pose.pose.position.x = state->_imu->pos()(0);
    poseIinM.pose.pose.position.y = state->_imu->pos()(1);
    poseIinM.pose.pose.position.z = state->_imu->pos()(2);

    // Finally set the covariance in the message (in the order position then orientation as per ros convention)
    std::vector<std::shared_ptr<Type>> statevars;
    statevars.push_back(state->_imu->pose()->p());
    statevars.push_back(state->_imu->pose()->q());
    Eigen::Matrix<double,6,6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(),statevars);
    for(int r=0; r<6; r++) {
        for(int c=0; c<6; c++) {
            poseIinM.pose.covariance[6*r+c] = covariance_posori(r,c);
        }
    }
    pub_poseimu.publish(poseIinM);


    //=========================================================
    //=========================================================

    // Append to our pose vector
    geometry_msgs::PoseStamped posetemp;
    posetemp.header = poseIinM.header;
    posetemp.pose = poseIinM.pose.pose;
    poses_imu.push_back(posetemp);

    // Create our path (imu)
    // NOTE: We downsample the number of poses as needed to prevent rviz crashes
    // NOTE: https://github.com/ros-visualization/rviz/issues/1107
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_imu;
    arrIMU.header.frame_id = "global";
    for(size_t i=0; i<poses_imu.size(); i+=std::floor(poses_imu.size()/16384.0)+1) {
        arrIMU.poses.push_back(poses_imu.at(i));
    }
    pub_pathimu.publish(arrIMU);

    // Move them forward in time
    poses_seq_imu++;

    // Publish our transform on TF
    // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
    // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "imu";
    tf::Quaternion quat(state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3));
    trans.setRotation(quat);
    tf::Vector3 orig(state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2));
    trans.setOrigin(orig);
    if(publish_global2imu_tf) {
        mTfBr->sendTransform(trans);
    }

    // Loop through each camera calibration and publish it
    for(const auto &calib : state->_calib_IMUtoCAM) {
        // need to flip the transform to the IMU frame
        Eigen::Vector4d q_ItoC = calib.second->quat();
        Eigen::Vector3d p_CinI = -calib.second->Rot().transpose()*calib.second->pos();
        // publish our transform on TF
        // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
        // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
        tf::StampedTransform trans;
        trans.stamp_ = ros::Time::now();
        trans.frame_id_ = "imu";
        trans.child_frame_id_ = "cam"+std::to_string(calib.first);
        tf::Quaternion quat(q_ItoC(0),q_ItoC(1),q_ItoC(2),q_ItoC(3));
        trans.setRotation(quat);
        tf::Vector3 orig(p_CinI(0),p_CinI(1),p_CinI(2));
        trans.setOrigin(orig);
        if(publish_calibration_tf) {
            mTfBr->sendTransform(trans);
        }
    }

}



void RosVisualizer::publish_images() {

    // Check if we have subscribers
    if(pub_tracks.getNumSubscribers()==0)
        return;

    // Get our image of history tracks
    cv::Mat img_history = _app->get_historical_viz_image();

    // Create our message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr exl_msg = cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();

    // Publish
    pub_tracks.publish(exl_msg);

}




void RosVisualizer::publish_features() {

    // Check if we have subscribers
    if(pub_points_msckf.getNumSubscribers()==0 && pub_points_slam.getNumSubscribers()==0 &&
       pub_points_aruco.getNumSubscribers()==0 && pub_points_sim.getNumSubscribers()==0)
        return;

    // Get our good features
    std::vector<Eigen::Vector3d> feats_msckf = _app->get_good_features_MSCKF();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "global";
    cloud.header.stamp = ros::Time::now();
    cloud.width  = 3*feats_msckf.size();
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(3*feats_msckf.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    // Fill our iterators
    for(const auto &pt : feats_msckf) {
        *out_x = pt(0); ++out_x;
        *out_y = pt(1); ++out_y;
        *out_z = pt(2); ++out_z;
    }

    // Publish
    pub_points_msckf.publish(cloud);

    //====================================================================
    //====================================================================

    // Get our good features
    std::vector<Eigen::Vector3d> feats_slam = _app->get_features_SLAM();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SLAM;
    cloud_SLAM.header.frame_id = "global";
    cloud_SLAM.header.stamp = ros::Time::now();
    cloud_SLAM.width  = 3*feats_slam.size();
    cloud_SLAM.height = 1;
    cloud_SLAM.is_bigendian = false;
    cloud_SLAM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SLAM(cloud_SLAM);
    modifier_SLAM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SLAM.resize(3*feats_slam.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SLAM(cloud_SLAM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SLAM(cloud_SLAM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SLAM(cloud_SLAM, "z");

    // Fill our iterators
    for(const auto &pt : feats_slam) {
        *out_x_SLAM = pt(0); ++out_x_SLAM;
        *out_y_SLAM = pt(1); ++out_y_SLAM;
        *out_z_SLAM = pt(2); ++out_z_SLAM;
    }

    // Publish
    pub_points_slam.publish(cloud_SLAM);

    //====================================================================
    //====================================================================

    // Get our good features
    std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_ARUCO;
    cloud_ARUCO.header.frame_id = "global";
    cloud_ARUCO.header.stamp = ros::Time::now();
    cloud_ARUCO.width  = 3*feats_aruco.size();
    cloud_ARUCO.height = 1;
    cloud_ARUCO.is_bigendian = false;
    cloud_ARUCO.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_ARUCO(cloud_ARUCO);
    modifier_ARUCO.setPointCloud2FieldsByString(1,"xyz");
    modifier_ARUCO.resize(3*feats_aruco.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_ARUCO(cloud_ARUCO, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_ARUCO(cloud_ARUCO, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_ARUCO(cloud_ARUCO, "z");

    // Fill our iterators
    for(const auto &pt : feats_aruco) {
        *out_x_ARUCO = pt(0); ++out_x_ARUCO;
        *out_y_ARUCO = pt(1); ++out_y_ARUCO;
        *out_z_ARUCO = pt(2); ++out_z_ARUCO;
    }

    // Publish
    pub_points_aruco.publish(cloud_ARUCO);


    //====================================================================
    //====================================================================

    // Skip the rest of we are not doing simulation
    if(_sim == nullptr)
        return;

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



void RosVisualizer::publish_groundtruth() {

    // Our groundtruth state
    Eigen::Matrix<double,17,1> state_gt;

    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = _app->get_state()->_timestamp + t_ItoC;

    // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
    if(_sim == nullptr && (gt_states.empty() || !DatasetReader::get_gt_state(timestamp_inI, state_gt, gt_states))) {
        return;
    }

    // Get the simulated groundtruth
    // NOTE: we get the true time in the IMU clock frame
    if(_sim != nullptr) {
        timestamp_inI = _app->get_state()->_timestamp + _sim->get_true_paramters().calib_camimu_dt;
        if(!_sim->get_state(timestamp_inI,state_gt))
            return;
    }

    // Get the GT and system state state
    Eigen::Matrix<double,16,1> state_ekf = _app->get_state()->_imu->value();

    // Create pose of IMU
    geometry_msgs::PoseStamped poseIinM;
    poseIinM.header.stamp = ros::Time(timestamp_inI);
    poseIinM.header.seq = poses_seq_gt;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.orientation.x = state_gt(1,0);
    poseIinM.pose.orientation.y = state_gt(2,0);
    poseIinM.pose.orientation.z = state_gt(3,0);
    poseIinM.pose.orientation.w = state_gt(4,0);
    poseIinM.pose.position.x = state_gt(5,0);
    poseIinM.pose.position.y = state_gt(6,0);
    poseIinM.pose.position.z = state_gt(7,0);
    pub_posegt.publish(poseIinM);

    // Append to our pose vector
    poses_gt.push_back(poseIinM);

    // Create our path (imu)
    // NOTE: We downsample the number of poses as needed to prevent rviz crashes
    // NOTE: https://github.com/ros-visualization/rviz/issues/1107
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_gt;
    arrIMU.header.frame_id = "global";
    for(size_t i=0; i<poses_gt.size(); i+=std::floor(poses_gt.size()/16384.0)+1) {
        arrIMU.poses.push_back(poses_gt.at(i));
    }
    pub_pathgt.publish(arrIMU);

    // Move them forward in time
    poses_seq_gt++;

    // Publish our transform on TF
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "truth";
    tf::Quaternion quat(state_gt(1,0),state_gt(2,0),state_gt(3,0),state_gt(4,0));
    trans.setRotation(quat);
    tf::Vector3 orig(state_gt(5,0),state_gt(6,0),state_gt(7,0));
    trans.setOrigin(orig);
    if(publish_global2imu_tf) {
        mTfBr->sendTransform(trans);
    }

    //==========================================================================
    //==========================================================================

    // Difference between positions
    double dx = state_ekf(4,0)-state_gt(5,0);
    double dy = state_ekf(5,0)-state_gt(6,0);
    double dz = state_ekf(6,0)-state_gt(7,0);
    double rmse_pos = std::sqrt(dx*dx+dy*dy+dz*dz);

    // Quaternion error
    Eigen::Matrix<double,4,1> quat_gt, quat_st, quat_diff;
    quat_gt << state_gt(1,0),state_gt(2,0),state_gt(3,0),state_gt(4,0);
    quat_st << state_ekf(0,0),state_ekf(1,0),state_ekf(2,0),state_ekf(3,0);
    quat_diff = quat_multiply(quat_st,Inv(quat_gt));
    double rmse_ori = (180/M_PI)*2*quat_diff.block(0,0,3,1).norm();


    //==========================================================================
    //==========================================================================

    // Get covariance of pose
    std::vector<std::shared_ptr<Type>> statevars;
    statevars.push_back(_app->get_state()->_imu->q());
    statevars.push_back(_app->get_state()->_imu->p());
    Eigen::Matrix<double,6,6> covariance = StateHelper::get_marginal_covariance(_app->get_state(),statevars);

    // Calculate NEES values
    double ori_nees = 2*quat_diff.block(0,0,3,1).dot(covariance.block(0,0,3,3).inverse()*2*quat_diff.block(0,0,3,1));
    Eigen::Vector3d errpos = state_ekf.block(4,0,3,1)-state_gt.block(5,0,3,1);
    double pos_nees = errpos.transpose()*covariance.block(3,3,3,3).inverse()*errpos;

    //==========================================================================
    //==========================================================================

    // Update our average variables
    summed_rmse_ori += rmse_ori;
    summed_rmse_pos += rmse_pos;
    summed_nees_ori += ori_nees;
    summed_nees_pos += pos_nees;
    summed_number++;

    // Nice display for the user
    printf(REDPURPLE "error to gt => %.3f, %.3f (deg,m) | average error => %.3f, %.3f (deg,m) | called %d times\n" RESET,rmse_ori,rmse_pos,summed_rmse_ori/summed_number,summed_rmse_pos/summed_number, (int)summed_number);
    printf(REDPURPLE "nees => %.1f, %.1f (ori,pos) | average nees = %.1f, %.1f (ori,pos)\n" RESET,ori_nees,pos_nees,summed_nees_ori/summed_number,summed_nees_pos/summed_number);


    //==========================================================================
    //==========================================================================



}



void RosVisualizer::publish_loopclosure_information() {

    // Get the current tracks in this frame
    double active_tracks_time1 = -1;
    double active_tracks_time2 = -1;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
    cv::Mat active_cam0_image;
    _app->get_active_tracks(active_tracks_time1, active_tracks_posinG, active_tracks_uvd);
    _app->get_active_image(active_tracks_time2, active_cam0_image);
    if(active_tracks_time1 == -1) return;
    if(_app->get_state()->_clones_IMU.find(active_tracks_time1) == _app->get_state()->_clones_IMU.end()) return;
    if(active_tracks_time1 != active_tracks_time2) return;

    // Default header
    std_msgs::Header header;
    header.stamp = ros::Time(active_tracks_time1);

    //======================================================
    // Check if we have subscribers for the pose odometry, camera intrinsics, or extrinsics
    if(pub_loop_pose.getNumSubscribers() != 0 ||
        pub_loop_extrinsic.getNumSubscribers() != 0 ||
        pub_loop_intrinsics.getNumSubscribers() != 0) {

        // PUBLISH HISTORICAL POSE ESTIMATE
        nav_msgs::Odometry odometry_pose;
        odometry_pose.header = header;
        odometry_pose.header.frame_id = "global";
        odometry_pose.pose.pose.position.x = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(0);
        odometry_pose.pose.pose.position.y = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(1);
        odometry_pose.pose.pose.position.z = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(2);
        odometry_pose.pose.pose.orientation.x = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(0);
        odometry_pose.pose.pose.orientation.y = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(1);
        odometry_pose.pose.pose.orientation.z = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(2);
        odometry_pose.pose.pose.orientation.w = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(3);
        pub_loop_pose.publish(odometry_pose);

        // PUBLISH IMU TO CAMERA0 EXTRINSIC
        // need to flip the transform to the IMU frame
        Eigen::Vector4d q_ItoC = _app->get_state()->_calib_IMUtoCAM.at(0)->quat();
        Eigen::Vector3d p_CinI = -_app->get_state()->_calib_IMUtoCAM.at(0)->Rot().transpose()*_app->get_state()->_calib_IMUtoCAM.at(0)->pos();
        nav_msgs::Odometry odometry_calib;
        odometry_calib.header = header;
        odometry_calib.header.frame_id = "imu";
        odometry_calib.pose.pose.position.x = p_CinI(0);
        odometry_calib.pose.pose.position.y = p_CinI(1);
        odometry_calib.pose.pose.position.z = p_CinI(2);
        odometry_calib.pose.pose.orientation.x = q_ItoC(0);
        odometry_calib.pose.pose.orientation.y = q_ItoC(1);
        odometry_calib.pose.pose.orientation.z = q_ItoC(2);
        odometry_calib.pose.pose.orientation.w = q_ItoC(3);
        pub_loop_extrinsic.publish(odometry_calib);

        // PUBLISH CAMERA0 INTRINSICS
        sensor_msgs::CameraInfo cameraparams;
        cameraparams.header = header;
        cameraparams.header.frame_id = "imu";
        cameraparams.distortion_model = (_app->get_state()->_cam_intrinsics_model.at(0))? "equidistant" : "plumb_bob";
        Eigen::VectorXd cparams = _app->get_state()->_cam_intrinsics.at(0)->value();
        cameraparams.D = {cparams(4), cparams(5), cparams(6), cparams(7)};
        cameraparams.K = {cparams(0), 0, cparams(2), 0, cparams(1), cparams(3), 0, 0, 1};
        pub_loop_intrinsics.publish(cameraparams);

    }

    //======================================================
    // PUBLISH FEATURE TRACKS IN THE GLOBAL FRAME OF REFERENCE
    if(pub_loop_point.getNumSubscribers() != 0) {

        // Construct the message
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = header;
        point_cloud.header.frame_id = "global";
        for(const auto &feattimes : active_tracks_posinG) {

            // Get this feature information
            size_t featid = feattimes.first;
            Eigen::Vector3d uvd = Eigen::Vector3d::Zero();
            if(active_tracks_uvd.find(featid)!=active_tracks_uvd.end()) {
                uvd = active_tracks_uvd.at(featid);
            }
            Eigen::Vector3d pFinG = active_tracks_posinG.at(featid);

            // Push back 3d point
            geometry_msgs::Point32 p;
            p.x = pFinG(0);
            p.y = pFinG(1);
            p.z = pFinG(2);
            point_cloud.points.push_back(p);

            // Push back the uv_norm, uv_raw, and feature id
            // NOTE: we don't use the normalized coordinates to save time here
            // NOTE: they will have to be re-normalized in the loop closure code
            sensor_msgs::ChannelFloat32 p_2d;
            p_2d.values.push_back(0);
            p_2d.values.push_back(0);
            p_2d.values.push_back(uvd(0));
            p_2d.values.push_back(uvd(1));
            p_2d.values.push_back(featid);
            point_cloud.channels.push_back(p_2d);

        }
        pub_loop_point.publish(point_cloud);

    }

    //======================================================
    // Depth images of sparse points and its colorized version
    if(pub_loop_img_depth.getNumSubscribers() != 0 || pub_loop_img_depth_color.getNumSubscribers() != 0) {

        // Create the images we will populate with the depths
        std::pair<int,int> wh_pair = {active_cam0_image.cols, active_cam0_image.rows};
        cv::Mat depthmap_viz;
        cv::cvtColor(active_cam0_image, depthmap_viz, cv::COLOR_GRAY2RGB);
        cv::Mat depthmap = cv::Mat::zeros(wh_pair.second, wh_pair.first, CV_16UC1);

        // Loop through all points and append
        for(const auto &feattimes : active_tracks_uvd) {

            // Get this feature information
            size_t featid = feattimes.first;
            Eigen::Vector3d uvd = active_tracks_uvd.at(featid);

            // Skip invalid points
            double dw = 3;
            if(uvd(0) < dw || uvd(0) > wh_pair.first-dw || uvd(1) < dw || uvd(1) > wh_pair.second-dw) {
                continue;
            }

            // Append the depth
            // NOTE: scaled by 1000 to fit the 16U
            // NOTE: access order is y,x (stupid opencv convention stuff)
            depthmap.at<uint16_t>((int)uvd(1),(int)uvd(0)) = (uint16_t)(1000*uvd(2));

            // Taken from LSD-SLAM codebase segment into 0-4 meter segments:
            // https://github.com/tum-vision/lsd_slam/blob/d1e6f0e1a027889985d2e6b4c0fe7a90b0c75067/lsd_slam_core/src/util/globalFuncs.cpp#L87-L96
            float id = 1.0f/(float)uvd(2);
            float r = (0.0f - id) * 255 / 1.0f;
            if (r < 0) r = -r;
            float g = (1.0f - id) * 255 / 1.0f;
            if (g < 0) g = -g;
            float b = (2.0f - id) * 255 / 1.0f;
            if (b < 0) b = -b;
            uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
            uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
            uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);
            cv::Scalar color(255-rc,255-gc,255-bc);

            // Small square around the point (note the above bound check needs to take into account this width)
            cv::Point p0(uvd(0)-dw, uvd(1)-dw);
            cv::Point p1(uvd(0)+dw, uvd(1)+dw);
            cv::rectangle(depthmap_viz, p0, p1, color, -1);

        }

        // Create our messages
        sensor_msgs::ImagePtr exl_msg1 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depthmap).toImageMsg();
        pub_loop_img_depth.publish(exl_msg1);
        sensor_msgs::ImagePtr exl_msg2 = cv_bridge::CvImage(header, "bgr8", depthmap_viz).toImageMsg();
        pub_loop_img_depth_color.publish(exl_msg2);

    }

}


void RosVisualizer::sim_save_total_state_to_file() {

    // Get current state
    std::shared_ptr<State> state = _app->get_state();

    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;

    // If we have our simulator, then save it to our groundtruth file
    if(_sim != nullptr) {

        // Note that we get the true time in the IMU clock frame
        // NOTE: we record both the estimate and groundtruth with the same "true" timestamp if we are doing simulation
        Eigen::Matrix<double,17,1> state_gt;
        timestamp_inI = state->_timestamp + _sim->get_true_paramters().calib_camimu_dt;
        if(_sim->get_state(timestamp_inI,state_gt)) {
            // STATE: write current true state
            of_state_gt.precision(5);
            of_state_gt.setf(std::ios::fixed, std::ios::floatfield);
            of_state_gt << state_gt(0) << " ";
            of_state_gt.precision(6);
            of_state_gt << state_gt(1) << " " << state_gt(2) << " " << state_gt(3) << " " << state_gt(4) << " ";
            of_state_gt << state_gt(5) << " " << state_gt(6) << " " << state_gt(7) << " ";
            of_state_gt << state_gt(8) << " " << state_gt(9) << " " << state_gt(10) << " ";
            of_state_gt << state_gt(11) << " " << state_gt(12) << " " << state_gt(13) << " ";
            of_state_gt << state_gt(14) << " " << state_gt(15) << " " << state_gt(16) << " ";

            // TIMEOFF: Get the current true time offset
            of_state_gt.precision(7);
            of_state_gt << _sim->get_true_paramters().calib_camimu_dt << " ";
            of_state_gt.precision(0);
            of_state_gt << state->_options.num_cameras << " ";
            of_state_gt.precision(6);

            // CALIBRATION: Write the camera values to file
            assert(state->_options.num_cameras==_sim->get_true_paramters().state_options.num_cameras);
            for(int i=0; i<state->_options.num_cameras; i++) {
                // Intrinsics values
                of_state_gt << _sim->get_true_paramters().camera_intrinsics.at(i)(0) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(1) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(2) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(3) << " ";
                of_state_gt << _sim->get_true_paramters().camera_intrinsics.at(i)(4) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(5) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(6) << " " << _sim->get_true_paramters().camera_intrinsics.at(i)(7) << " ";
                // Rotation and position
                of_state_gt << _sim->get_true_paramters().camera_extrinsics.at(i)(0) << " " << _sim->get_true_paramters().camera_extrinsics.at(i)(1) << " " << _sim->get_true_paramters().camera_extrinsics.at(i)(2) << " " << _sim->get_true_paramters().camera_extrinsics.at(i)(3) << " ";
                of_state_gt << _sim->get_true_paramters().camera_extrinsics.at(i)(4) << " " << _sim->get_true_paramters().camera_extrinsics.at(i)(5) << " " << _sim->get_true_paramters().camera_extrinsics.at(i)(6) << " ";
            }

            // New line
            of_state_gt << endl;
        }

    }

    //==========================================================================
    //==========================================================================
    //==========================================================================

    // Get the covariance of the whole system
    Eigen::MatrixXd cov = StateHelper::get_full_covariance(state);

    // STATE: Write the current state to file
    of_state_est.precision(5);
    of_state_est.setf(std::ios::fixed, std::ios::floatfield);
    of_state_est << timestamp_inI << " ";
    of_state_est.precision(6);
    of_state_est << state->_imu->quat()(0) << " " << state->_imu->quat()(1) << " " << state->_imu->quat()(2) << " " << state->_imu->quat()(3) << " ";
    of_state_est << state->_imu->pos()(0) << " " << state->_imu->pos()(1) << " " << state->_imu->pos()(2) << " ";
    of_state_est << state->_imu->vel()(0) << " " << state->_imu->vel()(1) << " " << state->_imu->vel()(2) << " ";
    of_state_est << state->_imu->bias_g()(0) << " " << state->_imu->bias_g()(1) << " " << state->_imu->bias_g()(2) << " ";
    of_state_est << state->_imu->bias_a()(0) << " " << state->_imu->bias_a()(1) << " " << state->_imu->bias_a()(2) << " ";

    // STATE: Write current uncertainty to file
    of_state_std.precision(5);
    of_state_std.setf(std::ios::fixed, std::ios::floatfield);
    of_state_std << timestamp_inI << " ";
    of_state_std.precision(6);
    int id = state->_imu->q()->id();
    of_state_std << std::sqrt(cov(id+0, id+0)) << " " << std::sqrt(cov(id+1, id+1)) << " " << std::sqrt(cov(id+2, id+2)) << " ";
    id = state->_imu->p()->id();
    of_state_std << std::sqrt(cov(id+0, id+0)) << " " << std::sqrt(cov(id+1, id+1)) << " " << std::sqrt(cov(id+2, id+2)) << " ";
    id = state->_imu->v()->id();
    of_state_std << std::sqrt(cov(id+0, id+0)) << " " << std::sqrt(cov(id+1, id+1)) << " " << std::sqrt(cov(id+2, id+2)) << " ";
    id = state->_imu->bg()->id();
    of_state_std << std::sqrt(cov(id+0, id+0)) << " " << std::sqrt(cov(id+1, id+1)) << " " << std::sqrt(cov(id+2, id+2)) << " ";
    id = state->_imu->ba()->id();
    of_state_std << std::sqrt(cov(id+0, id+0)) << " " << std::sqrt(cov(id+1, id+1)) << " " << std::sqrt(cov(id+2, id+2)) << " ";

    // TIMEOFF: Get the current estimate time offset
    of_state_est.precision(7);
    of_state_est << state->_calib_dt_CAMtoIMU->value()(0) << " ";
    of_state_est.precision(0);
    of_state_est << state->_options.num_cameras << " ";
    of_state_est.precision(6);

    // TIMEOFF: Get the current std values
    if(state->_options.do_calib_camera_timeoffset) {
        of_state_std << std::sqrt(cov(state->_calib_dt_CAMtoIMU->id(), state->_calib_dt_CAMtoIMU->id())) << " ";
    } else {
        of_state_std << 0.0 << " ";
    }
    of_state_std.precision(0);
    of_state_std << state->_options.num_cameras << " ";
    of_state_std.precision(6);

    // CALIBRATION: Write the camera values to file
    for(int i=0; i<state->_options.num_cameras; i++) {
        // Intrinsics values
        of_state_est << state->_cam_intrinsics.at(i)->value()(0) << " " << state->_cam_intrinsics.at(i)->value()(1) << " " << state->_cam_intrinsics.at(i)->value()(2) << " " << state->_cam_intrinsics.at(i)->value()(3) << " ";
        of_state_est << state->_cam_intrinsics.at(i)->value()(4) << " " << state->_cam_intrinsics.at(i)->value()(5) << " " << state->_cam_intrinsics.at(i)->value()(6) << " " << state->_cam_intrinsics.at(i)->value()(7) << " ";
        // Rotation and position
        of_state_est << state->_calib_IMUtoCAM.at(i)->value()(0) << " " << state->_calib_IMUtoCAM.at(i)->value()(1) << " " << state->_calib_IMUtoCAM.at(i)->value()(2) << " " << state->_calib_IMUtoCAM.at(i)->value()(3) << " ";
        of_state_est << state->_calib_IMUtoCAM.at(i)->value()(4) << " " << state->_calib_IMUtoCAM.at(i)->value()(5) << " " << state->_calib_IMUtoCAM.at(i)->value()(6) << " ";
        // Covariance
        if(state->_options.do_calib_camera_intrinsics) {
            int index_in = state->_cam_intrinsics.at(i)->id();
            of_state_std << std::sqrt(cov(index_in + 0, index_in + 0)) << " " << std::sqrt(cov(index_in + 1, index_in + 1)) << " " << std::sqrt(cov(index_in + 2, index_in + 2)) << " " << std::sqrt(cov(index_in + 3, index_in + 3)) << " ";
            of_state_std << std::sqrt(cov(index_in + 4, index_in + 4)) << " " << std::sqrt(cov(index_in + 5, index_in + 5)) << " " << std::sqrt(cov(index_in + 6, index_in + 6)) << " " << std::sqrt(cov(index_in + 7, index_in + 7)) << " ";
        } else {
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        }
        if(state->_options.do_calib_camera_pose) {
            int index_ex = state->_calib_IMUtoCAM.at(i)->id();
            of_state_std << std::sqrt(cov(index_ex + 0, index_ex + 0)) << " " << std::sqrt(cov(index_ex + 1, index_ex + 1)) << " " << std::sqrt(cov(index_ex + 2, index_ex + 2)) << " ";
            of_state_std << std::sqrt(cov(index_ex + 3, index_ex + 3)) << " " << std::sqrt(cov(index_ex + 4, index_ex + 4)) << " " << std::sqrt(cov(index_ex + 5, index_ex + 5)) << " ";
        } else {
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        }
    }

    // Done with the estimates!
    of_state_est << endl;
    of_state_std << endl;



}






