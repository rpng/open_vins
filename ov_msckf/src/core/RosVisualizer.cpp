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



RosVisualizer::RosVisualizer(ros::NodeHandle &nh, VioManager* app, Simulator *sim) : _nh(nh), _app(app), _sim(sim) {


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


    // publish current image
    publish_images();

    // Return if we have not inited
    if(!_app->intialized())
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

    // save total state
    if(save_total_state)
        sim_save_total_state_to_file();

}



void RosVisualizer::visualize_final() {


    // TODO: publish our calibration final results


    // Publish RMSE if we have it
    if(!gt_states.empty()) {
        ROS_INFO("\033[0;95mRMSE average: %.3f (deg) orientation\033[0m",summed_rmse_ori/summed_number);
        ROS_INFO("\033[0;95mRMSE average: %.3f (m) position\033[0m",summed_rmse_pos/summed_number);
    }

    // Publish RMSE and NEES if doing simulation
    if(_sim != nullptr) {
        ROS_INFO("\033[0;95mRMSE average: %.3f (deg) orientation\033[0m",summed_rmse_ori/summed_number);
        ROS_INFO("\033[0;95mRMSE average: %.3f (m) position\033[0m",summed_rmse_pos/summed_number);
        ROS_INFO("\033[0;95mNEES average: %.3f (deg) orientation\033[0m",summed_nees_ori/summed_number);
        ROS_INFO("\033[0;95mNEES average: %.3f (m) position\033[0m",summed_nees_pos/summed_number);
    }

    // Print the total time
    rT2 =  boost::posix_time::microsec_clock::local_time();
    ROS_INFO("\033[0;95mTIME: %.3f seconds\033[0m",(rT2-rT1).total_microseconds()*1e-6);

}



void RosVisualizer::publish_state() {

    // Get the current state
    State* state = _app->get_state();

    // Create pose of IMU (note we use the bag time)
    geometry_msgs::PoseWithCovarianceStamped poseIinM;
    poseIinM.header.stamp = ros::Time(state->timestamp());
    poseIinM.header.seq = poses_seq_imu;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.pose.orientation.x = state->imu()->quat()(0);
    poseIinM.pose.pose.orientation.y = state->imu()->quat()(1);
    poseIinM.pose.pose.orientation.z = state->imu()->quat()(2);
    poseIinM.pose.pose.orientation.w = state->imu()->quat()(3);
    poseIinM.pose.pose.position.x = state->imu()->pos()(0);
    poseIinM.pose.pose.position.y = state->imu()->pos()(1);
    poseIinM.pose.pose.position.z = state->imu()->pos()(2);

    // Finally set the covariance in the message (in the order position then orientation as per ros convention)
    std::vector<Type*> statevars;
    statevars.push_back(state->imu()->pose()->p());
    statevars.push_back(state->imu()->pose()->q());
    Eigen::Matrix<double,6,6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(),statevars);
    for(int r=0; r<6; r++) {
        for(int c=0; c<6; c++) {
            poseIinM.pose.covariance[6*r+c] = covariance_posori(r,c);
        }
    }
    pub_poseimu.publish(poseIinM);

    //=========================================================
    //=========================================================

    // Our odometry message (note we do not fill out angular velocities)
    nav_msgs::Odometry odomIinM;
    odomIinM.header = poseIinM.header;
    odomIinM.pose.pose = poseIinM.pose.pose;
    odomIinM.pose.covariance = poseIinM.pose.covariance;
    odomIinM.child_frame_id = "imu";
    odomIinM.twist.twist.angular.x = 0; // we do not estimate this...
    odomIinM.twist.twist.angular.y = 0; // we do not estimate this...
    odomIinM.twist.twist.angular.z = 0; // we do not estimate this...
    odomIinM.twist.twist.linear.x = state->imu()->vel()(0);
    odomIinM.twist.twist.linear.y = state->imu()->vel()(1);
    odomIinM.twist.twist.linear.z = state->imu()->vel()(2);

    // Velocity covariance (linear then angular)
    statevars.clear();
    statevars.push_back(state->imu()->v());
    Eigen::Matrix<double,6,6> covariance_linang = INFINITY*Eigen::Matrix<double,6,6>::Identity();
    covariance_linang.block(0,0,3,3) = StateHelper::get_marginal_covariance(_app->get_state(),statevars);
    for(int r=0; r<6; r++) {
        for(int c=0; c<6; c++) {
            odomIinM.twist.covariance[6*r+c] = (std::isnan(covariance_linang(r,c))) ? 0 : covariance_linang(r,c);
        }
    }
    pub_odomimu.publish(odomIinM);


    //=========================================================
    //=========================================================

    // Append to our pose vector
    geometry_msgs::PoseStamped posetemp;
    posetemp.header = poseIinM.header;
    posetemp.pose = poseIinM.pose.pose;
    poses_imu.push_back(posetemp);

    // Create our path (imu)
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_imu;
    arrIMU.header.frame_id = "global";
    arrIMU.poses = poses_imu;
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
    tf::Quaternion quat(state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
    trans.setRotation(quat);
    tf::Vector3 orig(state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
    trans.setOrigin(orig);
    mTfBr->sendTransform(trans);

    // Loop through each camera calibration and publish it
    for(const auto &calib : state->get_calib_IMUtoCAMs()) {
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
        mTfBr->sendTransform(trans);
    }

}



void RosVisualizer::publish_images() {

    // Check if we have subscribers
    if(pub_tracks.getNumSubscribers()==0)
        return;

    // Get our trackers
    TrackBase *trackFEATS = _app->get_track_feat();
    TrackBase *trackARUCO = _app->get_track_aruco();

    // Get our image of history tracks
    cv::Mat img_history;
    trackFEATS->display_history(img_history,255,255,0,255,255,255);
    if(trackARUCO != nullptr) {
        trackARUCO->display_history(img_history, 0, 255, 255, 255, 255, 255);
        trackARUCO->display_active(img_history, 0, 255, 255, 255, 255, 255);
    }

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

    // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
    if(_sim == nullptr && (gt_states.empty() || !DatasetReader::get_gt_state(_app->get_state()->timestamp(), state_gt, gt_states))) {
        return;
    }

    // Get the simulated groundtruth
    if(_sim != nullptr && !_sim->get_state(_app->get_state()->timestamp(),state_gt)) {
        return;
    }

    // Get the GT and system state state
    Eigen::Matrix<double,16,1> state_ekf = _app->get_state()->imu()->value();

    // Create pose of IMU
    geometry_msgs::PoseStamped poseIinM;
    poseIinM.header.stamp = ros::Time(_app->get_state()->timestamp());
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
    nav_msgs::Path arrIMU;
    arrIMU.header.stamp = ros::Time::now();
    arrIMU.header.seq = poses_seq_gt;
    arrIMU.header.frame_id = "global";
    arrIMU.poses = poses_gt;
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
    mTfBr->sendTransform(trans);

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
    Eigen::Matrix<double,6,6> covariance = _app->get_state()->Cov().block(_app->get_state()->imu()->pose()->id(),_app->get_state()->imu()->pose()->id(),6,6);

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
    ROS_INFO("\033[0;95merror to gt => %.3f, %.3f (deg,m) | average error => %.3f, %.3f (deg,m) | called %d times \033[0m",rmse_ori,rmse_pos,summed_rmse_ori/summed_number,summed_rmse_pos/summed_number, (int)summed_number);
    ROS_INFO("\033[0;95mnees => %.1f, %.1f (ori,pos) | average nees = %.1f, %.1f (ori,pos) \033[0m",ori_nees,pos_nees,summed_nees_ori/summed_number,summed_nees_pos/summed_number);


    //==========================================================================
    //==========================================================================



}





void RosVisualizer::sim_save_total_state_to_file() {

    // Get current state
    State* state = _app->get_state();

    // If we have our simulator, then save it to our groundtruth file
    Eigen::Matrix<double,17,1> state_gt;
    if(_sim != nullptr && _sim->get_state(state->timestamp(),state_gt)) {

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
        of_state_gt << _sim->get_true_imucamdt() << " ";
        of_state_gt.precision(0);
        of_state_gt << state->options().num_cameras << " ";
        of_state_gt.precision(6);

        // CALIBRATION: Write the camera values to file
        assert(state->options().num_cameras==_sim->get_num_cameras());
        for(int i=0; i<state->options().num_cameras; i++) {
            // Intrinsics values
            of_state_gt << _sim->get_true_intrinsics().at(i)(0) << " " << _sim->get_true_intrinsics().at(i)(1) << " " << _sim->get_true_intrinsics().at(i)(2) << " " << _sim->get_true_intrinsics().at(i)(3) << " ";
            of_state_gt << _sim->get_true_intrinsics().at(i)(4) << " " << _sim->get_true_intrinsics().at(i)(5) << " " << _sim->get_true_intrinsics().at(i)(6) << " " << _sim->get_true_intrinsics().at(i)(7) << " ";
            // Rotation and position
            of_state_gt << _sim->get_true_extrinsics().at(i)(0) << " " << _sim->get_true_extrinsics().at(i)(1) << " " << _sim->get_true_extrinsics().at(i)(2) << " " << _sim->get_true_extrinsics().at(i)(3) << " ";
            of_state_gt << _sim->get_true_extrinsics().at(i)(4) << " " << _sim->get_true_extrinsics().at(i)(5) << " " << _sim->get_true_extrinsics().at(i)(6) << " ";
        }

        // New line
        of_state_gt << endl;

    } else if(_sim != nullptr) {

        // Don't write anything to file if we can't get the groundtruth
        // If we don't have the simulator we will still write
        // But if we have the simulator lets only write the estimate if we have the groundtruth
        return;

    }

    //==========================================================================
    //==========================================================================
    //==========================================================================

    // STATE: Write the current state to file
    of_state_est.precision(5);
    of_state_est.setf(std::ios::fixed, std::ios::floatfield);
    of_state_est << state->timestamp() << " ";
    of_state_est.precision(6);
    of_state_est << state->imu()->quat()(0) << " " << state->imu()->quat()(1) << " " << state->imu()->quat()(2) << " " << state->imu()->quat()(3) << " ";
    of_state_est << state->imu()->pos()(0) << " " << state->imu()->pos()(1) << " " << state->imu()->pos()(2) << " ";
    of_state_est << state->imu()->vel()(0) << " " << state->imu()->vel()(1) << " " << state->imu()->vel()(2) << " ";
    of_state_est << state->imu()->bias_g()(0) << " " << state->imu()->bias_g()(1) << " " << state->imu()->bias_g()(2) << " ";
    of_state_est << state->imu()->bias_a()(0) << " " << state->imu()->bias_a()(1) << " " << state->imu()->bias_a()(2) << " ";

    // STATE: Write current uncertainty to file
    of_state_std.precision(5);
    of_state_std.setf(std::ios::fixed, std::ios::floatfield);
    of_state_std << state->timestamp() << " ";
    of_state_std.precision(6);
    int id = state->imu()->q()->id();
    of_state_std << std::sqrt(state->Cov()(id+0, id+0)) << " " << std::sqrt(state->Cov()(id+1, id+1)) << " " << std::sqrt(state->Cov()(id+2, id+2)) << " ";
    id = state->imu()->p()->id();
    of_state_std << std::sqrt(state->Cov()(id+0, id+0)) << " " << std::sqrt(state->Cov()(id+1, id+1)) << " " << std::sqrt(state->Cov()(id+2, id+2)) << " ";
    id = state->imu()->v()->id();
    of_state_std << std::sqrt(state->Cov()(id+0, id+0)) << " " << std::sqrt(state->Cov()(id+1, id+1)) << " " << std::sqrt(state->Cov()(id+2, id+2)) << " ";
    id = state->imu()->bg()->id();
    of_state_std << std::sqrt(state->Cov()(id+0, id+0)) << " " << std::sqrt(state->Cov()(id+1, id+1)) << " " << std::sqrt(state->Cov()(id+2, id+2)) << " ";
    id = state->imu()->ba()->id();
    of_state_std << std::sqrt(state->Cov()(id+0, id+0)) << " " << std::sqrt(state->Cov()(id+1, id+1)) << " " << std::sqrt(state->Cov()(id+2, id+2)) << " ";

    // TIMEOFF: Get the current estimate time offset
    of_state_est.precision(7);
    of_state_est << state->calib_dt_CAMtoIMU()->value()(0) << " ";
    of_state_est.precision(0);
    of_state_est << state->options().num_cameras << " ";
    of_state_est.precision(6);

    // TIMEOFF: Get the current std values
    if(state->options().do_calib_camera_timeoffset) {
        of_state_std << std::sqrt(state->Cov()(state->calib_dt_CAMtoIMU()->id(), state->calib_dt_CAMtoIMU()->id())) << " ";
    } else {
        of_state_std << 0.0 << " ";
    }
    of_state_std.precision(0);
    of_state_std << state->options().num_cameras << " ";
    of_state_std.precision(6);

    // CALIBRATION: Write the camera values to file
    for(int i=0; i<state->options().num_cameras; i++) {
        // Intrinsics values
        of_state_est << state->get_intrinsics_CAM(i)->value()(0) << " " << state->get_intrinsics_CAM(i)->value()(1) << " " << state->get_intrinsics_CAM(i)->value()(2) << " " << state->get_intrinsics_CAM(i)->value()(3) << " ";
        of_state_est << state->get_intrinsics_CAM(i)->value()(4) << " " << state->get_intrinsics_CAM(i)->value()(5) << " " << state->get_intrinsics_CAM(i)->value()(6) << " " << state->get_intrinsics_CAM(i)->value()(7) << " ";
        // Rotation and position
        of_state_est << state->get_calib_IMUtoCAM(i)->value()(0) << " " << state->get_calib_IMUtoCAM(i)->value()(1) << " " << state->get_calib_IMUtoCAM(i)->value()(2) << " " << state->get_calib_IMUtoCAM(i)->value()(3) << " ";
        of_state_est << state->get_calib_IMUtoCAM(i)->value()(4) << " " << state->get_calib_IMUtoCAM(i)->value()(5) << " " << state->get_calib_IMUtoCAM(i)->value()(6) << " ";
        // Covariance
        if(state->options().do_calib_camera_intrinsics) {
            int index_in = state->get_intrinsics_CAM(i)->id();
            of_state_std << std::sqrt(state->Cov()(index_in + 0, index_in + 0)) << " " << std::sqrt(state->Cov()(index_in + 1, index_in + 1)) << " " << std::sqrt(state->Cov()(index_in + 2, index_in + 2)) << " " << std::sqrt(state->Cov()(index_in + 3, index_in + 3)) << " ";
            of_state_std << std::sqrt(state->Cov()(index_in + 4, index_in + 4)) << " " << std::sqrt(state->Cov()(index_in + 5, index_in + 5)) << " " << std::sqrt(state->Cov()(index_in + 6, index_in + 6)) << " " << std::sqrt(state->Cov()(index_in + 7, index_in + 7)) << " ";
        } else {
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        }
        if(state->options().do_calib_camera_pose) {
            int index_ex = state->get_calib_IMUtoCAM(i)->id();
            of_state_std << std::sqrt(state->Cov()(index_ex + 0, index_ex + 0)) << " " << std::sqrt(state->Cov()(index_ex + 1, index_ex + 1)) << " " << std::sqrt(state->Cov()(index_ex + 2, index_ex + 2)) << " ";
            of_state_std << std::sqrt(state->Cov()(index_ex + 3, index_ex + 3)) << " " << std::sqrt(state->Cov()(index_ex + 4, index_ex + 4)) << " " << std::sqrt(state->Cov()(index_ex + 5, index_ex + 5)) << " ";
        } else {
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
            of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
        }
    }

    // Done with the estimates!
    of_state_est << endl;
    of_state_std << endl;



}






