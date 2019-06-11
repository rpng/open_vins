#include "RosVisualizer.h"


using namespace ov_msckf;



RosVisualizer::RosVisualizer(ros::NodeHandle &nh, VioManager* app) : _nh(nh), _app(app) {


    // Setup our transform broadcaster
    mTfBr = new tf::TransformBroadcaster();

    // Setup pose and path publisher
    pub_poseimu = nh.advertise<geometry_msgs::PoseStamped>("/ov_msckf/poseimu", 2);
    ROS_INFO("Publishing: %s", pub_poseimu.getTopic().c_str());
    pub_pathimu = nh.advertise<nav_msgs::Path>("/ov_msckf/pathimu", 2);
    ROS_INFO("Publishing: %s", pub_pathimu.getTopic().c_str());

    // 3D points publishing
    pub_points_msckf = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_msckf", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_slam = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_slam", 2);
    ROS_INFO("Publishing: %s", pub_points_msckf.getTopic().c_str());
    pub_points_aruco = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_aruco", 2);
    ROS_INFO("Publishing: %s", pub_points_aruco.getTopic().c_str());

    // Our tracking image
    pub_tracks = nh.advertise<sensor_msgs::Image>("/ov_msckf/trackhist", 2);
    ROS_INFO("Publishing: %s", pub_tracks.getTopic().c_str());

    // Groundtruth publishers
    pub_posegt = nh.advertise<geometry_msgs::PoseStamped>("/ov_msckf/posegt", 2);
    ROS_INFO("Publishing: %s", pub_posegt.getTopic().c_str());
    pub_pathgt = nh.advertise<nav_msgs::Path>("/ov_msckf/pathgt", 2);
    ROS_INFO("Publishing: %s", pub_pathgt.getTopic().c_str());

    // Load groundtruth if we have it
    if (nh.hasParam("path_gt")) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        load_gt_file(path_to_gt, gt_states);
    }

}



void RosVisualizer::visualize() {


    // publish current image
    publish_images();

    // Return if we have not inited
    if(!_app->intialized())
        return;

    // publish state
    publish_state();

    // publish points
    publish_features();

    // Publish gt if we have it
    publish_groundtruth();

}





void RosVisualizer::publish_state() {

    // Get the current state
    State* state = _app->get_state();

    // Create pose of IMU (note we use the bag time)
    geometry_msgs::PoseStamped poseIinM;
    poseIinM.header.stamp = ros::Time(state->timestamp());
    poseIinM.header.seq = poses_seq_imu;
    poseIinM.header.frame_id = "global";
    poseIinM.pose.orientation.x = state->imu()->quat()(0);
    poseIinM.pose.orientation.y = state->imu()->quat()(1);
    poseIinM.pose.orientation.z = state->imu()->quat()(2);
    poseIinM.pose.orientation.w = state->imu()->quat()(3);
    poseIinM.pose.position.x = state->imu()->pos()(0);
    poseIinM.pose.position.y = state->imu()->pos()(1);
    poseIinM.pose.position.z = state->imu()->pos()(2);
    pub_poseimu.publish(poseIinM);

    // Append to our pose vector
    poses_imu.push_back(poseIinM);

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
        Eigen::Vector4d q_CtoI = Inv(calib.second->quat());
        Eigen::Vector3d p_IinC = -calib.second->Rot().transpose()*calib.second->pos();
        // publish
        tf::StampedTransform trans;
        trans.stamp_ = ros::Time::now();
        trans.frame_id_ = "imu";
        trans.child_frame_id_ = "cam"+std::to_string(calib.first);
        tf::Quaternion quat(q_CtoI(0),q_CtoI(1),q_CtoI(2),q_CtoI(3));
        trans.setRotation(quat);
        tf::Vector3 orig(p_IinC(0),p_IinC(1),p_IinC(2));
        trans.setOrigin(orig);
        mTfBr->sendTransform(trans);
    }

}



void RosVisualizer::publish_images() {

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

}



void RosVisualizer::publish_groundtruth() {

    // Return if we don't have gt states
    if(gt_states.empty())
        return;

    // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
    Eigen::Matrix<double,17,1> state_gt;
    if(!get_gt_state(_app->get_state()->timestamp(), state_gt, gt_states)) {
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
    Eigen::Matrix<double,4,1> quat_gt,quat_st, quat_diff;
    quat_gt << state_gt(1,0),state_gt(2,0),state_gt(3,0),state_gt(4,0);
    quat_st << state_ekf(0,0),state_ekf(1,0),state_ekf(2,0),state_ekf(3,0);
    quat_diff = quat_multiply(quat_st,Inv(quat_gt));
    double rmse_ori = (180/M_PI)*2*quat_diff.block(0,0,3,1).norm();

    // Update our average variables
    summed_rmse_ori += rmse_ori;
    summed_rmse_pos += rmse_pos;
    summed_number++;

    // Nice display for the user
    ROS_INFO("\033[0;95merror to gt => %.3f, %.3f (deg,m) | average error => %.3f, %.3f (deg,m)\033[0m",rmse_ori,rmse_pos,summed_rmse_ori/summed_number,summed_rmse_pos/summed_number);

    //==========================================================================
    //==========================================================================



}







