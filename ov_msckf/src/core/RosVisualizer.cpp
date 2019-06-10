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

}











