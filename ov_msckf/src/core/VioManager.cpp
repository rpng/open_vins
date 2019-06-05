#include "VioManager.h"



using namespace ov_core;
using namespace ov_msckf;





VioManager::VioManager(ros::NodeHandle &nh) {



    // Load our state options
    StateOptions state_options;

    nh.param<bool>("use_fej", state_options.do_fej, false);
    nh.param<bool>("use_imuavg", state_options.imu_avg, false);
    nh.param<bool>("calib_cam_extrinsics", state_options.do_calib_camera_pose, false);
    nh.param<bool>("calib_cam_intrinsics", state_options.do_calib_camera_intrinsics, false);
    nh.param<bool>("calib_camimu_dt", state_options.do_calib_camera_timeoffset, false);
    nh.param<int>("max_clones", state_options.max_clone_size, 10);
    nh.param<int>("max_slam", state_options.max_slam_features, 0);
    nh.param<int>("max_cameras", state_options.num_cameras, 1);





}




