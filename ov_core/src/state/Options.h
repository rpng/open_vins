//
// Created by keck on 6/4/19.
//

#ifndef OPEN_VINS_OPTIONS_H
#define OPEN_VINS_OPTIONS_H

/**@brief Struct which stores all our filter options
 */

struct Options{

    /// Bool to determine whether or not to calibrate imu-to-camera pose
    bool do_calib_camera_pose;

    /// Bool to determine whether or not to calibrate camera intrinsics
    bool do_calib_camera_intrinsics;

    /// Bool to determine whether or not to calibrate camera to IMU time offset
    bool do_calib_camera_timeoffset;

    /// Max clone size of sliding window
    size_t max_clone_size;

    /// Max number of estimated SLAM features
    size_t max_slam_features;

    /// Number of cameras
    size_t num_cameras;

};



#endif //OPEN_VINS_OPTIONS_H
