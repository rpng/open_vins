#include "State.h"


using namespace ov_core;
using namespace ov_msckf;



void State::initialize_variables() {

    int current_id = 0;

    // Append the imu to the state and covariance
    _imu = new IMU();
    _imu->set_local_id(current_id);
    insert_variable(_imu);
    current_id += _imu->size();

    // Camera to IMU time offset
    _calib_dt_CAMtoIMU = new Vec(1);
    if (_options.do_calib_camera_timeoffset) {
        _calib_dt_CAMtoIMU->set_local_id(current_id);
        insert_variable(_calib_dt_CAMtoIMU);
        current_id += _calib_dt_CAMtoIMU->size();
    }

    // Loop through each camera and create extrinsic and intrinsics
    for (int i = 0; i < _options.num_cameras; i++) {

        // Allocate extrinsic transform
        PoseJPL *pose = new PoseJPL();

        // Allocate intrinsics for this camera
        Vec *intrin = new Vec(8);

        // Add these to the corresponding maps
        _calib_IMUtoCAM.insert({i, pose});
        _cam_intrinsics.insert({i, intrin});
        _cam_intrinsics_model.insert({i, false});

        // If calibrating camera-imu pose, add to variables
        if (_options.do_calib_camera_pose) {
            pose->set_local_id(current_id);
            insert_variable(pose);
            current_id += pose->size();
        }

        // If calibrating camera intrinsics, add to variables
        if (_options.do_calib_camera_intrinsics) {
            intrin->set_local_id(current_id);
            insert_variable(intrin);
            current_id += intrin->size();
        }
    }

    // Finally initialize our covariance to small value
    _Cov = 1e-4*Eigen::MatrixXd::Identity(current_id, current_id);

}

