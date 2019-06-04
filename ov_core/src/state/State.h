//
// Created by keck on 6/3/19.
//

#ifndef PROJECT_STATE_H
#define PROJECT_STATE_H

#include "types/IMU.h"
#include "types/Vec.h"
#include "types/PoseJPL.h"
#include <map>

class Landmark;


class State{


    //Return pointer to IMU
    IMU *imu(){
        return _imu;
    }


    //Return reference to covariance
    Eigen::MatrixXd& Cov(){
        return _Cov;
    }

protected:

    //Covariance of the state
    Eigen::MatrixXd _Cov;

    //Pointer to the "active" IMU state

    IMU *_imu;


    // Calibration poses for each camera (R_ItoC, p_IinC)
    std::map<size_t,PoseJPL*> calib_IMUtoCAM;


    // Our current set of SLAM features (3d positions)
    std::map<size_t,Landmark*> features_SLAM;

    //Time offset base IMU to camera
    Vec* calib_dt_CAMtoIMU;

    //Camera intrinsics
    std::map<size_t,Vec*> cam_intrinsics;


};



#endif //PROJECT_STATE_H
