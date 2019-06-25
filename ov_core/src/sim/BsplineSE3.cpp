#include "BsplineSE3.h"


using namespace ov_core;





void BsplineSE3::feed_trajectory(std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_points) {


    // Find the average frequency
    double sumdt = 0;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        sumdt = traj_points.at(i+1)(0)-traj_points.at(i)(0);
    }
    dt = sumdt/(traj_points.size()-1);

    // convert all our trajectory points into SE(3) matrices
    // we are given [timestamp, p_IinG, q_GtoI]
    std::map<double,Eigen::Matrix4d> trajectory_points;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();
        T_IinG.block(0,0,3,3) = quat_2_Rot(traj_points.at(i).block(4,0,4,1)).transpose();
        T_IinG.block(0,3,3,1) = traj_points.at(i).block(1,0,3,1);
        trajectory_points.insert({traj_points.at(i)(0),T_IinG});
    }


    // then create spline control points






}





