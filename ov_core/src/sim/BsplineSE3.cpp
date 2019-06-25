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
    // we are given q_GtoI and p_IinG
    std::map<double,Eigen::Matrix4d> trajectory_points;
    for(size_t i=0; i<traj_points.size()-1; i++) {



    }






    // then create spline control points






}





