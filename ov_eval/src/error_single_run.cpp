

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


#include "alignment/Trajectory.h"



int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "error_single_run");

    // Ensure we have a path
    if(argc < 4) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval error_single_run <align_mode> <file_gt.txt> <file_est.txt>");
        std::exit(EXIT_FAILURE);
    }


    // Create our trajectory object
    ov_eval::Trajectory traj(argv[3], argv[2], argv[1]);

    //===========================================================
    // Absolute trajectory error
    //===========================================================

    // Calculate
    ov_eval::Statistics error_ori, error_pos;
    traj.calculate_ate(error_ori, error_pos);

    // Print it
    ROS_INFO("============================");
    ROS_INFO("Absolute Trajectory Error");
    ROS_INFO("============================");
    ROS_INFO("rmse_ori = %.3f | rmse_pos = %.3f",error_ori.rmse,error_pos.rmse);
    ROS_INFO("mean_ori = %.3f | mean_pos = %.3f",error_ori.mean,error_pos.mean);
    ROS_INFO("min_ori  = %.3f | min_pos  = %.3f",error_ori.min,error_pos.min);
    ROS_INFO("max_ori  = %.3f | max_pos  = %.3f",error_ori.max,error_pos.max);
    ROS_INFO("std_ori  = %.3f | std_pos  = %.3f",error_ori.std,error_pos.std);

    //===========================================================
    // Relative pose error
    //===========================================================

    // Calculate
    std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
    std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> error_rpe;
    traj.calculate_rpe(segments, error_rpe);

    // Print it
    ROS_INFO("============================");
    ROS_INFO("Relative Pose Error");
    ROS_INFO("============================");
    for(const auto &seg : error_rpe) {
        ROS_INFO("seg %d - median_ori = %.3f | median_pos = %.3f (%d samples)",(int)seg.first,seg.second.first.median,seg.second.second.median,(int)seg.second.second.values.size());
        //ROS_INFO("seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
    }


    // Done!
    return EXIT_SUCCESS;

}


