

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "calc/ResultSimulation.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "error_simulation");

    // Ensure we have a path
    if(argc < 4) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval error_simulation <file_est.txt> <file_std.txt> <file_gt.txt>");
        std::exit(EXIT_FAILURE);
    }


    // Create our trajectory object
    ov_eval::ResultSimulation traj(argv[1], argv[2], argv[3]);

    // Plot the state errors
    ROS_INFO("Plotting state variable errors...");
    traj.plot_state(true);

    // Plot time offset
    ROS_INFO("Plotting time offset error...");
    traj.plot_timeoff(true, 10);

    // Plot camera intrinsics
    ROS_INFO("Plotting camera intrinsics...");
    traj.plot_cam_instrinsics(true, 30);

    // Plot camera extrinsics
    ROS_INFO("Plotting camera extrinsics...");
    traj.plot_cam_extrinsics(true);


#ifdef HAVE_PYTHONLIBS
    matplotlibcpp::show(true);
#endif


    // Done!
    return EXIT_SUCCESS;

}


