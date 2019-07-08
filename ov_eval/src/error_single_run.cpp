

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


#include "alignment/Trajectory.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

// Will plot three error values in three sub-plots in our current figure
void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz) {

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    params_value.insert({"label","error"});
    params_value.insert({"linestyle","-"});
    params_value.insert({"color","blue"});
    params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle","--"});
    params_bound.insert({"color","red"});

    // Plot our error value
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
    matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    for(size_t i=0; i<sx.timestamps.size(); i++) {
        sx.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sx.timestamps, sx.values_bound, "r--");

    // Plot our error value
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
    matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    for(size_t i=0; i<sy.timestamps.size(); i++) {
        sy.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sy.timestamps, sy.values_bound, "r--");

    // Plot our error value
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
    matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    for(size_t i=0; i<sz.timestamps.size(); i++) {
        sz.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sz.timestamps, sz.values_bound, "r--");

}

#endif


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
    ROS_INFO("======================================");
    ROS_INFO("Absolute Trajectory Error");
    ROS_INFO("======================================");
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
    ROS_INFO("======================================");
    ROS_INFO("Relative Pose Error");
    ROS_INFO("======================================");
    for(const auto &seg : error_rpe) {
        ROS_INFO("seg %d - median_ori = %.3f | median_pos = %.3f (%d samples)",(int)seg.first,seg.second.first.median,seg.second.second.median,(int)seg.second.second.values.size());
        //ROS_INFO("seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
    }


    //===========================================================
    // Normalized Estimation Error Squared
    //===========================================================

    // Calculate
    ov_eval::Statistics nees_ori, nees_pos;
    traj.calculate_nees(nees_ori, nees_pos);

    // Print it
    ROS_INFO("======================================");
    ROS_INFO("Normalized Estimation Error Squared");
    ROS_INFO("======================================");
    ROS_INFO("mean_ori = %.3f | mean_pos = %.3f",nees_ori.mean,nees_pos.mean);
    ROS_INFO("min_ori  = %.3f | min_pos  = %.3f",nees_ori.min,nees_pos.min);
    ROS_INFO("max_ori  = %.3f | max_pos  = %.3f",nees_ori.max,nees_pos.max);
    ROS_INFO("std_ori  = %.3f | std_pos  = %.3f",nees_ori.std,nees_pos.std);


    //===========================================================
    // Plot the error if we have matplotlib to plot!
    //===========================================================

    // Calculate
    ov_eval::Statistics posx, posy, posz;
    ov_eval::Statistics orix, oriy, oriz;
    ov_eval::Statistics roll, pitch, yaw;
    traj.calculate_error(posx,posy,posz,orix,oriy,oriz,roll,pitch,yaw);


    // Zero our time arrays
    double starttime = posx.timestamps.at(0);
    for(size_t i=0; i<posx.timestamps.size(); i++) {
        posx.timestamps.at(i) -= starttime;
        posy.timestamps.at(i) -= starttime;
        posz.timestamps.at(i) -= starttime;
        orix.timestamps.at(i) -= starttime;
        oriy.timestamps.at(i) -= starttime;
        oriz.timestamps.at(i) -= starttime;
        roll.timestamps.at(i) -= starttime;
        pitch.timestamps.at(i) -= starttime;
        yaw.timestamps.at(i) -= starttime;
    }

#ifdef HAVE_PYTHONLIBS

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1200, 780);
    plot_3errors(posx,posy,posz);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("X-Axis Position Error");
    matplotlibcpp::ylabel("error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::title("Y-Axis Position Error");
    matplotlibcpp::ylabel("error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::title("Z-Axis Position Error");
    matplotlibcpp::ylabel("error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::legend();

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1200, 780);
    plot_3errors(orix,oriy,oriz);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("X-Axis Orientation Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::title("Y-Axis Orientation Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::title("Z-Axis Orientation Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::legend();

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure_size(1200, 780);
    plot_3errors(roll,pitch,yaw);

    // Update the title and axis labels
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::title("Orientation Roll Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::title("Orientation Pitch Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::title("Orientation Yaw Error");
    matplotlibcpp::ylabel("error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::legend();

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(true);


#endif


    // Done!
    return EXIT_SUCCESS;

}


