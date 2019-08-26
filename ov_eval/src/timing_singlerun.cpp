/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utils/Statistics.h"
#include "utils/Loader.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "timing_singlerun");

    // Ensure we have a path
    if(argc < 2) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval timing_singlerun <file_times.txt>");
        std::exit(EXIT_FAILURE);
    }

    // Load it!!
    std::vector<double> times;
    std::vector<Eigen::Vector3d> summed_values;
    std::vector<Eigen::VectorXd> node_values;
    ov_eval::Loader::load_timing(argv[1], times, summed_values, node_values);
    ROS_INFO("[TIME]: loaded %d timestamps from file!!",(int)times.size());



#ifdef HAVE_PYTHONLIBS

    // Zero our time arrays
    double starttime1 = (times.empty())? 0 : times.at(0);
    double endtime1 = (times.empty())? 0 : times.at(times.size()-1);
    for(size_t j=0; j<times.size(); j++) {
        times.at(j) -= starttime1;
    }

    // Create vector for each summed value
    std::vector<double> veccpu, vecmem, vecthread;
    for(size_t j=0; j<times.size(); j++) {
        veccpu.push_back(summed_values.at(j)(0));
        vecmem.push_back(summed_values.at(j)(1));
        vecthread.push_back(summed_values.at(j)(2));
    }

    // Plot this figure
    matplotlibcpp::figure_size(800, 600);

    // percent cpu
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::plot(times, veccpu);
    matplotlibcpp::ylabel("cpu (%)");
    matplotlibcpp::xlim(0.0,endtime1-starttime1);

    // percent mem
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::plot(times, vecmem);
    matplotlibcpp::ylabel("memory (%)");
    matplotlibcpp::xlim(0.0,endtime1-starttime1);

    // number threads
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::plot(times, vecthread);
    matplotlibcpp::ylabel("total threads");
    matplotlibcpp::xlim(0.0,endtime1-starttime1);
    matplotlibcpp::xlabel("dataset time (s)");

    // Display to the user
    matplotlibcpp::show(true);

#endif



    // Done!
    return EXIT_SUCCESS;

}







