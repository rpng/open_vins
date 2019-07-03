

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/filesystem.hpp>


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "format_converter");

    // Ensure we have a path
    if(argc < 2) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval format_convert <path.csv>");
        std::exit(EXIT_FAILURE);
    }

    // Check if file paths are good
    std::string infile = argv[1];
    std::ifstream file1;
    std::string line;
    file1.open(infile);

    // Check that it was successful
    if (!file1) {
        ROS_ERROR("ERROR: Unable to open input file...");
        ROS_ERROR("ERROR: %s", infile.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::vector<Eigen::VectorXd> traj_data;
    std::string current_line;
    while(std::getline(file1, current_line) && ros::ok()) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        int i = 0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<double,8,1> data;

        // Loop through this line (timestamp(ns) tx ty tz qw qx qy qz)
        while(std::getline(s,field,',') && ros::ok()) {
            // Skip if empty
            if(field.empty() || i >= data.rows())
                continue;
            // save the data to our vector
            data(i) = std::atof(field.c_str());
            i++;
        }

        // Only a valid line if we have all the parameters
        if(i > 7) {
            traj_data.push_back(data);
            //std::cout << std::setprecision(5) << data.transpose() << std::endl;
        }

    }

    // Finally close the file
    file1.close();

    // Error if we don't have any data
    if (traj_data.empty()) {
        ROS_ERROR("ERROR: Could not parse any data from the file!!");
        ROS_ERROR("ERROR: %s",infile.c_str());
        std::exit(EXIT_FAILURE);
    }
    ROS_INFO("Loaded %d poses from file",(int)traj_data.size());

    // If file exists already then crash
    std::string outfile = infile.substr(0,infile.find_last_of('.'))+".txt";
    if(boost::filesystem::exists(outfile)) {
        ROS_ERROR("ERROR: Output file already exists!!");
        ROS_ERROR("ERROR: Please delete and re-run this script!");
        ROS_ERROR("ERROR: %s",outfile.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Open this file we want to write to
    std::ofstream file2;
    file2.open(outfile.c_str());
    if(file2.fail()) {
        ROS_ERROR("ERROR: Unable to open output file!!");
        ROS_ERROR("ERROR: %s",outfile.c_str());
        std::exit(EXIT_FAILURE);
    }
    file2 << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;

    // Write to disk in the correct order!
    for(size_t i=0; i<traj_data.size(); i++) {
        file2.precision(5);
        file2.setf(std::ios::fixed, std::ios::floatfield);
        file2 << 1e-9*traj_data.at(i)(0) << " ";
        file2.precision(6);
        file2 << traj_data.at(i)(1) << " " << traj_data.at(i)(2) << " " << traj_data.at(i)(3) << " "
              << traj_data.at(i)(5) << " " << traj_data.at(i)(6) << " " << traj_data.at(i)(7) << " " << traj_data.at(i)(4) << std::endl;
    }
    ROS_INFO("Saved to file %s",outfile.c_str());

    // Finally close the file
    file2.close();

    // Done!
    return EXIT_SUCCESS;

}



