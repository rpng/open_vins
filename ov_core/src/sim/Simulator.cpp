#include "Simulator.h"


using namespace ov_core;




void Simulator::load_data(std::string path_traj) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
        ROS_ERROR("ERROR: Unable to open simulation trajectory file...");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line) && ros::ok()) {

        // Loop variables
        int i = 0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<double,8,1> data;

        // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
        while(std::getline(s,field,' ') && ros::ok()) {
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
            //std::cout << std::setprecision(15) << data.transpose() << std::endl;
        }

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (traj_data.empty()) {
        ROS_ERROR("ERROR: Could not parse any data from the file!!");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

}







