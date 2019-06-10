#ifndef OV_CORE_DATASET_READER_H
#define OV_CORE_DATASET_READER_H

/**
 * @file
 * @brief Utility functions for reading dataset formats.
 *
 * This file has some nice functions for reading dataset files.
 * One of the main datasets that we test agaist is teh EuRoC MAV dataset.
 * We have some nice utility functions here that handle loading of the groundtruth data.
 * This can be used to initalize the VINS or for plotting and calculation of RMSE values. *
 *
 * > M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari,M. Achtelik and R. Siegwart,
 * > "The EuRoC micro aerial vehicle datasets", International Journal of Robotic Research, DOI: 10.1177/0278364915620033, 2016.
 * > https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets.
 *
 * @todo We need to also extend groundtruth reading to work with the TUM visual-inertial dataset.
 * https://vision.in.tum.de/data/datasets/visual-inertial-dataset
 */


#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>


using namespace std;

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {

    /// Our groundtruth states loaded, see load_gt_file()
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

    /**
     * @brief Load a ASL format groundtruth file
     * @param path Path to the CSV file of groundtruth data
     *
     * Here we will try to load a groundtruth file that is in the ASL/EUROCMAV format.
     * If we can't open the file, or it is in the wrong format we will error and exit the program.
     * See get_gt_state() for a way to get the groundtruth state at a given timestep
     */
    void load_gt_file(std::string path) {

        // Clear any old data
        gt_states.clear();

        // Open the file
        std::ifstream file;
        std::string line;
        file.open(path);

        // Check that it was successfull
        if (!file) {
            ROS_ERROR("ERROR: Unable to open groundtruth file...");
            ROS_ERROR("ERROR: %s", path.c_str());
            std::exit(EXIT_FAILURE);
        }

        // Skip the first line as it is just the header
        std::getline(file, line);

        // Loop through each line in the file
        while (std::getline(file, line) && ros::ok()) {
            // Loop variables
            int i = 0;
            std::istringstream s(line);
            std::string field;
            Eigen::Matrix<double, 17, 1> temp;
            // Loop through this line
            while (getline(s, field, ',')) {
                // Ensure we are in the range
                if (i > 16) {
                    ROS_ERROR("ERROR: Invalid groudtruth line, too long!");
                    ROS_ERROR("ERROR: %s", line.c_str());
                    std::exit(EXIT_FAILURE);
                }
                // Save our groundtruth state value
                temp(i, 0) = std::atof(field.c_str());
                i++;
            }
            // Append to our groundtruth map
            gt_states.insert({1e-9 * temp(0, 0), temp});
        }
        file.close();
    }


    /**
     * @brief Gets the 16x1 groundtruth state at a given timestep
     * @param timestep timestep we want to get the groundtruth for
     * @param imustate groundtruth state [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
     * @return true if we found the state, false otherwise
     */
    bool get_gt_state(double timestep, Eigen::Matrix<double, 17, 1> &imustate) {

        // Check that we even have groundtruth loaded
        if (gt_states.empty()) {
            ROS_ERROR_THROTTLE(5,
                               "Groundtruth data loaded is empty, make sure you call load before asking for a state.");
            return false;
        }

        // Check that we have the timestamp in our GT file
        if (gt_states.find(timestep) == gt_states.end()) {
            ROS_WARN_THROTTLE(5, "Unable to find %.6f timestamp in GT file, wrong GT file loaded???", timestep);
            return false;
        }

        // Get the GT state vector
        Eigen::Matrix<double, 17, 1> state = gt_states[timestep];

        // Our "fixed" state vector from the ETH GT format [q,p,v,bg,ba]
        imustate(0, 0) = state(5, 0); //quat
        imustate(1, 0) = state(6, 0);
        imustate(2, 0) = state(7, 0);
        imustate(3, 0) = state(4, 0);
        imustate(4, 0) = state(1, 0); //pos
        imustate(5, 0) = state(2, 0);
        imustate(6, 0) = state(3, 0);
        imustate(7, 0) = state(8, 0); //vel
        imustate(8, 0) = state(9, 0);
        imustate(9, 0) = state(10, 0);
        imustate(10, 0) = state(11, 0); //bg
        imustate(11, 0) = state(12, 0);
        imustate(12, 0) = state(13, 0);
        imustate(13, 0) = state(14, 0); //ba
        imustate(14, 0) = state(15, 0);
        imustate(15, 0) = state(16, 0);

        // Success!
        return true;
    }

}


#endif /* OV_CORE_DATASET_READER_H */