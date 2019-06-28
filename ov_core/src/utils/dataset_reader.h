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


    /**
     * @brief Load a ASL format groundtruth file
     * @param path Path to the CSV file of groundtruth data
     * @param gt_states Will be filled with groundtruth states
     *
     * Here we will try to load a groundtruth file that is in the ASL/EUROCMAV format.
     * If we can't open the file, or it is in the wrong format we will error and exit the program.
     * See get_gt_state() for a way to get the groundtruth state at a given timestep
     */
    void load_gt_file(std::string path, std::map<double, Eigen::Matrix<double, 17, 1>>& gt_states) {

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
     * @brief Gets the 17x1 groundtruth state at a given timestep
     * @param timestep timestep we want to get the groundtruth for
     * @param imustate groundtruth state [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
     * @param gt_states Should be loaded with groundtruth states, see load_gt_file() for details
     * @return true if we found the state, false otherwise
     */
    static bool get_gt_state(double timestep, Eigen::Matrix<double, 17, 1> &imustate, std::map<double, Eigen::Matrix<double, 17, 1>>& gt_states) {

        // Check that we even have groundtruth loaded
        if (gt_states.empty()) {
            ROS_ERROR_THROTTLE(5, "Groundtruth data loaded is empty, make sure you call load before asking for a state.");
            return false;
        }

        // Loop through gt states and find the closest time stamp
        double closest_time = INFINITY;
        auto it0 = gt_states.begin();
        while(it0 != gt_states.end() && ros::ok()) {
            if(std::abs(it0->first-timestep) < std::abs(closest_time-timestep)) {
                closest_time = it0->first;
            }
            it0++;
        }

        // If close to this timestamp, then use it
        if(std::abs(closest_time-timestep) < 0.005) {
            //ROS_INFO("init DT = %.4f", std::abs(closest_time-timestep));
            //ROS_INFO("timestamp = %.15f", closest_time);
            timestep = closest_time;
        }

        // Check that we have the timestamp in our GT file
        if(gt_states.find(timestep) == gt_states.end()) {
            ROS_WARN_THROTTLE(5,"Unable to find %.6f timestamp in GT file, wrong GT file loaded???",timestep);
            return false;
        }

        // Get the GT state vector
        Eigen::Matrix<double, 17, 1> state = gt_states[timestep];

        // Our "fixed" state vector from the ETH GT format [q,p,v,bg,ba]
        imustate(0, 0) = timestep; //time
        imustate(1, 0) = state(5, 0); //quat
        imustate(2, 0) = state(6, 0);
        imustate(3, 0) = state(7, 0);
        imustate(4, 0) = state(4, 0);
        imustate(5, 0) = state(1, 0); //pos
        imustate(6, 0) = state(2, 0);
        imustate(7, 0) = state(3, 0);
        imustate(8, 0) = state(8, 0); //vel
        imustate(9, 0) = state(9, 0);
        imustate(10, 0) = state(10, 0);
        imustate(11, 0) = 0;//state(11, 0); //bg
        imustate(12, 0) = 0;//state(12, 0);
        imustate(13, 0) = 0;//state(13, 0);
        imustate(14, 0) = 0;//state(14, 0); //ba
        imustate(15, 0) = 0;//state(15, 0);
        imustate(16, 0) = 0;//state(16, 0);

        // Success!
        return true;
    }

}


#endif /* OV_CORE_DATASET_READER_H */