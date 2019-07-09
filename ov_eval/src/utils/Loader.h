#ifndef OV_EVAL_LOADER_H
#define OV_EVAL_LOADER_H


#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include <ros/ros.h>


namespace ov_eval {


    /**
     * @brief Has helper functions to load text files from disk and process them.
     */
    class Loader {

    public:

        /**
         * @brief This will load the trajectory into memory
         * @param path_traj Path to the trajectory file that we want to read in.
         * @param times Timesteps in seconds
         * @param poses Pose at every timestep [pos,quat]
         * @param cov_ori Vector of orientation covariances at each timestep (empty if we can't load)
         * @param cov_pos Vector of position covariances at each timestep (empty if we can't load)
         */
        static void load_data(std::string path_traj,
                              std::vector<double> &times, std::vector<Eigen::Matrix<double,7,1>> &poses,
                              std::vector<Eigen::Matrix3d> &cov_ori, std::vector<Eigen::Matrix3d> &cov_pos);



        /**
         * @brief Will calculate the total trajectory distance
         * @param poses Pose at every timestep [pos,quat]
         * @return Distance travels (meters)
         */
        static double get_total_length(const std::vector<Eigen::Matrix<double,7,1>> &poses);

    private:

        /**
         * All function in this class should be static.
         * Thus an instance of this class cannot be created.
         */
        Loader() {}

    };



}

#endif //OV_EVAL_LOADER_H
