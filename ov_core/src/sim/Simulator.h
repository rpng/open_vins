#ifndef OV_CORE_SIMULATOR_H
#define OV_CORE_SIMULATOR_H


#include <fstream>
#include <sstream>
#include <random>
#include <string>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>


#include "BsplineSE3.h"



/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {



    /**
     * @brief Master simulator class that will create artifical measurements for our visual-inertial algorithms.
     */
    class Simulator {

    public:


        /**
         * @brief Default constructor
         * @param path_traj Path to the simulated trajectory
         */
        Simulator(std::string path_traj) {
            load_data(path_traj);
            spline.feed_trajectory(traj_data);
        }





    protected:


        /**
         * @brief This will load the trajectory into memory.
         * @param path_traj Path to the trajectory file that we want to read in.
         */
        void load_data(std::string path_traj);


        /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
        std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_data;

        /// Our b-spline trajectory
        BsplineSE3 spline;


    };


}

#endif //OV_CORE_SIMULATOR_H
