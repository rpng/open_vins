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


            std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_data_temp;
            for(size_t i=0; i<traj_data.size(); i+=10) {
                traj_data_temp.push_back(traj_data.at(i));
            }

            spline.feed_trajectory(traj_data_temp);

            for(size_t i=0; i<traj_data.size()-1; i++) {

                Eigen::Matrix3d R_GtoI;
                Eigen::Vector3d p_IinG;
                bool success = spline.get_pose(traj_data.at(i)(0),R_GtoI,p_IinG);

                Eigen::Vector3d w_IinG, v_IinG;
                success = spline.get_velocity(traj_data.at(i)(0),w_IinG,v_IinG);
                //std::cout << traj_data.at(i)(1) << "," << traj_data.at(i)(2) << "," << traj_data.at(i)(3) << "," << p_IinG(0) << "," << p_IinG(1) << "," << p_IinG(2) << std::endl;

                Eigen::Vector3d alpha_IinG, a_IinG;
                success = spline.get_acceleration(traj_data.at(i)(0),alpha_IinG,a_IinG);
                //std::cout << w_IinG(0) << "," << w_IinG(1) << "," << w_IinG(2) << "," << a_IinG(0) << "," << a_IinG(1) << "," << a_IinG(2) << std::endl;


            }

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
