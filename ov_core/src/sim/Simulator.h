#ifndef OV_CORE_SIMULATOR_H
#define OV_CORE_SIMULATOR_H


#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unordered_map>

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
         * @brief Default constructor, will load all configuration variables
         * @param nh ROS node handler which we will load parameters from
         */
        Simulator(ros::NodeHandle& nh);

        /**
         * @brief Returns if we are actively simulating
         * @return True if we still have simulation data
         */
        bool ok() {
            return is_running;
        }



        /**
         * @brief Gets the next inertial reading if we have one.
         * @param time_imu Time that this measurement occured at
         * @param wm Angular velocity measurement in the inertial frame
         * @param am Linear velocity in the inertial frame
         * @return True if we have a measurement
         */
        bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);


        /**
         * @brief Gets the next inertial reading if we have one.
         * @param time_cam Time that this measurement occured at
         * @param camids Camera ids that the corresponding vectors match
         * @param featids Feature ids of the following uvs
         * @param featuvs Noisy uv measurements for the returned time
         * @return True if we have a measurement
         */
        bool get_next_cam(double &time_cam, std::vector<int> &camids,
                          std::vector<size_t> &featids,
                          std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &featuvs);


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

        /// Mersenne twister PRNG for measurements
        std::mt19937 gen_meas;

        /// Mersenne twister PRNG for state initialization
        std::mt19937 gen_state_init;

        /// If our simulation is running
        bool is_running;

        //===================================================================
        // Simulation specific variables
        //===================================================================

        /// Current timestamp of the system
        double timestamp;

        /// Last time we had an IMU reading
        double timestamp_last_imu;

        /// Last time we had an CAMERA reading
        double timestamp_last_cam;

        /// Number of cameras we should simulate
        int max_cameras;

        /// Frequency of our camera sensors
        int freq_cam;

        /// Frequency of our imu sensor
        int freq_imu;

        /// Our running acceleration bias
        Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

        /// Our running gyroscope bias
        Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();


        //===================================================================
        // Sensor measurement specific variables
        //===================================================================

        /// Gravity in the global frame
        Eigen::Vector3d gravity;

        /// Timeoffset between camera and imu (t_imu = t_cam + t_off)
        double calib_camimu_dt;

        // Camera intrinsics that we will load in
        std::unordered_map<size_t,bool> camera_fisheye;
        std::unordered_map<size_t,double> camera_fov;
        std::unordered_map<size_t,Eigen::Matrix<double,8,1>> camera_intrinsics;
        std::unordered_map<size_t,Eigen::Matrix<double,7,1>> camera_extrinsics;

        /// Max number of features to have in a single image
        int num_pts;

        // Our sensor noises
        double sigma_w,sigma_a,sigma_wb,sigma_ab,sigma_pix;


    };


}

#endif //OV_CORE_SIMULATOR_H
