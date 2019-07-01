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

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>


#include "BsplineSE3.h"



/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {



    /**
     * @brief Master simulator class that will create artificial measurements for our visual-inertial algorithms.
     *
     * Given a trajectory this will generate a SE(3) bsline for that trajectory.
     * This allows us to get the inertial measurement information at each timestep during this trajectory.
     * After creating the bspline we will generate an environmental feature map which will be used as our feature measurements.
     * This map will be projected into the frame at each timestep to get our "raw" uv measurements.
     * We inject bias and white noises into our inertial readings while adding our white noise to the uv measurements also.
     * The user should specify the sensor rates that they desire along with the seeds of the random number generators.
     *
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
         * @brief Get the simulation state at the current timestep
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         * @return True if we have a state
         */
        bool get_current_state(Eigen::Matrix<double, 17, 1> &imustate);

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
         * @param feats Noisy uv measurements and ids for the returned time
         * @return True if we have a measurement
         */
        bool get_next_cam(double &time_cam, std::vector<int> &camids, std::vector<std::vector<std::pair<size_t,Eigen::Vector2d>>> &feats);


        /**
         * @brief Returns the whole map (used for visualization)
         * @return Map of 3d features in the global simulation frame
         */
        std::unordered_map<size_t,Eigen::Vector3d> get_map() {
            return featmap;
        }


    protected:


        /**
         * @brief This will load the trajectory into memory.
         * @param path_traj Path to the trajectory file that we want to read in.
         */
        void load_data(std::string path_traj);

        /**
         * @brief Projects the passed map features into the desired camera frame.
         * @param R_GtoI Orientation of the IMU pose
         * @param p_IinG Position of the IMU pose
         * @param camid Camera id of the camera sensor we want to project into
         * @param feats Our set of 3d features
         * @return True distorted raw image measurements and their ids for the specified camera
         */
        std::vector<std::pair<size_t,Eigen::Vector2d>> project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid, const std::unordered_map<size_t,Eigen::Vector3d> &feats);


        /**
         * @brief Will generate points in the fov of the specified camera
         * @param R_GtoI Orientation of the IMU pose
         * @param p_IinG Position of the IMU pose
         * @param camid Camera id of the camera sensor we want to project into
         * @param[out] feats Map we will append new features to
         * @param numpts Number of points we should generate
         */
        void generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, int camid, std::unordered_map<size_t,Eigen::Vector3d> &feats, int numpts);

        //===================================================================
        // State related variables
        //===================================================================

        /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
        std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_data;

        /// Our b-spline trajectory
        BsplineSE3 spline;

        /// Our map of 3d features
        size_t id_map = 0;
        std::unordered_map<size_t,Eigen::Vector3d> featmap;

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
        std::unordered_map<size_t,std::pair<double,double>> camera_wh;
        std::unordered_map<size_t,Eigen::Matrix<double,8,1>> camera_intrinsics;
        std::unordered_map<size_t,Eigen::Matrix<double,7,1>> camera_extrinsics;

        /// Max number of features to have in a single image
        int num_pts;

        // Our sensor noises
        double sigma_w,sigma_a,sigma_wb,sigma_ab,sigma_pix;


    };


}

#endif //OV_CORE_SIMULATOR_H
