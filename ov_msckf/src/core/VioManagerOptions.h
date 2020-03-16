/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
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
#ifndef OV_MSCKF_VIOMANAGEROPTIONS_H
#define OV_MSCKF_VIOMANAGEROPTIONS_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include "utils/colors.h"
#include "utils/quat_ops.h"


using namespace std;
using namespace ov_core;



namespace ov_msckf {


    /**
     * @brief Struct which stores all options needed for state estimation.
     *
     * This is broken into a few different parts: estimator, trackers, and simulation.
     * If you are going to add a parameter here you will need to add it to the parsers.
     * You will also need to add it to the print statement at the bottom of each.
     */
    struct VioManagerOptions {



        // ESTIMATOR ===============================

        /// Number of cameras we should process. These should all be time synchronized. 1=mono, 2=stereo/binocular
        int max_cameras = 1;

        /// If we should process two cameras are being stereo or binocular. If binocular, we do monocular feature tracking on each image.
        bool use_stereo = true;

        /// Gravity in the global frame (i.e. should be [0, 0, 9.81] typically)
        Eigen::Vector3d gravity = {0.0, 0.0, 9.81};


        /**
         * @brief This function will print out all estimator settings loaded.
         * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
         */
        void print_estimator() {
            printf("\t- max_cameras: %d\n", max_cameras);
            printf("\t- use_stereo: %d\n", use_stereo);
            printf("\t- gravity: %.3f, %.3f, %.3f\n", gravity(0), gravity(1), gravity(2));
        }



        // NOISE / CHI2 ============================

        /// IMU gyroscope "white noise" (rad/s/sqrt(hz))
        double gyroscope_noise_density = 1.6968e-04;

        /// IMU accelerometer "white noise" (m/s^2/sqrt(hz))
        double accelerometer_noise_density = 2.0000e-3;

        /// IMU gyroscope "random walk" (rad/s^2/sqrt(hz))
        double gyroscope_random_walk = 1.9393e-05;

        /// IMU accelerometer "random walk" (m/s^3/sqrt(hz))
        double accelerometer_random_walk = 3.0000e-03;

        /// Image reprojection pixel noise on the *raw* image (this is not on the normalized image plane).
        double up_msckf_sigma_px = 1;


        /**
         * @brief This function will print out all noise parameters loaded.
         * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
         */
        void print_noise() {
            printf("\t- gyroscope_noise_density: %.2f\n", gyroscope_noise_density);
            printf("\t- accelerometer_noise_density: %.2f\n", accelerometer_noise_density);
            printf("\t- gyroscope_random_walk: %.2f\n", gyroscope_random_walk);
            printf("\t- accelerometer_random_walk: %.2f\n", accelerometer_random_walk);
            printf("\t- up_msckf_sigma_px: %.2f\n", up_msckf_sigma_px);
        }


        // STATE DEFAULTS ==========================

        /// Time offset between camera and IMU.
        double calib_camimu_dt = 0.0;

        /// Map between camid and camera model (true=fisheye, false=radtan)
        std::map<size_t,bool> camera_fisheye;

        /// Map between camid and intrinsics. Values depends on the model but each should be a 4x1 vector normally.
        std::map<size_t,Eigen::VectorXd> camera_intrinsics;

        /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
        std::map<size_t,Eigen::VectorXd> camera_extrinsics;

        /// Map between camid and the dimensions of incoming images (width/cols, height/rows). This is normally only used during simulation.
        std::map<size_t,std::pair<int,int>> camera_wh;



        /**
         * @brief This function will print out all simulated parameters loaded.
         * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
         */
        void print_state() {
            printf("\t- calib_camimu_dt: %.4f\n", calib_camimu_dt);
            for(int n=0; n<max_cameras; n++) {
                std::cout << "cam_" << n << "_fisheye:" << camera_fisheye.at(n) << std::endl;
                std::cout << "cam_" << n << "_wh:" << endl << camera_wh.at(n).first << " x " << camera_wh.at(n).second << std::endl;
                std::cout << "cam_" << n << "_intrinsic(0:3):" << endl << camera_intrinsics.at(n).block(0,0,4,1).transpose() << std::endl;
                std::cout << "cam_" << n << "_intrinsic(4:7):" << endl << camera_intrinsics.at(n).block(4,0,4,1).transpose() << std::endl;
                std::cout << "cam_" << n << "_extrinsic(0:3):" << endl << camera_extrinsics.at(n).block(0,0,4,1).transpose() << std::endl;
                std::cout << "cam_" << n << "_extrinsic(4:6):" << endl << camera_extrinsics.at(n).block(4,0,3,1).transpose() << std::endl;
                Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
                T_CtoI.block(0,0,3,3) = quat_2_Rot(camera_extrinsics.at(n).block(0,0,4,1)).transpose();
                T_CtoI.block(0,3,3,1) = T_CtoI.block(0,0,3,3)*camera_extrinsics.at(n).block(4,0,3,1);
                std::cout << "T_C" << n << "toI:" << endl << T_CtoI << std::endl << std::endl;
            }
        }


        // TRACKERS ===============================

        /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
        int num_pts = 100;






        // SIMULATOR ===============================

        /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
        string sim_traj_path = "../ov_data/sim/udel_gore.txt";

        /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init from groundtruth in simulation.
        double sim_distance_threshold = 1.0;

        /// Frequency (Hz) that we will simulate our cameras
        double sim_freq_cam = 10.0;

        /// Frequency (Hz) that we will simulate our inertial measurement unit
        double sim_freq_imu = 400.0;

        /// Seed for initial states (i.e. random feature 3d positions in the generated map)
        int sim_seed_state_init = 0;

        /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are enabled.
        int sim_seed_preturb = 0;

        /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements, but diffferent noise values.
        int sim_seed_measurements = 0;



        /**
         * @brief This function will print out all simulated parameters loaded.
         * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
         */
        void print_simulation() {
            printf(BOLDRED "\t- state init seed: %d \n" RESET, sim_seed_state_init);
            printf(BOLDRED "\t- perturb seed: %d \n" RESET, sim_seed_preturb);
            printf(BOLDRED "\t- measurement seed: %d \n" RESET, sim_seed_measurements);
            printf("\t- traj path: %s\n", sim_traj_path.c_str());
            printf("\t- dist thresh: %.2f\n", sim_distance_threshold);
            printf("\t- cam feq: %.2f\n", sim_freq_cam);
            printf("\t- imu feq: %.2f\n", sim_freq_imu);
        }



    };

}

#endif //OV_MSCKF_VIOMANAGEROPTIONS_H