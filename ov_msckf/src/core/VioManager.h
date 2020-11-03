/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
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
#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include <string>
#include <algorithm>
#include <fstream>
#include <memory>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>

#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "init/InertialInitializer.h"
#include "types/LandmarkRepresentation.h"
#include "types/Landmark.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

#include "VioManagerOptions.h"


namespace ov_msckf {


    /**
     * @brief Core class that manages the entire system
     *
     * This class contains the state and other algorithms needed for the MSCKF to work.
     * We feed in measurements into this class and send them to their respective algorithms.
     * If we have measurements to propagate or update with, this class will call on our state to do that.
     */
    class VioManager {


    public:


        /**
         * @brief Default constructor, will load all configuration variables
         * @param params_ Parameters loaded from either ROS or CMDLINE
         */
        VioManager(VioManagerOptions &params_);


        /**
         * @brief Feed function for inertial data
         * @param timestamp Time of the inertial measurement
         * @param wm Angular velocity
         * @param am Linear acceleration
         */
        void feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);


        /**
         * @brief Feed function for a single camera
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param cam_id Unique id of what camera the image is from
         */
        void feed_measurement_monocular(double timestamp, cv::Mat &img0, size_t cam_id);

        /**
         * @brief Feed function for stereo camera pair
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param img1 Grayscale image
         * @param cam_id0 Unique id of what camera the image is from
         * @param cam_id1 Unique id of what camera the image is from
         */
        void feed_measurement_stereo(double timestamp, cv::Mat &img0, cv::Mat &img1, size_t cam_id0, size_t cam_id1);

        /**
         * @brief Feed function for a synchronized simulated cameras
         * @param timestamp Time that this image was collected
         * @param camids Camera ids that we have simulated measurements for
         * @param feats Raw uv simulated measurements
         */
        void feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate) {

            // Initialize the system
            state->_imu->set_value(imustate.block(1, 0, 16, 1));
            state->_imu->set_fej(imustate.block(1, 0, 16, 1));
            state->_timestamp = imustate(0, 0);
            startup_time = imustate(0, 0);
            is_initialized_vio = true;

            // Cleanup any features older then the initialization time
            trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
            if (trackARUCO != nullptr) {
                trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
            }

            // Print what we init'ed with
            printf(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
            printf(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET, state->_imu->quat()(0), state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3));
            printf(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2));
            printf(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET, state->_imu->vel()(0), state->_imu->vel()(1), state->_imu->vel()(2));
            printf(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));
            printf(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET, state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));

        }


        /// If we are initialized or not
        bool initialized() {
            return is_initialized_vio;
        }

        /// Timestamp that the system was initialized at
        double initialized_time() {
            return startup_time;
        }

        /// Accessor for current system parameters
        const VioManagerOptions get_params() {
            return params;
        }

        /// Accessor to get the current state
        std::shared_ptr<State> get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        std::shared_ptr<Propagator> get_propagator() {
            return propagator;
        }

        /// Get feature tracker
        std::shared_ptr<TrackBase> get_track_feat() {
            return trackFEATS;
        }

        /// Get aruco feature tracker
        std::shared_ptr<TrackBase> get_track_aruco() {
            return trackARUCO;
        }

        /// Returns 3d features used in the last update in global frame
        std::vector<Eigen::Vector3d> get_good_features_MSCKF() {
            return good_features_MSCKF;
        }

        /// Returns 3d SLAM features in the global frame
        std::vector<Eigen::Vector3d> get_features_SLAM() {
            std::vector<Eigen::Vector3d> slam_feats;
            for (auto &f : state->_features_SLAM) {
                if ((int) f.first <= state->_options.max_aruco_features) continue;
                if (LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id != -1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    slam_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    slam_feats.push_back(f.second->get_xyz(false));
                }
            }
            return slam_feats;
        }

        /// Returns 3d ARUCO features in the global frame
        std::vector<Eigen::Vector3d> get_features_ARUCO() {
            std::vector<Eigen::Vector3d> aruco_feats;
            for (auto &f : state->_features_SLAM) {
                if ((int) f.first > state->_options.max_aruco_features) continue;
                if (LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id != -1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    aruco_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    aruco_feats.push_back(f.second->get_xyz(false));
                }
            }
            return aruco_feats;
        }

        /// Return true if we did a zero velocity update
        bool did_zero_velocity_update() {
            return did_zupt_update;
        }

        /// Return the zero velocity update image
        cv::Mat get_zero_velocity_update_image() {
            return zupt_image;
        }

        /// Return the zero velocity update image
        void get_active_image(double &timestamp, cv::Mat &image) {
            timestamp = active_tracks_time;
            image = active_image;
        }

        /// Returns active tracked features in the current frame
        void get_active_tracks(double &timestamp,
                               std::unordered_map<size_t, Eigen::Vector3d> &feat_posinG,
                               std::unordered_map<size_t, Eigen::Vector3d> &feat_tracks_uvd) {
            timestamp = active_tracks_time;
            feat_posinG = active_tracks_posinG;
            feat_tracks_uvd = active_tracks_uvd;
        }


    protected:


        /**
         * @brief This function will try to initialize the state.
         *
         * This should call on our initializer and try to init the state.
         * In the future we should call the structure-from-motion code from here.
         * This function could also be repurposed to re-initialize the system after failure.         *
         * @return True if we have successfully initialized
         */
        bool try_to_initialize();


        /**
         * @brief This will do the propagation and feature updates to the state
         * @param timestamp The most recent timestamp we have tracked to
         * @param cam0_image First camera image mat object
         */
        void do_feature_propagate_update(double timestamp, cv::Mat &cam0_image);


        /**
         * @brief This function will will re-triangulate all features in the current frame
         *
         * For all features that are currently being tracked by the system, this will re-triangulate them.
         * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
         * This will try to triangulate *all* points, not just ones that have been used in the update.
         *
         * @param cam0_image First camera image mat object
         */
        void retriangulate_active_tracks(cv::Mat &cam0_image);


        /// Manager parameters
        VioManagerOptions params;

        /// Our master state object :D
        std::shared_ptr<State> state;

        /// Propagator of our state
        std::shared_ptr<Propagator> propagator;

        /// Our sparse feature tracker (klt or descriptor)
        std::shared_ptr<TrackBase> trackFEATS;

        /// Our aruoc tracker
        std::shared_ptr<TrackBase> trackARUCO;

        /// State initializer
        std::unique_ptr<InertialInitializer> initializer;

        /// Boolean if we are initialized or not
        bool is_initialized_vio = false;

        /// Our MSCKF feature updater
        std::unique_ptr<UpdaterMSCKF> updaterMSCKF;

        /// Our MSCKF feature updater
        std::unique_ptr<UpdaterSLAM> updaterSLAM;

        /// Our aruoc tracker
        std::unique_ptr<UpdaterZeroVelocity> updaterZUPT;

        // Timing statistic file and variables
        std::ofstream of_statistics;
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

        // Track how much distance we have traveled
        double timelastupdate = -1;
        double distance = 0;

        // Startup time of the filter
        double startup_time = -1;

        // If we did a zero velocity update
        bool did_zupt_update = false;
        cv::Mat zupt_image;

        // Good features that where used in the last update (used in visualization)
        std::vector<Eigen::Vector3d> good_features_MSCKF;

        /// Feature initializer used to triangulate all active tracks
        std::unique_ptr<FeatureInitializer> active_tracks_initializer;

        // Re-triangulated features 3d positions seen from the current frame (used in visualization)
        double active_tracks_time = -1;
        std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
        std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
        cv::Mat active_image;


    };


}


#endif //OV_MSCKF_VIOMANAGER_H
