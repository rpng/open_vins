#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include <string>
#include <algorithm>

#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "types/Landmark.h"


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
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
         * @param nh ROS node handler which we will load parameters from
         */
        VioManager(ros::NodeHandle& nh);


        /**
         * @brief Feed function for inertial data
         * @param timestamp Time of the inertial measurement
         * @param wm Angular velocity
         * @param am Linear acceleration
         */
        void feed_measurement_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am);


        /**
         * @brief Feed function for a single camera
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param cam_id Unique id of what camera the image is from
         */
        void feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id);

        /**
         * @brief Feed function for stereo camera pair
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param img1 Grayscale image
         * @param cam_id0 Unique id of what camera the image is from
         * @param cam_id1 Unique id of what camera the image is from
         */
        void feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1);


        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate) {

            // Initialize the system
            state->imu()->set_value(imustate.block(1,0,16,1));
            state->set_timestamp(imustate(0,0));
            is_initialized_vio = true;

            // Print what we init'ed with
            ROS_INFO("\033[0;32m[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\033[0m");
            ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
            ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
            ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
            ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
            ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
        }


        /// If we are initialized or not
        bool intialized() {
            return is_initialized_vio;
        }

        /// Accessor to get the current state
        State* get_state() {
            return state;
        }

        /// Get feature tracker
        TrackBase* get_track_feat() {
            return trackFEATS;
        }

        /// Get aruco feature tracker
        TrackBase* get_track_aruco() {
            return trackARUCO;
        }

        /// Returns 3d features used in the last update in global frame
        std::vector<Eigen::Vector3d> get_good_features_MSCKF() {
            return good_features_MSCKF;
        }

        /// Returns 3d SLAM features in the global frame
        std::vector<Eigen::Vector3d> get_features_SLAM() {
            std::vector<Eigen::Vector3d> slam_feats;
            for (auto &f : state->features_SLAM()){
                if((int)f.first <= state->options().max_aruco_features) continue;
                slam_feats.push_back(f.second->get_global_xyz(state, false));
            }
            return slam_feats;
        }

        /// Returns 3d ARUCO features in the global frame
        std::vector<Eigen::Vector3d> get_features_ARUCO() {
            std::vector<Eigen::Vector3d> aruco_feats;
            for (auto &f : state->features_SLAM()){
                if((int)f.first > state->options().max_aruco_features) continue;
                aruco_feats.push_back(f.second->get_global_xyz(state, false));
            }
            return aruco_feats;
        }



    protected:


        /**
         * @brief This function will try to initialize the state.
         *
         * This should call on our initalizer and try to init the state.
         * In the future we should call the structure-from-motion code from here.
         * This function could also be repurposed to re-initialize the system after failure.         *
         * @return True if we have successfully initialized
         */
        bool try_to_initialize();


        /**
         * @brief This will do the propagation and feature updates to the state
         * @param timestamp The most recent timestamp we have tracked to
         */
        void do_feature_propagate_update(double timestamp);


        /// Our master state object :D
        State* state;

        /// Propagator of our state
        Propagator* propagator;

        /// Our sparse feature tracker (klt or descriptor)
        TrackBase* trackFEATS = nullptr;

        /// Our aruoc tracker
        TrackBase* trackARUCO = nullptr;

        /// State initializer
        InertialInitializer* initializer;

        /// Boolean if we are initialized or not
        bool is_initialized_vio = false;

        /// Our MSCKF feature updater
        UpdaterMSCKF* updaterMSCKF;

        /// Our MSCKF feature updater
        UpdaterSLAM* updaterSLAM;

        /// Good features that where used in the last update
        std::vector<Eigen::Vector3d> good_features_MSCKF;

        // Timing variables
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6;

        // Track how much distance we have traveled
        double timelastupdate = -1;
        double distance = 0;

    };


}



#endif //OV_MSCKF_VIOMANAGER_H
