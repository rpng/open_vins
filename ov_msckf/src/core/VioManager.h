#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"


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




    protected:


        /// Our master state object :D
        State* state;

        /// Our sparse feature tracker (klt or descriptor)
        TrackBase* trackFEATS;

        /// Our aruoc tracker
        TrackBase* trackARUCO;



    };


}



#endif //OV_MSCKF_VIOMANAGER_H
