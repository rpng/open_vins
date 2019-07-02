#ifndef OV_CORE_TRACK_SIM_H
#define OV_CORE_TRACK_SIM_H


#include "TrackBase.h"


namespace ov_core {


    /**
     * @brief Simulated tracker for when we already have uv measurements!
     *
     * This class should be used when we are using the @ref Simulator class to generate measurements.
     */
    class TrackSIM : public TrackBase {

    public:

        /**
         * @brief Public constructor with configuration variables
         * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
         */
        TrackSIM(int numaruco) : TrackBase(0, numaruco) {}

        /**
         * @brief Set the width and height for the cameras
         * @param _camera_wh Width and height for each camera
         */
        void set_width_height(std::map<size_t,std::pair<int,int>> _camera_wh) {
            this->camera_wh = _camera_wh;
        }

        /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
        void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) override {
            ROS_ERROR("[SIM]: SIM TRACKER FEED MONOCULAR CALLED!!!");
            ROS_ERROR("[SIM]: THIS SHOULD NEVER HAPPEN!");
            std::exit(EXIT_FAILURE);
        }

        /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
        void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) override {
            ROS_ERROR("[SIM]: SIM TRACKER FEED STEREO CALLED!!!");
            ROS_ERROR("[SIM]: THIS SHOULD NEVER HAPPEN!");
            std::exit(EXIT_FAILURE);
        }

        /**
         * @brief Feed function for a synchronized simulated cameras
         * @param timestamp Time that this image was collected
         * @param camids Camera ids that we have simulated measurements for
         * @param feats Raw uv simulated measurements
         */
        void feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats);


    protected:

        /// Width and height of our cameras
        std::map<size_t,std::pair<int,int>> camera_wh;


    };


}


#endif /* OV_CORE_TRACK_SIM_H */