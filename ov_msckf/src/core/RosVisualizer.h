#ifndef OV_MSCKF_ROSVISUALIZER_H
#define OV_MSCKF_ROSVISUALIZER_H



#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>


#include "VioManager.h"
#include "utils/dataset_reader.h"


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Helper class that will publish results onto the ROS framework
     */
    class RosVisualizer {

    public:

        /**
         * @brief Default constructor
         * @param nh ROS node handler
         * @param app Core estimator manager
         */
        RosVisualizer(ros::NodeHandle &nh, VioManager* app);


        /**
         * @brief Will visualize the system if we have new things
         */
        void visualize();


    protected:

        /// Publish the current state
        void publish_state();

        /// Publish the active tracking image
        void publish_images();

        /// Publish current features
        void publish_features();

        /// Publish groundtruth (if we have it)
        void publish_groundtruth();



        /// ROS node handle that we publish onto
        ros::NodeHandle _nh;

        /// Core application of the filter system
        VioManager* _app;

        // Our publishers
        ros::Publisher pub_poseimu;
        ros::Publisher pub_pathimu;
        ros::Publisher pub_points_msckf;
        ros::Publisher pub_points_slam;
        ros::Publisher pub_points_aruco;
        ros::Publisher pub_tracks;
        tf::TransformBroadcaster *mTfBr;

        // For path viz
        unsigned int poses_seq_imu = 0;
        vector<geometry_msgs::PoseStamped> poses_imu;

        // Groundtruth infomation
        ros::Publisher pub_pathgt;
        ros::Publisher pub_posegt;
        double summed_rmse_ori = 0.0;
        double summed_rmse_pos = 0.0;
        size_t summed_number = 0;

        // Our groundtruth states
        std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

        // For path viz
        unsigned int poses_seq_gt = 0;
        vector<geometry_msgs::PoseStamped> poses_gt;



    };


}


#endif //OV_MSCKF_ROSVISUALIZER_H
