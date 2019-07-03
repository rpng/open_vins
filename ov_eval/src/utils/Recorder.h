#ifndef OV_EVAL_RECORDER_H
#define OV_EVAL_RECORDER_H


#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace ov_eval {


    /**
     * @brief This class takes in published poses and writes them to file.
     *
     * Original code is based on this modified [posemsg_to_file](https://github.com/rpng/posemsg_to_file/).
     * Output is in a text file that is space deliminated.
     */
    class Recorder {

    public:

        /**
         * @brief Default constructor that will open the specified file on disk.
         * If the output directory does not exists this will also create the directory path to this file.
         * @param filename Desired file we want to "record" into
         */
        Recorder(std::string filename) {
            // Create folder path to this location if not exists
            boost::filesystem::path dir(filename.c_str());
            if(boost::filesystem::create_directories(dir.parent_path())) {
                ROS_INFO("Created folder path to output file.");
                ROS_INFO("Path: %s",dir.parent_path().c_str());
            }
            // Open this file we want to write to
            outfile.open(filename.c_str());
            if(outfile.fail()) {
                ROS_ERROR("Unable to open output file!!");
                ROS_ERROR("Path: %s",filename.c_str());
                std::exit(EXIT_FAILURE);
            }
            outfile << "# timestamp(s) tx ty tz qx qy qz qw P11 P22 P33 P44 P55 P66" << std::endl;
            // Set initial state values
            timestamp = -1;
            q_ItoG << 0,0,0,1;
            p_IinG = Eigen::Vector3d::Zero();
            cov_diag = Eigen::Matrix<double,6,1>::Zero();
        }

        /**
         * @brief Callback for nav_msgs::Odometry message types.
         * @param msg New message
         */
        void callback_odometry(const nav_msgs::OdometryPtr &msg) {
            timestamp = msg->header.stamp.toSec();
            q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
            p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            cov_diag << msg->pose.covariance.at(0),msg->pose.covariance.at(7),msg->pose.covariance.at(14),
                        msg->pose.covariance.at(21),msg->pose.covariance.at(28),msg->pose.covariance.at(35);
            write();
        }

        /**
         * @brief Callback for geometry_msgs::PoseStamped message types
         * @param msg New message
         */
        void callback_pose(const geometry_msgs::PoseStampedPtr &msg) {
            timestamp = msg->header.stamp.toSec();
            q_ItoG << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
            p_IinG << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
            write();
        }

        /**
         * @brief Callback for geometry_msgs::PoseWithCovarianceStamped message types
         * @param msg New message
         */
        void callback_posecovariance(const geometry_msgs::PoseWithCovarianceStampedPtr &msg) {
            timestamp = msg->header.stamp.toSec();
            q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
            p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
            cov_diag << msg->pose.covariance.at(0),msg->pose.covariance.at(7),msg->pose.covariance.at(14),
                    msg->pose.covariance.at(21),msg->pose.covariance.at(28),msg->pose.covariance.at(35);
            write();
        }

        /**
         * @brief Callback for geometry_msgs::TransformStamped message types
         * @param msg New message
         */
        void callback_transform(const geometry_msgs::TransformStampedPtr &msg) {
            timestamp = msg->header.stamp.toSec();
            q_ItoG << msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w;
            p_IinG << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;
            write();
        }


    protected:

        /**
         * @brief This is the main write function that will save to disk.
         * This should be called after we have saved the desired pose to our class variables.
         */
        void write() {
            outfile.precision(5);
            outfile.setf(std::ios::fixed, std::ios::floatfield);
            outfile << timestamp << " ";
            outfile.precision(6);
            outfile << p_IinG.x() << " " << p_IinG.y() << " " << p_IinG.z() << " "
                    << q_ItoG(0) << " " << q_ItoG(1) << " " << q_ItoG(2) << " " << q_ItoG(3) << " "
                    << cov_diag(0) << " " << cov_diag(1) << " " << cov_diag(2) << " "
                    << cov_diag(3) << " " << cov_diag(4) << " " << cov_diag(5) << " " << std::endl;
        }


        // Output stream file
        std::ofstream outfile;

        // Temp storage objects for our pose and its certainty
        double timestamp;
        Eigen::Vector4d q_ItoG;
        Eigen::Vector3d p_IinG;
        Eigen::Matrix<double,6,1> cov_diag;

    };



}

#endif //OV_EVAL_RECORDER_H
