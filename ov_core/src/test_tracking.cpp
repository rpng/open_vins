#include <cmath>
#include <vector>
#include <deque>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "track/TrackKLT.h"
#include "track/TrackDescriptor.h"
#include "track/TrackAruco.h"

using namespace ov_core;

// Our feature extractor
TrackBase* extractor;

// FPS counter, and other statistics
// https://gamedev.stackexchange.com/a/83174
int frames = 0;
int num_lostfeats = 0;
int num_margfeats = 0;
int featslengths = 0;
int clone_states = 10;
std::deque<double> clonetimes;
ros::Time time_start;

// Our master function for tracking
void handle_stereo(double time0, double time1, cv::Mat img0, cv::Mat img1);


// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_tracking");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Our camera topics (left and right stereo)
    std::string topic_camera0;
    std::string topic_camera1;
    nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
    //nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V2_03_difficult.bag");
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 2);
    nh.param<double>("bag_durr", bag_durr, -1);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Parameters for our extractor
    int num_pts, num_aruco, fast_threshold, grid_x, grid_y, min_px_dist;
    double knn_ratio;
    bool do_downsizing;
    nh.param<int>("num_pts", num_pts, 1000);
    nh.param<int>("num_aruco", num_aruco, 1024);
    nh.param<int>("clone_states", clone_states, 10);
    nh.param<int>("fast_threshold", fast_threshold, 15);
    nh.param<int>("grid_x", grid_x, 5);
    nh.param<int>("grid_y", grid_y, 3);
    nh.param<int>("min_px_dist", min_px_dist, 10);
    nh.param<double>("knn_ratio", knn_ratio, 0.85);
    nh.param<bool>("downsize_aruco", do_downsizing, false);

    // Debug print!
    ROS_INFO("max features: %d", num_pts);
    ROS_INFO("max aruco: %d", num_aruco);
    ROS_INFO("clone states: %d", clone_states);
    ROS_INFO("grid size: %d x %d", grid_x, grid_y);
    ROS_INFO("fast threshold: %d", fast_threshold);
    ROS_INFO("min pixel distance: %d", min_px_dist);
    ROS_INFO("downsize aruco image: %d", do_downsizing);

    // Fake camera info (we don't need this, as we are not using the normalized coordinates for anything)
    Eigen::Matrix<double,8,1> cam0_calib;
    cam0_calib << 1,1,0,0,0,0,0,0;

    // Create our n-camera vectors
    std::unordered_map<size_t,bool> camera_fisheye;
    std::unordered_map<size_t,Eigen::Matrix<double,8,1>> camera_calibration;
    camera_fisheye.insert({0,false});
    camera_calibration.insert({0,cam0_calib});
    camera_fisheye.insert({1,false});
    camera_calibration.insert({1,cam0_calib});

    // Lets make a feature extractor
    extractor = new TrackKLT(camera_calibration,camera_fisheye,num_pts,num_aruco,fast_threshold,grid_x,grid_y,min_px_dist);
    //extractor = new TrackDescriptor(camera_calibration,camera_fisheye,num_pts,num_aruco,fast_threshold,grid_x,grid_y,knn_ratio);
    //extractor = new TrackAruco(camera_calibration,camera_fisheye,num_aruco,do_downsizing);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    // Record the start time for our FPS counter
    time_start = ros::Time::now();

    // Our stereo pair we have
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    double time0 = time_init.toSec();
    double time1 = time_init.toSec();

    // Step through the rosbag
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != NULL && m.getTopic() == topic_camera0) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s0, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable
            has_left = true;
            cv::equalizeHist(cv_ptr->image, img0);
            //img0 = cv_ptr->image.clone();
            time0 = cv_ptr->header.stamp.toSec();
        }

        //  Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != NULL && m.getTopic() == topic_camera1) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s1, sensor_msgs::image_encodings::MONO8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable
            has_right = true;
            cv::equalizeHist(cv_ptr->image, img1);
            //img1 = cv_ptr->image.clone();
            time1 = cv_ptr->header.stamp.toSec();
        }


        // If we have both left and right, then process
        if(has_left && has_right) {
            // process
            handle_stereo(time0, time1, img0, img1);
            // reset bools
            has_left = false;
            has_right = false;
        }


    }

    // Done!
    return EXIT_SUCCESS;
}



/**
 * This function will process the new stereo pair with the extractor!
 */
void handle_stereo(double time0, double time1, cv::Mat img0, cv::Mat img1) {

                    

    // Process this new image
    extractor->feed_stereo(time0, img0, img1, 0, 1);
    //extractor->feed_monocular(time0, img0, 2);


    // Display the resulting tracks
    cv::Mat img_active, img_history;
    extractor->display_active(img_active,255,0,0,0,0,255);
    extractor->display_history(img_history,0,255,255,255,255,255);

    // Show our image!
    cv::imshow("Active Tracks", img_active);
    cv::imshow("Track History", img_history);
    cv::waitKey(1);

    // Get lost tracks
    FeatureDatabase* database = extractor->get_feature_database();
    std::vector<Feature*> feats_lost = database->features_not_containing_newer(time0);
    num_lostfeats += feats_lost.size();

    // Mark theses feature pointers as deleted
    for(size_t i=0; i<feats_lost.size(); i++) {
        // Total number of measurements
        int total_meas = 0;
        for (auto const& pair : feats_lost[i]->timestamps) {
            total_meas += (int)pair.second.size();
        }
        // Update stats
        featslengths += total_meas;
        feats_lost[i]->to_delete = true;
    }

    // Push back the current time, as a clone time
    clonetimes.push_back(time0);

    // Marginalized features if we have reached 5 frame tracks
    if((int)clonetimes.size() >= clone_states) {
        // Remove features that have reached their max track length
        double margtime = clonetimes.at(0);
        clonetimes.pop_front();
        std::vector<Feature*> feats_marg = database->features_containing(margtime);
        num_margfeats += feats_marg.size();
        // Delete theses feature pointers
        for(size_t i=0; i<feats_marg.size(); i++) {
            feats_marg[i]->to_delete = true;
        }
    }

    // Tell the feature database to delete old features
    database->cleanup();

    // Debug print out what our current processing speed it
    // We want the FPS to be as high as possible
    ros::Time time_curr = ros::Time::now();
    //if (time_curr.toSec()-time_start.toSec() > 2) {
    if (frames > 60) {
        // Calculate the FPS
        double fps = (double) frames / (time_curr.toSec()-time_start.toSec());
        double lpf = (double) num_lostfeats / frames;
        double fpf = (double) featslengths / num_lostfeats;
        double mpf = (double) num_margfeats / frames;
        // DEBUG PRINT OUT
        ROS_INFO("fps = %.2f | lost_feats/frame = %.2f | track_length/lost_feat = %.2f | marg_tracks/frame = %.2f",fps,lpf,fpf,mpf);
        // Reset variables
        frames = 0;
        time_start = time_curr;
        num_lostfeats = 0;
        num_margfeats = 0;
        featslengths = 0;
    }
    frames++;

}

