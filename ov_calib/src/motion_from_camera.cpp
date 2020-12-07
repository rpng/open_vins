#include <iomanip>   // for setiosflags
#include <csignal>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>

// ov_core
#include "track/TrackArr.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"
#include "utils/parse_ros.h"

// ov_msckf
#include "sim/Simulator.h"

// ov_calib
#include "utils.h"


using namespace ov_msckf;

Simulator* sim;
TrackArr* feature_tracker;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}

// Main function
int main(int argc, char** argv)
{
    // Define constant variables
    Eigen::Matrix3d R_CtoI;
    R_CtoI << 0,-1, 0,
              1, 0, 0,
              0, 0, 1;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 458.654, 0.0, 367.215,
                                                             0.0, 457.296, 248.375,
                                                             0.0, 0.0, 1.0);

    // Read in our parameters
    VioManagerOptions params;
    ros::init(argc, argv, "motion_from_camera");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);
    
    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);

    // Create our system
    sim = new Simulator(params);
    feature_tracker = new TrackArr(camera_matrix);

    // Time variables
    double t0;
    double dt_cam = 1 / params.sim_freq_cam;
    double dt_imu = 1 / params.sim_freq_imu;

    int count = -1;
    while(sim->ok() && ros::ok()) {
        // State variables
        double time_imu, time_cam;
        Eigen::Vector3d wm, am, wc, ac;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

        // IMU: get the next simulated IMU measurement if we have it
        bool hasimu = sim->get_next_imu(time_imu, wm, am);
        // CAM: get the next simulated camera uv measurements if we have them
        bool hascam = sim->get_next_cam(time_cam, camids, feats);

        if (hascam) {
            feature_tracker->push_back(time_cam, feats);
            count ++;
            
            if (count < 2) continue;   // jump to next loop until feature_tracker's three member variables are filled

            // get ground truth rotation matrix
            Eigen::Vector3d p_IinG_prev, w_IinI_prev, v_IinG_prev, v_IinG, p_IinG, w_IinI;
            Eigen::Matrix3d R_GtoI_prev, R_GtoI;
            bool success_vel_prev = sim->get_spline()->get_velocity(time_imu - dt_cam, R_GtoI_prev, p_IinG_prev, w_IinI_prev, v_IinG_prev);
            bool success_vel_curr = sim->get_spline()->get_velocity(time_imu, R_GtoI, p_IinG, w_IinI, v_IinG);

            if (count == 2) {  // initialize variables
                t0 = time_imu;
                continue;
            }
            
            Eigen::Matrix3d R_dC;     // rotation frame-to-frame
            feature_tracker->calc_motion(wc, ac, R_dC);
            Eigen::Vector3d wCinI = R_CtoI * wc;    // wc in G; thus, transform it in frame I

            double cos_sim = ov_calib::cosine_similarity(wc, w_IinI);
            double euc_dist = ov_calib::euclidian_distance(wc, w_IinI);
            double norm_ratio = ov_calib::norm_ratio(wc, w_IinI);
            printf("[Step %d] time_cam: %.3f, euc: %.3f, rat: %.3f, cos: %.3f  (|w_gt|: %.3f)\n", 
            count - 3, time_cam - t0, cos_sim, euc_dist, norm_ratio, w_IinI.norm());
        }
    }

    // Finally delete our system
    delete sim;
    delete feature_tracker;

    // Done!
    return EXIT_SUCCESS;

}
