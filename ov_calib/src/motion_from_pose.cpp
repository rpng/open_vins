#include <iomanip>   // for setiosflags
#include <csignal>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>

// ov_core
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
    ros::init(argc, argv, "motion_from_pose");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);
    
    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);

    // Create our system
    sim = new Simulator(params);

    // Time variables
    double t0 = 0;
    double dt_cam = 1 / params.sim_freq_cam;
    double dt_imu = 1 / params.sim_freq_imu;

    std::fstream f;
    f.open("/home/jlee/data_new.csv");

    int count = -1;
    while(sim->ok() && ros::ok()) {
        // State variables
        double t;
        Eigen::Vector3d wm, am;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

        bool hasimu = sim->get_next_imu(t, wm, am);
        bool hascam = sim->get_next_cam(t, camids, feats);

        if (hascam) {
            if (t0 == 0) t0 = t;
            Eigen::Vector3d p_IinG_prev, v_IinG_prev, w_IinI_prev, p_IinG, v_IinG, w_IinI, p_IinG_next, v_IinG_next, w_IinI_next;
            Eigen::Vector3d alpha_IinI_prev, a_IinG_prev, alpha_IinI, a_IinG, alpha_IinI_next, a_IinG_next;
            Eigen::Matrix3d dR_prev, dR_next, R_GtoI_prev, R_GtoI, R_GtoI_next;
            
            // Calculate relative poses between successive frames
            bool success_vel_prev = sim->get_spline()->get_acceleration(t - dt_imu, R_GtoI_prev, p_IinG_prev, w_IinI_prev, v_IinG_prev, alpha_IinI_prev, a_IinG_prev);
            bool success_vel_curr = sim->get_spline()->get_acceleration(t, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
            bool success_vel_next = sim->get_spline()->get_acceleration(t + dt_imu, R_GtoI_next, p_IinG_next, w_IinI_next, v_IinG_next, alpha_IinI_next, a_IinG_next);
            dR_prev = R_GtoI * R_GtoI_prev.transpose();
            dR_next = R_GtoI_next * R_GtoI.transpose();

            // Estimate angular motion from relative pose between frames
            Eigen::Vector3d what_IinI_prev, what_IinI_next, what_IinI, alphahat_IinI;
            ov_calib::dR_to_w(dR_prev, dt_imu, what_IinI_prev);
            ov_calib::dR_to_w(dR_next, dt_imu, what_IinI_next);
            what_IinI = (what_IinI_prev + what_IinI_next) / 2;
            alphahat_IinI = (what_IinI_next - dR_prev * what_IinI_prev) / dt_imu;

            // Show estimated angular velocity
            double euc_dist = ov_calib::euclidian_distance(what_IinI, w_IinI);
            double norm_ratio = ov_calib::norm_ratio(what_IinI, w_IinI);
            double cos_sim = ov_calib::cosine_similarity(what_IinI, w_IinI);

            printf("[time:  %.3f] euc: %.3f, rat: %.3f, cos: %.3f  (|w_gt|: %.3f)\n", t - t0, euc_dist, norm_ratio, cos_sim, w_IinI.norm());

            // Show estimated angular acceleration
            euc_dist = ov_calib::euclidian_distance(alphahat_IinI, alpha_IinI);
            norm_ratio = ov_calib::norm_ratio(alphahat_IinI, alpha_IinI);
            cos_sim = ov_calib::cosine_similarity(alphahat_IinI, alpha_IinI);

            printf("[time:  %.3f] euc: %.3f, rat: %.3f, cos: %.3f  (|a_gt|: %.3f)\n", t - t0, euc_dist, norm_ratio, cos_sim, alpha_IinI.norm());

        }
        
    }

    f.close();

    // Finally delete our system
    delete sim;

    // Done!
    return EXIT_SUCCESS;

}
