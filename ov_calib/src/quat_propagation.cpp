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
#include "utils/quat_ops.h"

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
int main(int argc, char** argv) {
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
    ros::init(argc, argv, "quat_propagation");
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
    double dist_r = 0.0;

    int count = -1;
    while(sim->ok() && ros::ok()) {
        // State variables
        double t;
        Eigen::Vector3d wm, am;
        Eigen::Matrix3d R_GtoIh;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

        bool hasimu = sim->get_next_imu(t, wm, am);
        bool hascam = sim->get_next_cam(t, camids, feats);

        if (hascam) {
            Eigen::Vector3d p_IinG, w_IinI, v_IinG, p_IinG_next, w_IinI_next, v_IinG_next;
            Eigen::Matrix3d R_GtoI, R_GtoI_next;
            Eigen::Quaterniond q_curr, q_next, q_pred;
            
            // Read GT angular velocity (w_IinI, w_IinI_next) and rotation (R_GtoI, R_GtoI_next) for current step and next step, respectively
            bool success_vel_curr = sim->get_spline()->get_velocity(t, R_GtoI, p_IinG, w_IinI, v_IinG);
            bool success_vel_next = sim->get_spline()->get_velocity(t + dt_imu, R_GtoI_next, p_IinG_next, w_IinI_next, v_IinG_next);
            
            // Calculate distance traveled so far
            Eigen::Quaterniond dq_gt(R_GtoI_next * R_GtoI.transpose());
            dist_r += dq_gt.angularDistance(Eigen::Quaterniond::Identity());

            // Convert R_GtoI_next to be Eigen::Quaterniond type
            q_next = R_GtoI_next;

            // Initialize variables
            if (count == -1) {
                t0 = t;
                q_pred = R_GtoI;
                q_curr = R_GtoI;
            }
            
            // Propagate from q_curr to q_pred w.r.t. w_IinI 
            Eigen::Vector4d old_quat = q_curr.coeffs();
            Eigen::Vector4d new_quat;
            // ov_calib::predict_mean_discrete(old_quat, dt_imu, w_IinI, w_IinI_next, new_quat, true);
            ov_calib::predict_mean_rk4(old_quat, dt_imu, w_IinI, w_IinI_next, new_quat);
            q_pred.x() = new_quat(0); q_pred.y() = new_quat(1); q_pred.z() = new_quat(2); q_pred.w() = new_quat(3);
            
            // Compare predicted rotation and GT rotation for every second
            if (count % (int)params.sim_freq_imu == 0) {
                std::cout.precision(3);
                std::cout << "[time : " << t - t0 << ", dist : " << dist_r;
                std::cout << "] q_pred : " << q_pred.coeffs().transpose() << ", q_next : " << q_next.coeffs().transpose();
                std::cout << " (diff : " << q_pred.angularDistance(q_next) << ")" <<  std::endl;
            }

            // Move to next step
            q_curr = q_pred;
            count ++;
        }
        
    }
    
    // Finally delete our system
    delete sim;

    // Done!
    return EXIT_SUCCESS;

}