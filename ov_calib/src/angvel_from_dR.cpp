#include <iomanip>   // for setiosflags
#include <csignal>
#include <cmath>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>

// ov_core
#include "track/TrackArr.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"
#include "utils/parse_ros.h"

// ov_eval
#include "utils/Math.h"

// ov_msckf
#include "core/RosVisualizer.h"
#include "sim/Simulator.h"


using namespace ov_msckf;

Simulator* sim;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}

void dR_to_w(const Eigen::Matrix3d& dR, const double dt, Vector3d& w);

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
    ros::init(argc, argv, "angvel_from_dR");
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
            Eigen::Vector3d p_IinG, w_IinI, v_IinG, p_InextinG, w_InextinInext, v_InextinG;
            Eigen::Matrix3d dR, R_GtoI, R_GtoInext;
            
            bool success_vel_curr = sim->get_spline()->get_velocity(t, R_GtoI, p_IinG, w_IinI, v_IinG);
            bool success_vel_next = sim->get_spline()->get_velocity(t + dt_imu, R_GtoInext, p_InextinG, w_InextinInext, v_InextinG);
            dR = R_GtoInext * R_GtoI.transpose();

            Eigen::Vector3d w_IinIh;
            dR_to_w(dR.transpose(), dt_imu, w_IinIh);

            std::cout << "time: " << t - t0 << ", |w0 - w_gt| : " << (w_IinIh - w_IinI).norm() << " (|w_gt| : " << w_IinI.norm() << ")" <<  std::endl;

            f << t-t0 << ", " << (w_IinIh - w_IinI).norm() << ", " << (w_IinI).norm() << std::endl;
        }
        
    }

    f.close();

    // Finally delete our system
    delete sim;

    // Done!
    return EXIT_SUCCESS;

}


void dR_to_w(const Eigen::Matrix3d& dR, const double dt, Vector3d& w) {
    Eigen::Matrix3d v_skewed = dR - dR.transpose();
    Eigen::Vector3d v; 
    v << -v_skewed(1, 2), v_skewed(0, 2), -v_skewed(0, 1);
    double w_norm = asin(v.norm() / 2) / dt;
    w = dR * v.normalized() * w_norm;
    
    return ;
}
