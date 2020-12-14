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
#include "sim/BsplineSE3FourPts.h"

// ov_msckf
#include "sim/SimulatorMultiIMU.h"

// ov_calib
#include "utils.h"

#ifdef ROS_AVAILABLE
#include <ros/ros.h>
#include "core/RosTinyVisualizer.h"
#include "utils/parse_ros.h"
#endif

using namespace ov_msckf;

SimulatorMultiIMU* sim;
#ifdef ROS_AVAILABLE
RosTinyVisualizer* viz;
#endif

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
    ros::init(argc, argv, "run_multi_imu");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);
    
    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);

    // Create our system
    sim = new SimulatorMultiIMU(params);
    #ifdef ROS_AVAILABLE
    viz = new RosTinyVisualizer(nh, sim);
    #endif

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
        std::vector<Eigen::Vector3d> wm, am;
        std::vector<int> camids, imuids;
        std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

        bool hasimu = sim->get_next_imu(t, imuids, wm, am);
        bool hascam = sim->get_next_cam(t, camids, feats);

        if (hascam) {
            if (t0 == 0) t0 = t;
            Eigen::Vector3d p_IinG, v_IinG, w_IinI;
            Eigen::Vector3d alpha_IinI, a_IinG;
            Eigen::Matrix3d R_GtoI;
            
            bool success_vel_curr = sim->get_spline()->get_acceleration(t, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
            viz->visualize(t);
            
            for (int i=0; i<params.num_imus; i++) {
                printf("[time: %.2f - IMU %d] w: (%.2f, %.2f, %.2f),  a: (%.2f, %.2f, %.2f)\n", t-t0, i, 
                wm.at(i)(0), wm.at(i)(1), wm.at(i)(2), am.at(i)(0), am.at(i)(1), am.at(i)(2));
            }

            sleep(0.01);
        }
        
    }

    f.close();

    // Finally delete our system
    delete sim;
    delete viz;

    // Done!
    return EXIT_SUCCESS;

}
