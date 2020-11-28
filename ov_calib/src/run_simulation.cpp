#include <iomanip>   // for setiosflags
#include <csignal>
#include <cmath>
#include <ros/ros.h>

#include <fstream>

// ov_core
#include "track/TrackArr.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"
#include "utils/parse_ros.h"

// ov_msckf
#include "core/RosVisualizer.h"
#include "sim/Simulator.h"
#include <eigen3/Eigen/Dense>


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
    ros::init(argc, argv, "run_calib");
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

    // File IO
    std::ofstream f;
    f.open("/home/jlee/data_new.csv");

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
            Eigen::Vector3d p_IprevinG, wGTPrev, v_IprevinG, v_IinG, p_IinG, wGT;
            Eigen::Matrix3d R_GtoIprev, R_GtoI;
            bool success_vel_prev = sim->get_spline()->get_velocity(time_imu - dt_cam, R_GtoIprev, p_IprevinG, wGTPrev, v_IprevinG);
            bool success_vel_curr = sim->get_spline()->get_velocity(time_imu, R_GtoI, p_IinG, wGT, v_IinG);

            if (count == 2) {  // initialize variables
                t0 = time_imu;
                continue;
            }
            
            Eigen::Matrix3d R_dC;     // rotation frame-to-frame
            feature_tracker->calc_motion(wc, ac, R_dC);

            printf("[Step %d] time_imu: %.1f, time_cam: %.1f\n", count - 3, time_imu - t0, time_cam - t0);
            
            Eigen::Vector3d wIinI = wm;
            Eigen::Vector3d wCinI = R_CtoI * wc;    // wc in G; thus, transform it in frame I

            // Similarity measurement
            // cosine similarity: consistency in directions (u dot v / |u||v|)
            // euclidian distance: how they are near from each other (sqrt(|u-v|^2))
            // ratio: how their norms are similar (|u|/|v|)

            double cos_similarity = wIinI.dot(wGT) / (wIinI.norm() * wGT.norm());
            double euc_distance = (wIinI - wGT).norm();
            double ratio = wIinI.norm() / wGT.norm();

            printf(" -- [wIinI] cos_sim: %.3f, euc_dist: %.3f, ratio: %.3f\n", cos_similarity, euc_distance, ratio);
            f << count - 3 << ", " << time_imu-t0 << ", ";
            f << cos_similarity << ", " << euc_distance << ", " << ratio << ", ";

            cos_similarity = wCinI.dot(wGT) / (wCinI.norm() * wGT.norm());
            euc_distance = (wCinI - wGT).norm();
            ratio = wCinI.norm() / wGT.norm();

            printf(" -- [wCinI] cos_sim: %.3f, euc_dist: %.3f, ratio: %.3f\n", cos_similarity, euc_distance, ratio);
            f << cos_similarity << ", " << euc_distance << ", " << ratio << std::endl;
            
        }
    }

    // Finally delete our system
    delete sim;
    delete feature_tracker;

    // Done!
    return EXIT_SUCCESS;

}
