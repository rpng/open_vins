#include <iomanip>   // for setiosflags
#include <csignal>
#include <cmath>
#include <ros/ros.h>

// ov_core
#include "track/TrackArr.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"
#include "utils/parse_ros.h"

// ov_msckf
#include "core/RosVisualizer.h"
#include "sim/Simulator.h"


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

    // Initialize internal variables
    // Rotations
    Eigen::Matrix3d R_GtoI, R_GtoIh, R_GtoCh;
    Eigen::Matrix3d R_dI, R_dC;     // rotation frame-to-frame
    Eigen::Matrix3d R_GtoIprev;
    Eigen::Vector3d p_IinG, w_IinI, v_IinG;
    // Time variables
    double t0;
    double dt = 0.1;

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
            
            if (count < 3) continue;   // break until feature_tracker's three member variables are filled

            // get ground truth rotation matrix
            bool success_vel_prev = sim->get_spline()->get_velocity(time_imu - dt, R_GtoIprev, p_IinG, w_IinI, v_IinG);
            bool success_vel_curr = sim->get_spline()->get_velocity(time_imu, R_GtoI, p_IinG, w_IinI, v_IinG);
            R_dI = R_GtoIprev.transpose() * R_GtoI;

            if (count == 3) {  // initialize variables
                t0 = time_imu;
                R_GtoIh = R_GtoIprev;
                R_GtoCh = R_GtoIprev * R_CtoI.transpose();
            }
            
            // get camera rotation matrix
            feature_tracker->calc_motion(wc, ac, R_dC);
            // Estimate
            R_GtoIh = R_GtoIh * R_dI;
            R_GtoCh = R_GtoCh * R_dC;
            
            Eigen::Quaterniond q_GtoI(R_GtoI);   // Ground-truth IMU rotation
            Eigen::Quaterniond q_GtoC(R_GtoI * R_CtoI.transpose());     // Ground-truth camera rotation
            Eigen::Quaterniond q_GtoIh(R_GtoIh);     // estimated IMU rotation
            Eigen::Quaterniond q_GtoCh(R_GtoCh);     // estimated camera rotation
            Eigen::Quaterniond q_dI(R_dI);           // rotational increment of IMU (GT)
            Eigen::Quaterniond q_dC(R_dC);           // rotational increment of camera
            double diff_I = q_GtoIh.angularDistance(q_GtoI);   // should be the same
            double diff_C = q_GtoCh.angularDistance(q_GtoC);   // should be the same
            double diff_delQ = q_dC.angularDistance(q_dI);        // should be the same
            
            std::cout.precision(4);
            std::cout << "[step " << count - 3 << "] time_imu: " << time_imu - t0 << ", time_cam: " << time_cam - t0;
            std::cout << " (diff : " << time_cam - time_imu << ")" << std::endl;
            std::cout << " -- q_GtoI : " << q_GtoI.coeffs().transpose() << ",   ";
            std::cout << "q_GtoIh: " << q_GtoIh.coeffs().transpose() << " (Distance: " << diff_I << ")" << std::endl;
            std::cout << " -- q_GtoC : " << q_GtoC.coeffs().transpose() << ",   ";
            std::cout << "q_GtoCh: " << q_GtoCh.coeffs().transpose() << " (Distance: " << diff_C << ")" << std::endl;
            std::cout << " -- q_dI   : " << q_dI.coeffs().transpose() << ",   ";
            std::cout << "q_dC   : " << q_dC.coeffs().transpose() << " (Difference: " << diff_delQ << ")" << std::endl;
            
        }
    }

    // Finally delete our system
    delete sim;
    delete feature_tracker;

    // Done!
    return EXIT_SUCCESS;

}














