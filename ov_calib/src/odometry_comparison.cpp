#include <iomanip>   // for setiosflags
#include <csignal>
#include <ros/ros.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

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
    ros::init(argc, argv, "odometry_comparison");
    ros::NodeHandle nh("~");
    params = parse_ros_nodehandler(nh);
    
    // Step through the rosbag
    signal(SIGINT, signal_callback_handler);

    // Create our system
    sim = new Simulator(params);
    feature_tracker = new TrackArr(camera_matrix);

    // Initialize internal variables
    // Rotations
    Eigen::Matrix3d Rhat_GtoI, Rhat_GtoC;
    Eigen::Quaterniond qhat_GtoI;
    // Time variables
    double t0;
    double dt_cam = 1 / params.sim_freq_cam;
    double dt_imu = 1 / params.sim_freq_imu;
    // Distance traveled
    double dist_t = 0.0;
    double dist_r = 0.0;
    // dQ error
    double q_dI_sum = 0;
    double q_dC_sum = 0;

    int count = -1;
    while(sim->ok() && ros::ok()) {
        // State variables
        double time_imu, time_cam;
        Eigen::Vector3d wm, am, wc, ac;
        std::vector<Eigen::Vector3d> wm_vec;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;

        // IMU: get the next simulated IMU measurement if we have it
        bool hasimu = sim->get_next_imu(time_imu, wm, am); 
        if (hasimu) wm_vec.push_back(wm);
        // CAM: get the next simulated camera uv measurements if we have them
        bool hascam = sim->get_next_cam(time_cam, camids, feats);

        if (hascam) {
            feature_tracker->push_back(time_cam, feats);
            count ++;
            
            if (count < 2) continue;   // jump to next loop until feature_tracker's three member variables are filled

            // Initialize variables
            Eigen::Matrix3d R_GtoI_prev, R_GtoI;
            Eigen::Matrix3d R_dGT, R_dI, R_dC;
            Eigen::Vector3d p_IinG_prev, w_IinI_prev, v_IinG_prev, v_IinG, p_IinG, w_IinI;

            bool success_vel_prev = sim->get_spline()->get_velocity(time_imu - dt_cam, R_GtoI_prev, p_IinG_prev, w_IinI_prev, v_IinG_prev);
            bool success_vel_curr = sim->get_spline()->get_velocity(time_imu, R_GtoI, p_IinG, w_IinI, v_IinG);

            if (count == 2) {
                t0 = time_imu;
                Rhat_GtoI = R_GtoI; 
                qhat_GtoI = R_GtoI;
                Rhat_GtoC = R_CtoI.transpose() * R_GtoI;
                wm_vec.clear();
                continue;
            }
            
            // GT increment
            R_dGT = R_GtoI * R_GtoI_prev.transpose();

            // Do IMU odometry
            Eigen::Matrix3d Rhat_GtoI_prev = Rhat_GtoI;
            for (const auto& w : wm_vec) {
                // Propagate qhat_GtoI w.r.t. gyro measurements
                Eigen::Vector4d old_quat = qhat_GtoI.coeffs();
                Eigen::Vector4d new_quat;
                ov_calib::predict_mean_discrete(old_quat, dt_imu, w, w, new_quat, false);
                qhat_GtoI.x() = new_quat(0); qhat_GtoI.y() = new_quat(1); qhat_GtoI.z() = new_quat(2); qhat_GtoI.w() = new_quat(3);
                Rhat_GtoI = qhat_GtoI.matrix();     // Iteratively update Rhat_GtoI
            }
            R_dI = Rhat_GtoI * Rhat_GtoI_prev.transpose();      // Compute R_dI passed by wm_vec.size() number of IMU measurements
            wm_vec.clear();

            // Do camera odometry
            feature_tracker->calc_motion(wc, ac, R_dC);
            Rhat_GtoC = R_dC * Rhat_GtoC;

            Eigen::Quaterniond q_GtoI(R_GtoI);   // Ground-truth IMU rotation (GT)
            Eigen::Quaterniond q_GtoC(R_CtoI.transpose() * R_GtoI);     // Ground-truth camera rotation
            Eigen::Quaterniond qhat_GtoI(Rhat_GtoI);     // estimated IMU rotation
            Eigen::Quaterniond qhat_GtoC(Rhat_GtoC);     // estimated camera rotation
            double error_I = qhat_GtoI.angularDistance(q_GtoI);
            double error_C = qhat_GtoC.angularDistance(q_GtoC);
            
            Eigen::Vector3d dx = p_IinG - p_IinG_prev;
            Eigen::Quaterniond q_dGT(R_dGT);
            dist_t += dx.norm();
            dist_r += q_dGT.angularDistance(Eigen::Quaterniond::Identity());
            
            printf("[Step %d] time_imu: %.1f, time_cam: %.1f (dist = %.2f [m], %.2f [rad])\n", 
            count - 3, time_imu - t0, time_cam - t0, dist_t, dist_r);
            printf(" -- q_GtoI : %.3f %.3f %.3f %.3f,  qhat_GtoI : %.3f %.3f %.3f %.3f (dist = %.2f [rad])\n",
            q_GtoI.coeffs().w(), q_GtoI.x(), q_GtoI.y(), q_GtoI.z(), 
            qhat_GtoI.coeffs().w(), qhat_GtoI.x(), qhat_GtoI.y(), qhat_GtoI.z(), error_I);
            printf(" -- q_GtoC : %.3f %.3f %.3f %.3f,  qhat_GtoC : %.3f %.3f %.3f %.3f (dist = %.2f [rad])\n",
            q_GtoC.coeffs().w(), q_GtoC.x(), q_GtoC.y(), q_GtoC.z(), 
            qhat_GtoC.coeffs().w(), qhat_GtoC.x(), qhat_GtoC.y(), qhat_GtoC.z(), error_C);
            
            /*
            Eigen::Quaterniond q_dGT(R_dGT);           // rotational increment of IMU (GT)
            Eigen::Quaterniond q_dI(R_dI);           // rotational increment of IMU
            Eigen::Quaterniond q_dC(R_dC);           // rotational increment of camera
            double diff_qI = q_dI.angularDistance(q_dGT);
            double diff_qC = q_dC.angularDistance(q_dGT);

            printf(" -- q_dI (GT) : %.3f %.3f %.3f %.3f,  q_dI : %.3f %.3f %.3f %.3f (diff = %.2f [rad])\n",
            q_dGT.coeffs().w(), q_dGT.x(), q_dGT.y(), q_dGT.z(), 
            q_dI.coeffs().w(), q_dI.x(), q_dI.y(), q_dI.z(), diff_qI);
            printf(" -- q_dI (GT) : %.3f %.3f %.3f %.3f,  q_dC : %.3f %.3f %.3f %.3f (diff = %.2f [rad])\n",
            q_dGT.coeffs().w(), q_dGT.x(), q_dGT.y(), q_dGT.z(), 
            q_dC.coeffs().w(), q_dC.x(), q_dC.y(), q_dC.z(), diff_qC);
            
            double diff_qGT = q_dGT.angularDistance(Eigen::Quaterniond::Identity());
            */
        }
    }

    // Finally delete our system
    delete sim;
    delete feature_tracker;

    // Done!
    return EXIT_SUCCESS;

}














