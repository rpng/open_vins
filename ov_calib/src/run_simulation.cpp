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
    Eigen::Matrix3d R_dA, R_dI, R_dC;     // rotation frame-to-frame
    Eigen::Matrix3d R_GtoIprev, R_GtoIcurr;
    Eigen::Vector3d p_IinG, p_IprevinG, p_IcurrinG, w_IinI, v_IinG;
    std::vector<Eigen::Vector3d> wm_vec;
    Eigen::Quaterniond q_GtoIh;
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
        if (hasimu) wm_vec.push_back(wm);
        // CAM: get the next simulated camera uv measurements if we have them
        bool hascam = sim->get_next_cam(time_cam, camids, feats);

        if (hascam) {
            feature_tracker->push_back(time_cam, feats);
            count ++;
            
            if (count < 3) continue;   // break until feature_tracker's three member variables are filled

            // get ground truth rotation matrix
            bool success_vel_prev = sim->get_spline()->get_velocity(time_imu - dt_cam, R_GtoIprev, p_IprevinG, w_IinI, v_IinG);
            bool success_vel_curr = sim->get_spline()->get_velocity(time_imu, R_GtoIcurr, p_IcurrinG, w_IinI, v_IinG);
            R_dA = R_GtoIprev.transpose() * R_GtoIcurr;    // GT increment

            if (count == 3) {  // initialize variables
                t0 = time_imu;
                R_GtoI = R_GtoIprev;
                R_GtoIh = R_GtoIprev; 
                q_GtoIh = R_GtoIh;
                R_GtoCh = R_GtoIprev * R_CtoI.transpose();
            }
            
            // Do GT odometry
            R_GtoI = R_GtoI * R_dA;
            
            // Do IMU odometry
            R_dI.setIdentity();
            for (auto& w : wm_vec) {
                Eigen::AngleAxisd rollAngle((w * dt_imu)(0), Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle((w * dt_imu)(1), Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle((w * dt_imu)(2), Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
                R_dI = R_dI * q.matrix();
            }
            R_GtoIh = R_GtoIh * R_dI;
            wm_vec.clear();
            
            // Do camera odometry
            feature_tracker->calc_motion(wc, ac, R_dC);
            R_GtoCh = R_GtoCh * R_dC; // FIXME: Which one is correct: either R_dC or R_dC.transpose()?
            
            Eigen::Quaterniond q_GtoI(R_GtoI);   // Ground-truth IMU rotation
            Eigen::Quaterniond q_GtoIcurr(R_GtoIcurr);   // Ground-truth IMU rotation (direct)
            Eigen::Quaterniond q_GtoC(R_GtoI * R_CtoI.transpose());     // Ground-truth camera rotation
            Eigen::Quaterniond q_GtoIh(R_GtoIh);     // estimated IMU rotation
            Eigen::Quaterniond q_GtoCh(R_GtoCh);     // estimated camera rotation
            Eigen::Quaterniond q_dA(R_dA);           // rotational increment of IMU (GT)
            Eigen::Quaterniond q_dI(R_dI);           // rotational increment of IMU
            Eigen::Quaterniond q_dC(R_dC);           // rotational increment of camera
            Eigen::Quaterniond q_ref; q_ref.x() = 0; q_ref.y() = 0; q_ref.z() = 0; q_ref.w() = 1; 
            double error_qIodom = q_GtoI.angularDistance(q_GtoIcurr);
            double error_qI = q_GtoIh.angularDistance(q_GtoI);
            double error_qC = q_GtoCh.angularDistance(q_GtoC);
            double error_dqI = q_dI.angularDistance(q_dA);
            double error_dqC = q_dC.angularDistance(q_dA);
            
            Eigen::Vector3d dx = p_IinG - p_IprevinG; dist_t += dx.norm();
            dist_r += q_dI.angularDistance(Eigen::Quaterniond::Identity());
            
            printf("[Step %d] time_imu: %.1f, time_cam: %.1f (dist = %.2f [m], %.2f [rad])\n", 
            count - 3, time_imu - t0, time_cam - t0, dist_t, dist_r);

            printf(" -- q_GtoI : %.3f %.3f %.3f %.3f,  q_GtoIh : %.3f %.3f %.3f %.3f (dist = %.2f [rad])\n",
            q_GtoI.coeffs().transpose()(0), q_GtoI.coeffs().transpose()(1), q_GtoI.coeffs().transpose()(2), q_GtoI.coeffs().transpose()(3), 
            q_GtoIh.coeffs().transpose()(0), q_GtoIh.coeffs().transpose()(1), q_GtoIh.coeffs().transpose()(2), q_GtoIh.coeffs().transpose()(3), error_qI);
            printf(" -- q_GtoC : %.3f %.3f %.3f %.3f,  q_GtoCh : %.3f %.3f %.3f %.3f (dist = %.2f [rad])\n",
            q_GtoC.coeffs().transpose()(0), q_GtoC.coeffs().transpose()(1), q_GtoC.coeffs().transpose()(2), q_GtoC.coeffs().transpose()(3), 
            q_GtoCh.coeffs().transpose()(0), q_GtoCh.coeffs().transpose()(1), q_GtoCh.coeffs().transpose()(2), q_GtoCh.coeffs().transpose()(3), error_qC);
            
            printf(" -- q_dI (GT) : %.3f %.3f %.3f %.3f,  q_dI : %.3f %.3f %.3f %.3f (diff = %.2f [rad])\n",
            q_dA.coeffs().transpose()(0), q_dA.coeffs().transpose()(1), q_dA.coeffs().transpose()(2), q_dA.coeffs().transpose()(3), 
            q_dI.coeffs().transpose()(0), q_dI.coeffs().transpose()(1), q_dI.coeffs().transpose()(2), q_dI.coeffs().transpose()(3), error_dqI);
            printf(" -- q_dI (GT) : %.3f %.3f %.3f %.3f,  q_dC : %.3f %.3f %.3f %.3f (diff = %.2f [rad])\n",
            q_dA.coeffs().transpose()(0), q_dA.coeffs().transpose()(1), q_dA.coeffs().transpose()(2), q_dA.coeffs().transpose()(3), 
            q_dC.coeffs().transpose()(0), q_dC.coeffs().transpose()(1), q_dC.coeffs().transpose()(2), q_dC.coeffs().transpose()(3), error_dqC);
            
            double q_dA_val = q_dA.angularDistance(q_ref);
            q_dI_sum += error_dqI / q_dA_val;
            q_dC_sum += error_dqC / q_dA_val;

            printf(" -- q_dI_avg : %.3f,  q_dC_avg : %.3f\n", q_dI_sum / (count - 2), q_dC_sum / (count - 2));

            // step, time, travel distance, error_qI, error_qC, error_dqI, error_dqC
            f << count - 3 << ", " << time_imu-t0 << ", " << dist_t << ", " << error_qI << ", " << error_qC << ", " << error_dqI << ", " << error_dqC << std::endl;
        }
    }
    f.close();

    // Finally delete our system
    delete sim;
    delete feature_tracker;

    // Done!
    return EXIT_SUCCESS;

}














