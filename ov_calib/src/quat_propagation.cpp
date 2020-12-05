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
#include "utils/quat_ops.h"

// ov_eval
#include "utils/Math.h"

// ov_msckf
#include "core/RosVisualizer.h"
#include "sim/Simulator.h"
#include "state/Propagator.h"


using namespace ov_msckf;

Simulator* sim;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}

void predict_mean_discrete(const Eigen::Vector4d &old_q, const double dt,
                           const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &w_hat2,
                           Eigen::Vector4d &new_q, bool use_avg) {
    
    // If we are averaging the IMU, then do so
    Eigen::Vector3d w_hat = w_hat1;
    if (use_avg == true) {
        w_hat = .5*(w_hat1+w_hat2);
    }

    // Pre-compute things
    double w_norm = w_hat.norm();
    Eigen::Matrix<double,4,4> I_4x4 = Eigen::Matrix<double,4,4>::Identity();

    // Orientation: Equation (101) and (103) and of Trawny indirect TR
    Eigen::Matrix<double,4,4> bigO;
    if(w_norm > 1e-20) {
        bigO = cos(0.5*w_norm*dt)*I_4x4 + 1/w_norm*sin(0.5*w_norm*dt)*ov_eval::Math::Omega(w_hat);
    } else {
        bigO = I_4x4 + 0.5*dt*Omega(w_hat);
    }
    new_q = ov_eval::Math::quatnorm(bigO * old_q);
    //new_q = rot_2_quat(exp_so3(-w_hat*dt)*R_Gtoi);

}

void predict_mean_rk4(const Eigen::Vector4d &old_q, const double dt,
                      const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &w_hat2,
                      Eigen::Vector4d &new_q) {

    // Pre-compute things
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d w_alpha = (w_hat2-w_hat1)/dt;

    // y0 ================
    Eigen::Vector4d q_0 = old_q;

    // k1 ================
    Eigen::Vector4d dq_0 = {0,0,0,1};
    Eigen::Vector4d q0_dot = 0.5*ov_eval::Math::Omega(w_hat)*dq_0;
    Eigen::Vector4d k1_q = q0_dot*dt;

    // k2 ================
    w_hat += 0.5*w_alpha*dt;

    Eigen::Vector4d dq_1 = ov_eval::Math::quatnorm(dq_0+0.5*k1_q);
    Eigen::Vector4d q1_dot = 0.5*ov_eval::Math::Omega(w_hat)*dq_1;
    Eigen::Vector4d k2_q = q1_dot*dt;

    // k3 ================
    Eigen::Vector4d dq_2 = ov_eval::Math::quatnorm(dq_0+0.5*k2_q);
    Eigen::Vector4d q2_dot = 0.5*ov_eval::Math::Omega(w_hat)*dq_2;
    Eigen::Vector4d k3_q = q2_dot*dt;

    // k4 ================
    w_hat += 0.5*w_alpha*dt;

    Eigen::Vector4d dq_3 = ov_eval::Math::quatnorm(dq_0+k3_q);
    Eigen::Vector4d q3_dot = 0.5*ov_eval::Math::Omega(w_hat)*dq_3;

    Eigen::Vector4d k4_q = q3_dot*dt;

    // y+dt ================
    Eigen::Vector4d dq = ov_eval::Math::quatnorm(dq_0+(1.0/6.0)*k1_q+(1.0/3.0)*k2_q+(1.0/3.0)*k3_q+(1.0/6.0)*k4_q);
    new_q = ov_core::quat_multiply(dq, q_0);

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
            Eigen::Vector3d p_IinG, w_IinI, v_IinG, p_InextinG, w_InextinInext, v_InextinG;
            Eigen::Matrix3d R_GtoI, R_GtoInext;
            Eigen::Quaterniond q_curr, q_next, q_pred;
            
            // Read GT angular velocity (w_IinI, w_InextinInext) and rotation (R_GtoI, R_GtoInext) for current step and next step, respectively
            bool success_vel_curr = sim->get_spline()->get_velocity(t, R_GtoI, p_IinG, w_IinI, v_IinG);
            bool success_vel_next = sim->get_spline()->get_velocity(t + dt_imu, R_GtoInext, p_InextinG, w_InextinInext, v_InextinG);
            
            // Calculate distance traveled so far
            Eigen::Quaterniond dq_gt(R_GtoInext * R_GtoI.transpose());
            dist_r += dq_gt.angularDistance(Eigen::Quaterniond::Identity());

            // Convert R_GtoInext to be Eigen::Quaterniond type
            q_next = R_GtoInext;

            // Initialize variables
            if (count == -1) {
                t0 = t;
                q_pred = R_GtoI;
                q_curr = R_GtoI;
            }
            
            // Propagate from q_curr to q_pred w.r.t. w_IinI 
            Eigen::Vector4d old_quat = q_curr.coeffs();
            Eigen::Vector4d new_quat;
            // predict_mean_discrete(old_quat, dt_imu, w_IinI, w_InextinInext, new_quat, true);
            predict_mean_rk4(old_quat, dt_imu, w_IinI, w_InextinInext, new_quat);
            q_pred.x() = new_quat(0); q_pred.y() = new_quat(1); q_pred.z() = new_quat(2); q_pred.w() = new_quat(3);
            
            // Compare predicted rotation and GT rotation for every second
            if (count % (int)params.sim_freq_imu == 0) {
                std::cout.precision(3);
                std::cout << "[time : " << t - t0 << ", dist : " << dist_r;
                std::cout << ", |q_pred| : " << q_pred.coeffs().transpose() << ", |q_next|  : " << q_next.coeffs().transpose();
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