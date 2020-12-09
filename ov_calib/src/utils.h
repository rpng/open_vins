#ifndef OV_CALIB_UTILS
#define OV_CALIB_UTILS

#include <cmath>
#include <fstream>
#include <eigen3/Eigen/Dense>

// ov_eval
#include "utils/Math.h"

namespace ov_calib {

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

void dR_to_w(const Eigen::Matrix3d& dR, const double dt, Eigen::Vector3d& w) {
    Eigen::Matrix3d v_skewed = dR.transpose() - dR;
    Eigen::Vector3d v; 
    v << -v_skewed(1, 2), v_skewed(0, 2), -v_skewed(0, 1);
    double w_norm = asin(v.norm() / 2) / dt;
    w = dR * v.normalized() * w_norm;
    
    return ;
}

Eigen::Matrix3d add_noise(const Eigen::Matrix3d& R_raw, double noise_angle) {
    Eigen::Quaterniond q_raw(R_raw);
    Eigen::Quaterniond q_noise;
    
    double eps = 0.001;
    // Calculate direction vector of q_raw
    Eigen::Vector3d direction;
    if (fabs(q_raw.x()) <= 0.001 && fabs(q_raw.x()) <= 0.001 && fabs(q_raw.x()) <= 0.001) {
        direction.setOnes();
    } else {
        direction(0) = q_raw.x();
        direction(1) = q_raw.y();
        direction(2) = q_raw.z();
    }
    direction.normalize();
    
    // Create q_noise
    q_noise.w() = cos(noise_angle/2);
    q_noise.x() = direction(0) * sin(noise_angle/2);
    q_noise.y() = direction(1) * sin(noise_angle/2);
    q_noise.z() = direction(2) * sin(noise_angle/2);
    
    // Calculate noised q_raw
    Eigen::Matrix3d R;
    Eigen::Matrix3d R_noise(q_noise);
    R = R_noise * R_raw;

    return R;
}

double euclidian_distance(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
    return (vec1 - vec2).norm();
}

double norm_ratio(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
    return vec1.norm() / vec2.norm();
}

double cosine_similarity(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
    return vec1.dot(vec2) / (vec1.norm() * vec2.norm());
}

}




#endif /* OV_CALIB_UTILS */