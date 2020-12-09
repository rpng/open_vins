/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BsplineSE3FourPts.h"

using namespace ov_core;

// Members newly defined
void BsplineSE3FourPts::feed_rotations(const Eigen::Matrix3d &_R0, const Eigen::Matrix3d &_R1, 
                                       const Eigen::Matrix3d &_R2, const Eigen::Matrix3d &_R3) {
    // FIXME: need to be fixed
    pose0.setIdentity();
    pose1.setIdentity();
    pose2.setIdentity();
    pose3.setIdentity();

    pose0.block(0,0,3,3) = _R0.transpose();
    pose1.block(0,0,3,3) = _R1.transpose();
    pose2.block(0,0,3,3) = _R2.transpose();
    pose3.block(0,0,3,3) = _R3.transpose();
};

void BsplineSE3FourPts::feed_timestamps(const double &_t0, const double &_t1, const double &_t2, const double &_t3) {
    t0 = _t0;
    t1 = _t1;
    t2 = _t2;
    t3 = _t3;
};

// Members to be overrode
bool BsplineSE3FourPts::get_pose(double timestamp, Eigen::Matrix3d &R_GtoI) {

    bool success = (timestamp >= t1) && (timestamp <= t2);
    //printf("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if(!success) {
        printf("[SIM] get_pose not suceed...\n");
        R_GtoI.setIdentity();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2-t1);
    double u = (timestamp-t1)/DT;
    double b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    double b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    double b2 = 1.0/6.0*(u*u*u);

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*log_se3(Inv_se3(pose0)*pose1));
    Eigen::Matrix4d A1 = exp_se3(b1*log_se3(Inv_se3(pose1)*pose2));
    Eigen::Matrix4d A2 = exp_se3(b2*log_se3(Inv_se3(pose2)*pose3));

    // Finally get the interpolated pose
    Eigen::Matrix4d pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();
    return true;

};

bool BsplineSE3FourPts::get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &w_IinI) {
    
    bool success = (timestamp >= t1) && (timestamp <= t2);
    //printf("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);
    
    // Return failure if we can't get bounding poses
    if(!success) {
        printf("[SIM] get_velocity not suceed...\n");
        R_GtoI.setIdentity();
        w_IinI.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2-t1);
    double u = (timestamp-t1)/DT;
    double b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    double b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    double b2 = 1.0/6.0*(u*u*u);
    double b0dot = 1.0/(6.0*DT)*(3-6*u+3*u*u);
    double b1dot = 1.0/(6.0*DT)*(3+6*u-6*u*u);
    double b2dot = 1.0/(6.0*DT)*(3*u*u);

    // Cache some values we use alot
    Eigen::Matrix<double,6,1> omega_10 = log_se3(Inv_se3(pose0)*pose1);
    Eigen::Matrix<double,6,1> omega_21 = log_se3(Inv_se3(pose1)*pose2);
    Eigen::Matrix<double,6,1> omega_32 = log_se3(Inv_se3(pose2)*pose3);

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*omega_10);
    Eigen::Matrix4d A1 = exp_se3(b1*omega_21);
    Eigen::Matrix4d A2 = exp_se3(b2*omega_32);
    Eigen::Matrix4d A0dot = b0dot*hat_se3(omega_10)*A0;
    Eigen::Matrix4d A1dot = b1dot*hat_se3(omega_21)*A1;
    Eigen::Matrix4d A2dot = b2dot*hat_se3(omega_32)*A2;

    // Get the interpolated pose
    Eigen::Matrix4d pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();

    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
    Eigen::Matrix4d vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2dot);
    w_IinI = vee(pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3));
    return true;

};

bool BsplineSE3FourPts::get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &w_IinI, Eigen::Vector3d &alpha_IinI) {

    bool success = (timestamp >= t1) && (timestamp <= t2);
    //printf("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);
    
    // Return failure if we can't get bounding poses
    if(!success) {
        printf("[SIM] get_acceleration not suceed...\n");
        R_GtoI.setIdentity();
        w_IinI.setZero();
        alpha_IinI.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2-t1);
    double u = (timestamp-t1)/DT;
    double b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    double b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    double b2 = 1.0/6.0*(u*u*u);
    double b0dot = 1.0/(6.0*DT)*(3-6*u+3*u*u);
    double b1dot = 1.0/(6.0*DT)*(3+6*u-6*u*u);
    double b2dot = 1.0/(6.0*DT)*(3*u*u);
    double b0dotdot = 1.0/(6.0*DT*DT)*(-6+6*u);
    double b1dotdot = 1.0/(6.0*DT*DT)*(6-12*u);
    double b2dotdot = 1.0/(6.0*DT*DT)*(6*u);

    // Cache some values we use alot
    Eigen::Matrix<double,6,1> omega_10 = log_se3(Inv_se3(pose0)*pose1);
    Eigen::Matrix<double,6,1> omega_21 = log_se3(Inv_se3(pose1)*pose2);
    Eigen::Matrix<double,6,1> omega_32 = log_se3(Inv_se3(pose2)*pose3);

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*omega_10);
    Eigen::Matrix4d A1 = exp_se3(b1*omega_21);
    Eigen::Matrix4d A2 = exp_se3(b2*omega_32);
    Eigen::Matrix4d A0dot = b0dot*hat_se3(omega_10)*A0;
    Eigen::Matrix4d A1dot = b1dot*hat_se3(omega_21)*A1;
    Eigen::Matrix4d A2dot = b2dot*hat_se3(omega_32)*A2;
    Eigen::Matrix4d A0dotdot = b0dot*hat_se3(omega_10)*A0dot+b0dotdot*hat_se3(omega_10)*A0;
    Eigen::Matrix4d A1dotdot = b1dot*hat_se3(omega_21)*A1dot+b1dotdot*hat_se3(omega_21)*A1;
    Eigen::Matrix4d A2dotdot = b2dot*hat_se3(omega_32)*A2dot+b2dotdot*hat_se3(omega_32)*A2;

    // Get the interpolated pose
    Eigen::Matrix4d pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();

    // Get the interpolated velocities
    // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
    Eigen::Matrix4d vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2dot);
    w_IinI = vee(pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3));

    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega)
    // NOTE: Rdotdot = Rdot*skew(omega) + R*skew(alpha) => R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
    Eigen::Matrix4d acc_interp = pose0*(A0dotdot*A1*A2+A0*A1dotdot*A2+A0*A1*A2dotdot
                                        +2*A0dot*A1dot*A2+2*A0*A1dot*A2dot+2*A0dot*A1*A2dot);
    Eigen::Matrix3d omegaskew = pose_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3);
    alpha_IinI = vee(pose_interp.block(0,0,3,3).transpose()*(acc_interp.block(0,0,3,3)-vel_interp.block(0,0,3,3)*omegaskew));
    return true;

};
