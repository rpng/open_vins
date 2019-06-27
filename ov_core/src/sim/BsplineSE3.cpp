#include "BsplineSE3.h"


using namespace ov_core;





void BsplineSE3::feed_trajectory(std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_points) {


    // Find the average frequency to use as our uniform timesteps
    double sumdt = 0;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        sumdt += traj_points.at(i+1)(0)-traj_points.at(i)(0);
    }
    dt = sumdt/(traj_points.size()-1);
    dt = (dt < 0.05)? 0.05 : dt;
    ROS_INFO("[B-SPLINE]: control point dt = %.3f (original dt of %.3f)",dt,sumdt/(traj_points.size()-1));

    // convert all our trajectory points into SE(3) matrices
    // we are given [timestamp, p_IinG, q_GtoI]
    std::map<double,Eigen::Matrix4d> trajectory_points;
    for(size_t i=0; i<traj_points.size()-1; i++) {
        Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();
        T_IinG.block(0,0,3,3) = quat_2_Rot(traj_points.at(i).block(4,0,4,1)).transpose();
        T_IinG.block(0,3,3,1) = traj_points.at(i).block(1,0,3,1);
        trajectory_points.insert({traj_points.at(i)(0),T_IinG});
    }

    // Get the oldest timestamp
    double timestamp_min = INFINITY;
    double timestamp_max = -INFINITY;
    for(std::pair<const double, Eigen::Matrix4d> &pose : trajectory_points) {
        if(pose.first <= timestamp_min) {
            timestamp_min = pose.first;
        }
        if(pose.first >= timestamp_min) {
            timestamp_max = pose.first;
        }
    }
    ROS_INFO("[B-SPLINE]: trajectory start time = %.6f",timestamp_min);
    ROS_INFO("[B-SPLINE]: trajectory end time = %.6f",timestamp_max);


    // then create spline control points
    double timestamp_curr = timestamp_min;
    while(true) {

        // Get bounding posed for the current time
        double t0, t1;
        Eigen::Matrix4d pose0, pose1;
        bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0, pose0, t1, pose1);
        //ROS_INFO("[SIM]: time curr = %.6f | lambda = %.3f | dt = %.3f | dtmeas = %.3f",timestamp_curr,(timestamp_curr-t0)/(t1-t0),dt,(t1-t0));

        // If we didn't find a bounding pose, then that means we are at the end of the dataset
        // Thus break out of this loop since we have created our max number of control points
        if(!success)
            break;

        // Linear interpolation and append to our control points
        double lambda = (timestamp_curr-t0)/(t1-t0);
        Eigen::Matrix4d pose_interp = exp_se3(lambda*log_se3(pose1*pose0.inverse()))*pose0;
        control_points.insert({timestamp_curr, pose_interp});
        timestamp_curr += dt;
        //std::cout << pose_interp(0,3) << "," << pose_interp(1,3) << "," << pose_interp(2,3) << std::endl;

    }
    ROS_INFO("[B-SPLINE]: using %d control points in total",(int)control_points.size());

    // The start time of the system is three dt in since we need at least two older control points
    timestamp_start = timestamp_min + 3*dt;
    ROS_INFO("[B-SPLINE]: start trajectory time of %.6f",timestamp_start);

}





bool BsplineSE3::get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG) {

    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Eigen::Matrix4d pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
    //ROS_INFO("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if(!success) {
        R_GtoI.setIdentity();
        p_IinG.setZero();
        return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2-t1);
    double u = (timestamp-t1)/DT;
    double b0 = 1.0/6.0*(5+3*u-3*u*u+u*u*u);
    double b1 = 1.0/6.0*(1+3*u+3*u*u-2*u*u*u);
    double b2 = 1.0/6.0*(u*u*u);

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*log_se3(pose0.inverse()*pose1));
    Eigen::Matrix4d A1 = exp_se3(b1*log_se3(pose1.inverse()*pose2));
    Eigen::Matrix4d A2 = exp_se3(b2*log_se3(pose2.inverse()*pose3));

    // Finally get the interpolated pose
    Eigen::Matrix4d pose_interp = pose0*A0*A1*A2;
    R_GtoI = pose_interp.block(0,0,3,3).transpose();
    p_IinG = pose_interp.block(0,3,3,1);
    return true;

}




bool BsplineSE3::get_velocity(double timestamp, Eigen::Vector3d &w_IinG, Eigen::Vector3d &v_IinG) {

    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Eigen::Matrix4d pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
    //ROS_INFO("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success = %d",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if(!success) {
        w_IinG.setZero();
        v_IinG.setZero();
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

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*log_se3(pose0.inverse()*pose1));
    Eigen::Matrix4d A1 = exp_se3(b1*log_se3(pose1.inverse()*pose2));
    Eigen::Matrix4d A2 = exp_se3(b2*log_se3(pose2.inverse()*pose3));
    Eigen::Matrix4d A0dot = b0dot*hat_se3(log_se3(pose0.inverse()*pose1))*A0;
    Eigen::Matrix4d A1dot = b1dot*hat_se3(log_se3(pose1.inverse()*pose2))*A1;
    Eigen::Matrix4d A2dot = b2dot*hat_se3(log_se3(pose2.inverse()*pose3))*A2;

    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
    Eigen::Matrix4d pos_interp = pose0*A0*A1*A2;
    Eigen::Matrix4d vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2dot);
    w_IinG = vee(pos_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3));
    v_IinG = vel_interp.block(0,3,3,1);
    return true;

}




bool BsplineSE3::get_acceleration(double timestamp, Eigen::Vector3d &alpha_IinG, Eigen::Vector3d &a_IinG) {

    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Eigen::Matrix4d pose0, pose1, pose2, pose3;
    bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

    // Return failure if we can't get bounding poses
    if(!success) {
        alpha_IinG.setZero();
        a_IinG.setZero();
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

    // Calculate interpolated poses
    Eigen::Matrix4d A0 = exp_se3(b0*log_se3(pose0.inverse()*pose1));
    Eigen::Matrix4d A1 = exp_se3(b1*log_se3(pose1.inverse()*pose2));
    Eigen::Matrix4d A2 = exp_se3(b2*log_se3(pose2.inverse()*pose3));
    Eigen::Matrix4d A0dot = b0dot*hat_se3(log_se3(pose0.inverse()*pose1))*A0;
    Eigen::Matrix4d A1dot = b1dot*hat_se3(log_se3(pose1.inverse()*pose2))*A1;
    Eigen::Matrix4d A2dot = b2dot*hat_se3(log_se3(pose2.inverse()*pose3))*A2;
    Eigen::Matrix4d A0dotdot = b0dot*hat_se3(log_se3(pose0.inverse()*pose1))*A0dot+b0dotdot*hat_se3(log_se3(pose0.inverse()*pose1))*A0;
    Eigen::Matrix4d A1dotdot = b1dot*hat_se3(log_se3(pose1.inverse()*pose2))*A1dot+b1dotdot*hat_se3(log_se3(pose1.inverse()*pose2))*A1;
    Eigen::Matrix4d A2dotdot = b2dot*hat_se3(log_se3(pose2.inverse()*pose3))*A2dot+b2dotdot*hat_se3(log_se3(pose2.inverse()*pose3))*A2;


    // Finally get the interpolated velocities
    // NOTE: Rdot = R*skew(omega)
    // NOTE: Rdotdot = Rdot*skew(omega) + R*skew(alpha) => R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
    Eigen::Matrix4d pos_interp = pose0*A0*A1*A2;
    Eigen::Matrix4d vel_interp = pose0*(A0dot*A1*A2+A0*A1dot*A2+A0*A1*A2dot);
    Eigen::Matrix4d acc_interp = pose0*(A0dotdot*A1*A2+A0*A1dotdot*A2+A0*A1*A2dotdot
                                        +2*A0dot*A1dot*A2+2*A0*A1dot*A2dot+2*A0dot*A1*A2dot);
    Eigen::Matrix3d omegaskew = pos_interp.block(0,0,3,3).transpose()*vel_interp.block(0,0,3,3);
    alpha_IinG = vee(pos_interp.block(0,0,3,3).transpose()*(acc_interp.block(0,0,3,3)-vel_interp.block(0,0,3,3)*omegaskew));
    a_IinG = acc_interp.block(0,3,3,1);
    return true;

}


bool BsplineSE3::find_bounding_poses(double timestamp, std::map<double,Eigen::Matrix4d> &poses,
                                     double &t0, Eigen::Matrix4d &pose0, double &t1, Eigen::Matrix4d &pose1) {

    // Set the default values
    t0 = -1;
    t1 = -1;
    pose0 = Eigen::Matrix4d::Identity();
    pose1 = Eigen::Matrix4d::Identity();

    // Find the bounding poses
    double min_time = -INFINITY;
    double max_time = INFINITY;
    bool found_older = false;
    bool found_newer = false;

    // Find the bounding poses for interpolation. If no older one is found, measurement is unusable
    for(std::pair<const double, Eigen::Matrix4d> &pose : poses) {
        if (pose.first > min_time && pose.first <= timestamp){
            min_time = pose.first;
            found_older = true;
        }
        if (pose.first < max_time && pose.first > timestamp){
            max_time = pose.first;
            found_newer = true;
        }
    }

    // If we found the oldest one, set it
    if (found_older) {
        t0 = min_time;
        pose0 = poses.at(min_time);
    }

    // If we found the newest one, set it
    if(found_newer) {
        t1 = max_time;
        pose1 = poses.at(max_time);
    }

    // Assert the timestamps
    if(found_older && found_newer)
        assert(t0<t1);

    // Return true if we found both bounds
    return (found_older && found_newer);

}



bool BsplineSE3::find_bounding_control_points(double timestamp, std::map<double,Eigen::Matrix4d> &poses,
                                              double &t0, Eigen::Matrix4d &pose0, double &t1, Eigen::Matrix4d &pose1,
                                              double &t2, Eigen::Matrix4d &pose2, double &t3, Eigen::Matrix4d &pose3) {

    // Set the default values
    t0 = -1;
    t1 = -1;
    t2 = -1;
    t3 = -1;
    pose0 = Eigen::Matrix4d::Identity();
    pose1 = Eigen::Matrix4d::Identity();
    pose2 = Eigen::Matrix4d::Identity();
    pose3 = Eigen::Matrix4d::Identity();

    // Get the two bounding poses
    bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

    // Return false if this was a failure
    if(!success)
        return false;

    // Now find the poses that are below and above
    double min_time = -INFINITY;
    double max_time = INFINITY;
    bool found_min_of_min = false;
    bool found_max_of_max = false;
    for(std::pair<const double, Eigen::Matrix4d> &pose : poses) {
        if (pose.first > min_time && pose.first < t1){
            min_time = pose.first;
            found_min_of_min = true;
        }
        if (pose.first < max_time && pose.first > t2){
            max_time = pose.first;
            found_max_of_max = true;
        }
    }

    // If we found the oldest one, set it
    if (found_min_of_min) {
        t0 = min_time;
        pose0 = poses.at(min_time);
    }

    // If we found the newest one, set it
    if(found_max_of_max) {
        t3 = max_time;
        pose3 = poses.at(max_time);
    }

    // Assert the timestamps
    if(success && found_min_of_min && found_max_of_max) {
        assert(t0<t1);
        assert(t1<t2);
        assert(t2<t3);
    }

    // Return true if we found all four bounding poses
    return (success && found_min_of_min && found_max_of_max);

}


