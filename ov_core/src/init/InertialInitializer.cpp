#include "InertialInitializer.h"


using namespace ov_core;


void InertialInitializer::feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

    // Create our imu data object
    IMUDATA data;
    data.timestamp = timestamp;
    data.wm = wm;
    data.am = am;

    // Append it to our vector
    imu_data.emplace_back(data);

    // Delete all measurements older than three of our initialization windows
    auto it0 = imu_data.begin();
    while(it0 != imu_data.end() && it0->timestamp < timestamp-3*_window_length) {
        it0 = imu_data.erase(it0);
    }

}





bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double, 4, 1> &q_GtoI0, Eigen::Matrix<double, 3, 1> &b_w0,
                                              Eigen::Matrix<double, 3, 1> &v_I0inG, Eigen::Matrix<double, 3, 1> &b_a0, Eigen::Matrix<double, 3, 1> &p_I0inG) {

//    // Return if we don't have any measurements
//    if(imu_data.empty()) {
//        return false;
//    }
//
//    // Newest imu timestamp
//    double newesttime = imu_data.at(imu_data.size()-1).timestamp;
//
//    // First lets collect a window of IMU readings from the newest measurement to the oldest
//    std::vector<IMUDATA> window_newest, window_secondnew;
//    for(IMUDATA data : imu_data) {
//        if(data.timestamp > newesttime-1*_window_length && data.timestamp <= newesttime-0*_window_length) {
//            window_newest.push_back(data);
//        }
//        if(data.timestamp > newesttime-2*_window_length && data.timestamp <= newesttime-1*_window_length) {
//            window_secondnew.push_back(data);
//        }
//    }
//
//    // Return if both of these failed
//    if(window_newest.empty() || window_secondnew.empty()) {
//        //ROS_WARN("InertialInitializer::initialize_with_imu(): unable to select window of IMU readings, not enough readings");
//        return false;
//    }
//
//    // Calculate the sample variance for the newest one
//    Eigen::Matrix<double,3,1> a_avg = Eigen::Matrix<double,3,1>::Zero();
//    for(IMUDATA data : window_newest) {
//        a_avg += data.am;
//    }
//    a_avg /= (int)window_newest.size();
//    double a_var = 0;
//    for(IMUDATA data : window_newest) {
//        a_var += (data.am-a_avg).dot(data.am-a_avg);
//    }
//    a_var = std::sqrt(a_var/((int)window_newest.size()-1));
//
//    // If it is below the threshold just return
//    if(a_var < _imu_excite_threshold) {
//        ROS_WARN("InertialInitializer::initialize_with_imu(): no IMU excitation, below threshold %.4f < %.4f",a_var,_imu_excite_threshold);
//        return false;
//    }

    // Return if we don't have any measurements
    if(imu_data.size() < 200) {
        return false;
    }

    // Sum up our current accelerations and velocities
    Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
    for(size_t i=0; i<imu_data.size(); i++) {
        linsum += imu_data.at(i).am;
        angsum += imu_data.at(i).wm;
    }

    // Calculate the mean of the linear acceleration and angular velocity
    Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
    linavg = linsum/imu_data.size();
    angavg = angsum/imu_data.size();

    // Get z axis, which alines with -g (z_in_G=0,0,1)
    Eigen::Vector3d z_axis = linavg/linavg.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1,0,0);

    // Make x_axis perpendicular to z
    Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
    x_axis= x_axis/x_axis.norm();

    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

    // From these axes get rotation
    Eigen::Matrix<double,3,3> Ro;
    Ro.block(0,0,3,1) = x_axis;
    Ro.block(0,1,3,1) = y_axis;
    Ro.block(0,2,3,1) = z_axis;

    // Create our state variables
    Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro);

    // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
    Eigen::Matrix<double,3,1> bg = angavg;
    Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI)*_gravity;


    // Set our state variables
    time0 = imu_data.at(imu_data.size()-1).timestamp;
    q_GtoI0 = q_GtoI;
    b_w0 = bg;
    v_I0inG = Eigen::Matrix<double,3,1>::Zero();
    b_a0 = ba;
    p_I0inG = Eigen::Matrix<double,3,1>::Zero();

    // Done!!!
    return true;


}




