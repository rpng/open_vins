//
// Created by keck on 6/4/19.
//

#ifndef OPEN_VINS_PROPAGATOR_H
#define OPEN_VINS_PROPAGATOR_H

#include "state/StateHelper.h"
#include "utils/quat_ops.h"


/**@brief Structure containing the IMU data
 */
struct IMUDATA {
    /// timestamp of the reading
    double timestamp;

    //Gyroscope reading
    Eigen::Matrix<double,3,1> wm;

    //Accellerometer reading
    Eigen::Matrix<double,3,1> am;
};

/**@brief Structure containing the IMU noise parameters
 */
struct NoiseManager{

    /// Gyro white noise covariance
    double sigma_w_2;

    /// Gyro random walk covariance
    double sigma_wb_2;

    /// Accel white noise covariance
    double sigma_a_2;

    /// Accel random walk covariance
    double sigma_ab_2;

};

/**@brief Performs the covariance and mean propagation using IMU measurements
 */
class Propagator{

public:

    Propagator(NoiseManager noises_, Eigen::Matrix<double,3,1> gravity_) : _noises(noises_),
    _gravity(gravity_){}


    /**
     * Stores incoming readings
     * @param timestamp Timestamp of reading
     * @param wm Gyro reading
     * @param am Accelerometer reading
     */
    void feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

        // Create our imu data object
        IMUDATA data;
        data.timestamp = timestamp;
        data.wm = wm;
        data.am = am;

        // Append it to our vector
        imu_data.emplace_back(data);

    }


    /** @brief Propagate state up to given timestamp and then clone
     *  @param state Pointer to state
     *  @timestamp Pointer to progate to and clone at
     */
    bool propagateAndClone(State* state, double timestamp);

protected:
    
    /// Estimate for time offset at last propagation time
    double last_prop_time_offset = 0;

    /**
     * @brief Propagates the state forward using the IMU data and computes the noise covariance and state-transition
     * matrix of this interval. This function can be replaced with analytical/numerical integration or when using a
     * different state representation
     * @param imu Pointer to imu
     * @param data_minus IMU readings at beginning of interval
     * @param data_plus IMU readings at end of interval
     * @param F State-transition matrix over the interval
     * @param Qd Discrete-time noise covariance over the interval
     */
    void predictAndcomputeJacobians(State *state, const IMUDATA data_minus, const IMUDATA data_plus,
            Eigen::Matrix<double,15,15> &F, Eigen::Matrix<double,15,15> &Qd);


    void propagateSingleInterval(State *state, const IMUDATA &data_minus, const IMUDATA &data_plus,
            Eigen::Matrix<double,15,15> &Phi, Eigen::Matrix<double,15,15> &Qd){

        Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Zero();
        Eigen::Matrix<double,15,15> Qdi = Eigen::Matrix<double,15,15>::Zero();

        predictAndcomputeJacobians(state, data_minus, data_plus, F, Qdi);

        Phi = F*Phi;
        Qd = F*Qd*F.transpose()+Qdi;
    }

    /// Container for the noise values
    NoiseManager _noises;

    /// Our history of IMU messages (time, angular, linear)
    std::vector<IMUDATA> imu_data;

    /// Gravity vector

    Eigen::Matrix<double,3,1> _gravity;


    /**
     * @brief Nice helper function that will linearly interpolate between two imu messages
     * This should be used instead of just "cutting" imu messages that bound the camera times
     * Give better time offset if we use this function....
     * @param imu_1 IMU at beggining of interpolation interval
     * @param imu_2 IMU at end of interpolation interval
     * @param timestamp Timestamp being interpolated to
     */
    IMUDATA interpolate_data(const IMUDATA imu_1, const IMUDATA imu_2, double timestamp) {
        // time-distance lambda
        double lambda = (timestamp-imu_1.timestamp)/(imu_2.timestamp-imu_1.timestamp);
        //cout << "lambda - " << lambda << endl;
        // interpolate between the two times
        IMUDATA data;
        data.timestamp = timestamp;
        data.am = (1-lambda)*imu_1.am+lambda*imu_2.am;
        data.wm = (1-lambda)*imu_1.wm+lambda*imu_2.wm;
        return data;
    }




};




#endif //OPEN_VINS_PROPAGATOR_H
