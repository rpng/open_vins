#ifndef OV_MSCKF_STATE_PROPAGATOR_H
#define OV_MSCKF_STATE_PROPAGATOR_H


#include "state/StateHelper.h"
#include "utils/quat_ops.h"

using namespace ov_core;



/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Performs the state covariance and mean propagation using imu measurements
     */
    class Propagator {

    public:

        /**
         * @brief Struct for a single imu measurement (time, wm, am)
         */
        struct IMUDATA {

            /// Timestamp of the reading
            double timestamp;

            /// Gyroscope reading, angular velocity (rad/s)
            Eigen::Matrix<double, 3, 1> wm;

            /// Accelerometer reading, linear acceleration (m/s^2)
            Eigen::Matrix<double, 3, 1> am;

        };


        /**
         * @brief Struct of our imu noise parameters
         */
        struct NoiseManager {

            /// Gyro white noise
            double sigma_w;

            /// Gyro white noise covariance
            double sigma_w_2;

            /// Gyro random walk
            double sigma_wb;

            /// Gyro random walk covariance
            double sigma_wb_2;

            /// Accel white noise
            double sigma_a;

            /// Accel white noise covariance
            double sigma_a_2;

            /// Accel random walk
            double sigma_ab;

            /// Accel random walk covariance
            double sigma_ab_2;

        };


        /**
         * @brief Default constructor
         * @param noises imu noise characteristics (continuous time)
         * @param gravity Global gravity of the system (normally [0,0,9.81])
         */
        Propagator(NoiseManager noises, Eigen::Matrix<double, 3, 1> gravity) : _noises(noises), _gravity(gravity) {
            _noises.sigma_w_2 = std::pow(_noises.sigma_w,2);
            _noises.sigma_a_2 = std::pow(_noises.sigma_a,2);
            _noises.sigma_wb_2 = std::pow(_noises.sigma_wb,2);
            _noises.sigma_ab_2 = std::pow(_noises.sigma_ab,2);
        }


        /**
         * @brief Stores incoming inertial readings
         *
         * @param timestamp Timestamp of imu reading
         * @param wm Gyro angular velocity reading
         * @param am Accelerometer linear acceleration reading
         */
        void feed_imu(double timestamp, Eigen::Matrix<double, 3, 1> wm, Eigen::Matrix<double, 3, 1> am) {

            // Create our imu data object
            IMUDATA data;
            data.timestamp = timestamp;
            data.wm = wm;
            data.am = am;

            // Append it to our vector
            imu_data.emplace_back(data);

        }


        /**
         * @brief Propagate state up to given timestamp and then clone
         *
         * This will first collect all imu readings that occured between the
         * *current* state time and the new time we want the state to be at.
         * If we don't have any imu readings we will try to extrapolate into the future.
         * After propagating the mean and covariance using our dynamics,
         * We clone the current imu pose as a new clone in our state.
         *
         * @param state Pointer to state
         * @param timestamp Time to propagate to and clone at
         */
        void propagate_and_clone(State *state, double timestamp);



    protected:


        /// Estimate for time offset at last propagation time
        double last_prop_time_offset = 0;


        /**
         * @brief Propagates the state forward using the imu data and computes the noise covariance and state-transition
         * matrix of this interval.
         *
         * This function can be replaced with analytical/numerical integration or when using a different state representation.
         * This contains our state transition matix along with how our noise evolves in time.
         * If you have other state variables besides the IMU that evolve you would add them here.
         *
         * @param state Pointer to state
         * @param data_minus imu readings at beginning of interval
         * @param data_plus imu readings at end of interval
         * @param F State-transition matrix over the interval
         * @param Qd Discrete-time noise covariance over the interval
         */
        void predict_and_compute(State *state, const IMUDATA data_minus, const IMUDATA data_plus,
                                 Eigen::Matrix<double, 15, 15> &F, Eigen::Matrix<double, 15, 15> &Qd);


        /// Container for the noise values
        NoiseManager _noises;

        /// Our history of IMU messages (time, angular, linear)
        std::vector<IMUDATA> imu_data;

        /// Gravity vector
        Eigen::Matrix<double, 3, 1> _gravity;


        /**
         * @brief Nice helper function that will linearly interpolate between two imu messages.
         *
         * This should be used instead of just "cutting" imu messages that bound the camera times
         * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
         *
         * @param imu_1 imu at beggining of interpolation interval
         * @param imu_2 imu at end of interpolation interval
         * @param timestamp Timestamp being interpolated to
         */
        IMUDATA interpolate_data(const IMUDATA imu_1, const IMUDATA imu_2, double timestamp) {
            // time-distance lambda
            double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
            //cout << "lambda - " << lambda << endl;
            // interpolate between the two times
            IMUDATA data;
            data.timestamp = timestamp;
            data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
            data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
            return data;
        }


    };


}

#endif //OV_MSCKF_STATE_PROPAGATOR_H
