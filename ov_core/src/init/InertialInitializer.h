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
#ifndef OV_CORE_INERTIALINITIALIZER_H
#define OV_CORE_INERTIALINITIALIZER_H

#include <Eigen/Eigen>
#include "utils/quat_ops.h"
#include "utils/colors.h"

namespace ov_core {



    /**
     * @brief Initializer for visual-inertial system.
     *
     * This class has a series of functions that can be used to initialize your system.
     * Right now we have our implementation that assumes that the imu starts from standing still.
     * In the future we plan to add support for structure-from-motion dynamic initialization.
     *
     * To initialize from standstill:
     * 1. Collect all inertial measurements
     * 2. See if within the last window there was a jump in acceleration
     * 3. If the jump is past our threshold we should init (i.e. we have started moving)
     * 4. Use the *previous* window, which should have been stationary to initialize orientation
     * 5. Return a roll and pitch aligned with gravity and biases.
     *
     */
    class InertialInitializer {

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
         * @brief Default constructor
         * @param gravity Gravity in the global frame of reference
         * @param window_length Amount of time we will initialize over (seconds)
         * @param imu_excite_threshold Variance threshold on our acceleration to be classified as moving
         */
        InertialInitializer(Eigen::Matrix<double,3,1> gravity, double window_length, double imu_excite_threshold) :
                            _gravity(gravity), _window_length(window_length), _imu_excite_threshold(imu_excite_threshold) {}


        /**
         * @brief Stores incoming inertial readings
         *
         * @param timestamp Timestamp of imu reading
         * @param wm Gyro angular velocity reading
         * @param am Accelerometer linear acceleration reading
         */
        void feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am);


        /**
         * @brief Try to initialize the system using just the imu
         *
         * This will check if we have had a large enough jump in our acceleration.
         * If we have then we will use the period of time before this jump to initialize the state.
         * This assumes that our imu is sitting still and is not moving (so this would fail if we are experiencing constant acceleration).
         *
         * @param[out] time0 Timestamp that the returned state is at
         * @param[out] q_GtoI0 Orientation at initialization
         * @param[out] b_w0 Gyro bias at initialization
         * @param[out] v_I0inG Velocity at initialization
         * @param[out] b_a0 Acceleration bias at initialization
         * @param[out] p_I0inG Position at initialization
         * @return True if we have successfully initialized our system
         */
        bool initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                                 Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG);


    protected:

        /// Gravity vector
        Eigen::Matrix<double,3,1> _gravity;

        /// Amount of time we will initialize over (seconds)
        double _window_length;

        /// Variance threshold on our acceleration to be classified as moving
        double _imu_excite_threshold;

        /// Our history of IMU messages (time, angular, linear)
        std::vector<IMUDATA> imu_data;


    };


}

#endif //OV_CORE_INERTIALINITIALIZER_H
