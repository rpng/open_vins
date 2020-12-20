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
#ifndef OV_MSCKF_STATE_H
#define OV_MSCKF_STATE_H


#include <map>
#include <vector>
#include <unordered_map>

#include "types/Type.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "types/PoseJPL.h"
#include "types/Landmark.h"
#include "StateOptions.h"

using namespace ov_core;
using namespace ov_type;


namespace ov_msckf {

    /**
     * @brief State of our filter
     *
     * This state has all the current estimates for the filter.
     * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
     * We additionally have more parameters for online estimation of calibration and SLAM features.
     * We also have the covariance of the system, which should be managed using the StateHelper class.
     */
    class State {

    public:

        /**
         * @brief Default Constructor (will initialize variables to defaults)
         * @param options_ Options structure containing filter options
         */
        State(StateOptions &options_);

        ~State() {}


        /**
         * @brief Will return the timestep that we will marginalize next.
         * As of right now, since we are using a sliding window, this is the oldest clone.
         * But if you wanted to do a keyframe system, you could selectively marginalize clones.
         * @return timestep of clone we will marginalize
         */
        double margtimestep() {
            double time = INFINITY;
            for (const auto &clone_imu : _clones_IMU) {
                if (clone_imu.first < time) {
                    time = clone_imu.first;
                }
            }
            return time;
        }

        /**
         * @brief Calculates the current max size of the covariance
         * @return Size of the current covariance matrix
         */
        int max_covariance_size() {
            return (int)_Cov.rows();
        }


        /// Current timestamp (should be the last update time!)
        double _timestamp;

        /// Struct containing filter options
        StateOptions _options;

        /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
        std::shared_ptr<IMU> _imu;

        /// Map between imaging times and clone poses (q_GtoIi, p_IiinG)
        std::map<double, std::shared_ptr<PoseJPL>> _clones_IMU;

        /// Our current set of SLAM features (3d positions)
        std::unordered_map<size_t, std::shared_ptr<Landmark>> _features_SLAM;

        /// Time offset base IMU to camera (t_imu = t_cam + t_off)
        std::shared_ptr<Vec> _calib_dt_CAMtoIMU;

        /// Calibration poses for each camera (R_ItoC, p_IinC)
        std::unordered_map<size_t, std::shared_ptr<PoseJPL>> _calib_IMUtoCAM;

        /// Camera intrinsics
        std::unordered_map<size_t, std::shared_ptr<Vec>> _cam_intrinsics;

        /// What distortion model we are using (false=radtan, true=fisheye)
        std::unordered_map<size_t, bool> _cam_intrinsics_model;


    private:

        // Define that the state helper is a friend class of this class
        // This will allow it to access the below functions which should normally not be called
        // This prevents a developer from thinking that the "insert clone" will actually correctly add it to the covariance
        friend class StateHelper;

        /// Covariance of all active variables
        Eigen::MatrixXd _Cov;

        /// Vector of variables
        std::vector<std::shared_ptr<Type>> _variables;


    };

}

#endif //OV_MSCKF_STATE_H