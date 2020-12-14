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
#ifndef OV_MSCKF_SIMULATOR_MULTI_IMU_H
#define OV_MSCKF_SIMULATOR_MULTI_IMU_H


#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>


#include "core/VioManagerOptions.h"
#include "sim/BsplineSE3.h"
#include "utils/colors.h"

#include "Simulator.h"


using namespace ov_core;


namespace ov_msckf {

    class SimulatorMultiIMU : public Simulator {

    public:
        SimulatorMultiIMU(VioManagerOptions& params_) : Simulator(params_) {
            true_bias_accel.assign(params_.num_imus, Eigen::Vector3d::Zero());
            true_bias_gyro.assign(params_.num_imus, Eigen::Vector3d::Zero());
            hist_true_bias_time.resize(params_.num_imus);
            hist_true_bias_accel.resize(params_.num_imus);
            hist_true_bias_gyro.resize(params_.num_imus);
        };

        bool get_next_imu(double &time_imu, std::vector<int> &imuids, std::vector<Eigen::Vector3d> &wm, std::vector<Eigen::Vector3d> &am);

    protected:
        /// Our running acceleration bias
        std::vector<Eigen::Vector3d> true_bias_accel;
        /// Our running gyroscope bias
        std::vector<Eigen::Vector3d> true_bias_gyro;
        // Our history of true biases
        std::vector<std::vector<double>> hist_true_bias_time;
        std::vector<std::vector<Eigen::Vector3d>> hist_true_bias_accel;
        std::vector<std::vector<Eigen::Vector3d>> hist_true_bias_gyro;
    };


}

#endif //OV_MSCKF_SIMULATOR_MULTI_IMU_H
