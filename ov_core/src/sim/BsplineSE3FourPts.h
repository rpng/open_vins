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
#ifndef OV_CORE_BSPLINESE3FOURPTS_H
#define OV_CORE_BSPLINESE3FOURPTS_H


#include <Eigen/Eigen>

#include "utils/quat_ops.h"
#include "sim/BsplineSE3.h"

namespace ov_core {

    class BsplineSE3FourPts : public BsplineSE3 {

    public:
        BsplineSE3FourPts() {}

        // Members newly defined
        void feed_rotations(const Eigen::Matrix3d &_R0, const Eigen::Matrix3d &_R1, 
                            const Eigen::Matrix3d &_R2, const Eigen::Matrix3d &_R3);
        void feed_timestamps(const double &_t0, const double &_t1, const double &_t2, const double &_t3);
        // Members to be overrode
        bool get_pose(double timestamp, Eigen::Matrix3d &R_GtoI);
        bool get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &w_IinI);
        bool get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &w_IinI, Eigen::Vector3d &alpha_IinI);

    protected:
        Eigen::Matrix4d pose0, pose1, pose2, pose3;
        double t0, t1, t2, t3;

    private:
        // Members to be deprecated
        using BsplineSE3::feed_trajectory;
        using BsplineSE3::get_start_time;
        using BsplineSE3::find_bounding_poses;
        using BsplineSE3::find_bounding_control_points;
        using BsplineSE3::timestamp_start;
        using BsplineSE3::control_points;

    };

}

#endif //OV_CORE_BSPLINESE3FOURPTS_H
