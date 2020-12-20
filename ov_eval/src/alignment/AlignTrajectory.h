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
#ifndef OV_EVAL_ALIGNTRAJECTORY_H
#define OV_EVAL_ALIGNTRAJECTORY_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>

#include "AlignUtils.h"
#include "utils/Colors.h"


namespace ov_eval {



    /**
     * @brief Class that given two trajectories, will align the two.
     *
     * Given two trajectories that have already been time synchronized we will compute the alignment transform between the two.
     * We can do this using different alignment methods such as full SE(3) transform, just postiion and yaw, or SIM(3).
     * These scripts are based on the [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) toolkit by Zhang and Scaramuzza.
     * Please take a look at their [2018 IROS](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) paper on these methods.
     */
    class AlignTrajectory {


    public:

        /**
         * @brief Align estimate to GT using a desired method using a set of initial poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param s scale from estimate to GT frame that will be computed
         * @param method Method used for alignment
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_trajectory(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                                     Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, std::string method, int n_aligned = -1);


    protected:

        /**
         * @brief Align estimate to GT using only position and yaw (for gravity aligned trajectories) using only the first poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         */
        static void align_posyaw_single(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                                        Eigen::Matrix3d &R, Eigen::Vector3d &t);

        /**
         * @brief Align estimate to GT using only position and yaw (for gravity aligned trajectories) using a set of initial poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_posyaw(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                                 Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned = -1);




        /**
         * @brief Align estimate to GT using a full SE(3) transform using only the first poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         */
        static void align_se3_single(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                                     Eigen::Matrix3d &R, Eigen::Vector3d &t);

        /**
         * @brief Align estimate to GT using a full SE(3) transform using a set of initial poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_se3(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                              Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned = -1);


        /**
         * @brief Align estimate to GT using a full SIM(3) transform using a set of initial poses
         * @param traj_es Estimated trajectory values in estimate frame [pos,quat]
         * @param traj_gt Groundtruth trjaectory in groundtruth frame [pos,quat]
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param s scale from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_sim3(const std::vector<Eigen::Matrix<double,7,1>> &traj_es, const std::vector<Eigen::Matrix<double,7,1>> &traj_gt,
                               Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, int n_aligned = -1);



    };



}


#endif /* OV_EVAL_ALIGNTRAJECTORY_H */


