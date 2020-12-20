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
#ifndef OV_EVAL_TRAJECTORY_H
#define OV_EVAL_TRAJECTORY_H

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "alignment/AlignTrajectory.h"
#include "utils/Statistics.h"
#include "utils/Math.h"
#include "utils/Loader.h"
#include "utils/Colors.h"


namespace ov_eval {



    /**
     * @brief A single run for a given dataset.
     *
     * This class has all the error function which can be calculated for the loaded trajectory.
     * Given a groundtruth and trajectory we first align the two so that they are in the same frame.
     * From there the following errors could be computed:
     * - Absolute trajectory error
     * - Relative pose Error
     * - Normalized estimation error squared
     * - Error and bound at each timestep
     *
     * Please see the @ref evaluation page for details and Zhang and Scaramuzza [A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) paper for implementation specific details.
     */
    class ResultTrajectory {

    public:


        /**
         * @brief Default constructor that will load, intersect, and align our trajectories.
         * @param path_est Path to the estimate text file
         * @param path_gt Path to the groundtruth text file
         * @param alignment_method The alignment method to use [sim3, se3, posyaw, none]
         */
        ResultTrajectory(std::string path_est, std::string path_gt, std::string alignment_method);


        /**
         * @brief Computes the Absolute Trajectory Error (ATE) for this trajectory.
         *
         * This will first do our alignment of the two trajectories.
         * Then at each point the error will be calculated and normed as follows:
         * \f{align*}{
         * e_{ATE} &= \sqrt{ \frac{1}{K} \sum_{k=1}^{K} ||\mathbf{x}_{k,i} \boxminus \hat{\mathbf{x}}^+_{k,i}||^2_{2} }
         * \f}
         *
         * @param error_ori Error values for the orientation
         * @param error_pos Error values for the position
         */
        void calculate_ate(Statistics &error_ori, Statistics &error_pos);

        /**
         * @brief Computes the Absolute Trajectory Error (ATE) for this trajectory in the 2d x-y plane.
         *
         * This will first do our alignment of the two trajectories.
         * We just grab the yaw component of the orientation and the xy plane error.
         * Then at each point the error will be calculated and normed as follows:
         * \f{align*}{
         * e_{ATE} &= \sqrt{ \frac{1}{K} \sum_{k=1}^{K} ||\mathbf{x}_{k,i} \boxminus \hat{\mathbf{x}}^+_{k,i}||^2_{2} }
         * \f}
         *
         * @param error_ori Error values for the orientation (yaw error)
         * @param error_pos Error values for the position (xy error)
         */
        void calculate_ate_2d(Statistics &error_ori, Statistics &error_pos);

        /**
         * @brief Computes the Relative Pose Error (RPE) for this trajectory
         *
         * For the given set of segment lengths, this will split the trajectory into segments.
         * From there it will compute the relative pose error for all segments.
         * These are then returned as a map for each segment length.
         * \f{align*}{
         * \tilde{\mathbf{x}}_{r} &= \mathbf{x}_{k} \boxminus \mathbf{x}_{k+d_i} \\
         * e_{rpe,d_i} &= \frac{1}{D_i} \sum_{k=1}^{D_i} ||\tilde{\mathbf{x}}_{r} \boxminus \hat{\tilde{\mathbf{x}}}_{r}||^2_{2}
         * \f}
         *
         * @param segment_lengths What segment lengths we want to calculate for
         * @param error_rpe Map of segment lengths => errors for that length (orientation and position)
         */
        void calculate_rpe(const std::vector<double> &segment_lengths, std::map<double,std::pair<Statistics,Statistics>> &error_rpe);


        /**
         * @brief Computes the Normalized Estimation Error Squared (NEES) for this trajectory.
         *
         * If we have a covariance in addition to our pose estimate we can compute the NEES values.
         * At each timestep we compute this for both orientation and position.
         * \f{align*}{
         * e_{nees,k} &= \frac{1}{N} \sum_{i=1}^{N} (\mathbf{x}_{k,i} \boxminus \hat{\mathbf{x}}_{k,i})^\top \mathbf{P}^{-1}_{k,i} (\mathbf{x}_{k,i} \boxminus \hat{\mathbf{x}}_{k,i})
         * \f}
         *
         * @param nees_ori NEES values for the orientation
         * @param nees_pos NEES values for the position
         */
        void calculate_nees(Statistics &nees_ori, Statistics &nees_pos);


        /**
         * @brief Computes the error at each timestamp for this trajectory.
         *
         * As compared to ATE error (see @ref calculate_ate()) this will compute the error for each individual pose component.
         * This is normally used if you just want to look at a single run on a single dataset.
         * \f{align*}{
         * e_{rmse,k} &= \sqrt{ \frac{1}{N} \sum_{i=1}^{N} ||\mathbf{x}_{k,i} \boxminus \hat{\mathbf{x}}_{k,i}||^2_{2} }
         * \f}
         *
         * @param posx Position x-axis error and bound if we have it in our file
         * @param posy Position y-axis error and bound if we have it in our file
         * @param posz Position z-axis error and bound if we have it in our file
         * @param orix Orientation x-axis error and bound if we have it in our file
         * @param oriy Orientation y-axis error and bound if we have it in our file
         * @param oriz Orientation z-axis error and bound if we have it in our file
         * @param roll Orientation roll error and bound if we have it in our file
         * @param pitch Orientation pitch error and bound if we have it in our file
         * @param yaw Orientation yaw error and bound if we have it in our file
         */
        void calculate_error(Statistics &posx, Statistics &posy, Statistics &posz,
                             Statistics &orix, Statistics &oriy, Statistics &oriz,
                             Statistics &roll, Statistics &pitch, Statistics &yaw);


    protected:

        // Trajectory data (loaded from file and timestamp intersected)
        std::vector<double> est_times, gt_times;
        std::vector<Eigen::Matrix<double,7,1>> est_poses, gt_poses;
        std::vector<Eigen::Matrix3d> est_covori, est_covpos, gt_covori, gt_covpos;

        // Aligned trajectories
        std::vector<Eigen::Matrix<double,7,1>> est_poses_aignedtoGT;
        std::vector<Eigen::Matrix<double,7,1>> gt_poses_aignedtoEST;

        /**
         * @brief Gets the indices at the end of subtractories of a given length when starting at each index.
         * For each starting pose, find the end pose index which is the desired distance away.
         * @param distances Total distance travelled from start at each index
         * @param distance Distance of subtrajectory
         * @param max_dist_diff Maximum error between current trajectory length and the desired
         * @return End indices for each subtrajectory
         */
        std::vector<int> compute_comparison_indices_length(std::vector<double> &distances, double distance, double max_dist_diff) {

            // Vector of end ids for our pose indexes
            std::vector<int> comparisons;

            // Loop through each pose in our trajectory (i.e. our distance vector generated from the trajectory).
            for (size_t idx = 0; idx < distances.size(); idx++) {

                // Loop through and find the pose that minimized the difference between
                // The desired trajectory distance and our current trajectory distance
                double distance_startpose = distances.at(idx);
                int best_idx = -1;
                double best_error = max_dist_diff;
                for (size_t i = idx; i < distances.size(); i++) {
                    if (std::abs(distances.at(i) - (distance_startpose + distance)) < best_error) {
                        best_idx = i;
                        best_error = std::abs(distances.at(i) - (distance_startpose + distance));
                    }
                }

                // If we have an end id that reached this trajectory distance then add it!
                // Else this isn't a valid segment, thus we shouldn't add it (we will try again at the next pose)
                // NOTE: just because we searched through all poses and didn't find a close one doesn't mean we have ended
                // NOTE: this could happen if there is a gap in the groundtruth poses and we just couldn't find a pose with low error
                comparisons.push_back(best_idx);

            }

            // Finally return the ids for each starting pose that have this distance
            return comparisons;
        }



    };


}



#endif //OV_EVAL_TRAJECTORY_H
