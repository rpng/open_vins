#ifndef OV_EVAL_ALIGNTRAJECTORY_H
#define OV_EVAL_ALIGNTRAJECTORY_H


#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>


#include "AlignUtils.h"


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
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param s scale from estimate to GT frame that will be computed
         * @param method Method used for alignment
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_trajectory(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                     const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                     Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, std::string method, int n_aligned = -1);


    protected:

        /**
         * @brief Align estimate to GT using only position and yaw (for gravity aligned trajectories) using only the first poses
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         */
        static void align_posyaw_single(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                        const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                        Eigen::Matrix3d &R, Eigen::Vector3d &t);

        /**
         * @brief Align estimate to GT using only position and yaw (for gravity aligned trajectories) using a set of initial poses
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_posyaw(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                 const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                 Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned = -1);




        /**
         * @brief Align estimate to GT using a full SE(3) transform using only the first poses
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         */
        static void align_se3_single(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                     const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                     Eigen::Matrix3d &R, Eigen::Vector3d &t);

        /**
         * @brief Align estimate to GT using a full SE(3) transform using a set of initial poses
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_se3(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                              const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                              Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned = -1);


        /**
         * @brief Align estimate to GT using a full SIM(3) transform using a set of initial poses
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         * @param s scale from estimate to GT frame that will be computed
         * @param n_aligned Number of poses to use for alignment (-1 will use all)
         */
        static void align_sim3(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                               const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                               Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, int n_aligned = -1);



    };



}


#endif /* OV_EVAL_ALIGNTRAJECTORY_H */


