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
     * TODO: add more details here...
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
        static void alignTrajectory(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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
        static void alignPositionYawSingle(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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
        static void alignPositionYaw(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                              std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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
        static void alignSE3Single(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                            std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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
        static void alignSE3(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                      std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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
        static void alignSIM3(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                       std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                       Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, int n_aligned = -1);



        /**
         * @brief Align estimate to GT body frames (assuming world frames already aligned) using SE(3)
         * @param p_es Estimated position values in estimate frame
         * @param p_gt GT Position values in GT frame
         * @param q_es Estimated JPL quaternion rotating from Estimate frame to IMU frame
         * @param q_gt GT JPL quaternion rotating from GT frame to IMU frame
         * @param R Rotation from estimate to GT frame that will be computed
         * @param t translation from estimate to GT frame that will be computed
         */
        static void alignBodySE3Single(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                Eigen::Matrix3d &R, Eigen::Vector3d &t);

    };



}


#endif /* OV_EVAL_ALIGNTRAJECTORY_H */


