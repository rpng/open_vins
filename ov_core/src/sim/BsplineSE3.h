#ifndef OV_CORE_BSPLINESE3_H
#define OV_CORE_BSPLINESE3_H


#include <Eigen/Eigen>

#include "utils/quat_ops.h"

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {



    /**
     * @brief B-Spline which performs interpolation over SE(3) manifold.
     *
     * This class implements the b-spline functionality that allows for interpolation over the \f$\mathbb{SE}(3)\f$ manifold.
     * This is based off of the derivations from [Continuous-Time Visual-Inertial Odometry for Event Cameras](https://ieeexplore.ieee.org/abstract/document/8432102/)
     * and [A Spline-Based Trajectory Representation for Sensor Fusion and Rolling Shutter Cameras](https://link.springer.com/article/10.1007/s11263-015-0811-3)
     * with some additional derivations being avalible in [these notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf).
     * The use of b-splines for \f$\mathbb{SE}(3)\f$ interpolation has the following properties:
     *
     * 1. Local control, allowing the system to function online as well as in batch
     * 2. \f$C^2\f$-continuity to enable inertial predictions and calculations
     * 3. Good approximation of minimal torque trajectories
     * 4. A parameterization of rigid-body motion devoid of singularities
     *
     * The key idea is to convert a set of trajectory points into a continuous-time *uniform cubic cumulative* b-spline.
     * As compared to standard b-spline representations, the cumulative form ensures local continuity which is needed for on-manifold interpolation.
     * We leverage the cubic b-spline to ensure \f$C^2\f$-continuity to ensure that we can calculate accelerations at any point along the trajectory.
     *
     */
    class BsplineSE3 {

    public:


        /**
         * @brief Default constructor
         */
        BsplineSE3() {}


        /**
         * @brief Will feed in a series of poses that we will then convert into control points.
         *
         * Our control points need to be uniformly spaced over the trajectory, thus given a vector we will
         * uniformly sample based on the average spacing between the pose points specified.
         *
         * @param traj_points Trajectory poses that we will convert into control points (timestamp(s), q_GtoI, p_IinG)
         */
        void feed_trajectory(std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_points);


        /**
         * @brief Gets the orientation and position at a given timestamp
         * @param timestamp Desired time to get the pose at
         * @param R_GtoI SO(3) orientation of the pose in the global frame
         * @param p_IinG Position of the pose in the global
         */
        void get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG);


        /**
         * @brief Gets the angular and linear velocity at a given timestamp
         * @param timestamp Desired time to get the pose at
         * @param w_IinG Angular velocity in the global frame
         * @param v_IinG Linear velocity in the global frame
         */
        void get_velocity(double timestamp, Eigen::Vector3d &w_IinG, Eigen::Vector3d &v_IinG);


        /**
         * @brief Gets the angular and linear acceleration at a given timestamp
         * @param timestamp Desired time to get the pose at
         * @param alpha_IinG Angular acceleration in the global frame
         * @param a_IinG Linear acceleration in the global frame
         */
        void get_acceleration(double timestamp, Eigen::Vector3d &alpha_IinG, Eigen::Vector3d &a_IinG);



    protected:

        /// Uniform sampling time for our control points
        double dt;

        /// Our control SE3 control poses (R_ItoG, p_IinG)
        std::map<double,Eigen::Matrix4d> control_points;



    };


}

#endif //OV_CORE_BSPLINESE3_H
