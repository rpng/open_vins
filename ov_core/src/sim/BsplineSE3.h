#ifndef OV_CORE_BSPLINESE3_H
#define OV_CORE_BSPLINESE3_H


#include <Eigen/Eigen>

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

        /// Our control SE3 control poses
        //std::vector<CONTROLPOSE> control_points;


        /**
         * @brief SE(3) matrix exponential function
         *
         * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
         * \f{align*}{
         * \exp([\boldsymbol\omega,\mathbf u])&=\begin{bmatrix} \mathbf R & \mathbf V \mathbf u \\ \mathbf 0 & 1 \end{bmatrix} \\[1em]
         * \mathbf R &= \mathbf I + A \lfloor \boldsymbol\omega \times\rfloor + B \lfloor \boldsymbol\omega \times\rfloor^2 \\
         * \mathbf V &= \mathbf I + B \lfloor \boldsymbol\omega \times\rfloor + C \lfloor \boldsymbol\omega \times\rfloor^2
         * \f}
         * where we have the following definitions
         * \f{align*}{
         * \theta &= \sqrt{\boldsymbol\omega^\top\boldsymbol\omega} \\
         * A &= \sin\theta/\theta \\
         * B &= (1-\cos\theta)/\theta^2 \\
         * C &= (1-A)/\theta^2
         * \f}
         *
         * @param vec 6x1 in the se(3) space [omega, u]
         * @return 4x4 SE(3) matrix
         */
        Eigen::Matrix4d exp_se3(Eigen::Matrix<double,6,1> vec) {

            // Precompute our values
            Eigen::Vector3d w = vec.head<3>();
            Eigen::Vector3d u = vec.tail<3>();
            double theta = sqrt(w.dot(w));
            Eigen::Matrix3d wskew;
            wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

            // Handle small angle values
            double A, B, C;
            if(theta < 1e-8) {
                A = 1;
                B = 0.5;
                C = 1.0/6.0;
            } else {
                A = sin(theta)/theta;
                B = (1-cos(theta))/(theta*theta);
                C = (1-A)/(theta*theta);
            }

            // Matrices we need V and Identity
            Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d V = I_33 + B*wskew + C*wskew*wskew;

            // Get the final matrix to return
            Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
            mat.block(0,0,3,3) = I_33 + A*wskew + B*wskew*wskew;
            mat.block(0,3,3,1) = V*u;
            mat(3,3) = 1;
            return mat;

        }


        /**
         * @brief SE(3) matrix logarithm
         *
         * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
         * \f{align*}{
         * \boldsymbol\omega &=\mathrm{skew\_offdiags}\Big(\frac{\theta}{2\sin\theta}(\mathbf R - \mathbf R^\top)\Big) \\
         * \mathbf u &= \mathbf V^{-1}\mathbf t
         * \f}
         * where we have the following definitions
         * \f{align*}{
         * \theta &= \mathrm{arccos}((\mathrm{tr}(\mathbf R)-1)/2) \\
         * \mathbf V^{-1} &= \mathbf I - \frac{1}{2} \lfloor \boldsymbol\omega \times\rfloor + \frac{1}{\theta}\Big(1-\frac{A}{2B}\Big)\lfloor \boldsymbol\omega \times\rfloor^2
         * \f}
         *
         * @param mat 4x4 SE(3) matrix
         * @return 6x1 in the se(3) space [omega, u]
         */
        Eigen::Matrix<double,6,1> log_se3(Eigen::Matrix4d mat) {

            // Get sub-matrices
            Eigen::Matrix3d R = mat.block(0,0,3,3);
            Eigen::Vector3d t = mat.block(0,3,3,1);

            // Get theta
            double theta = acos(0.5*(R.trace()-1));

            // Handle small angle values
            double A, B, D;
            if(theta < 1e-8) {
                A = 1;
                B = 0.5;
                D = 0.5;
            } else {
                A = sin(theta)/theta;
                B = (1-cos(theta))/(theta*theta);
                D = theta/(2*sin(theta));
            }

            // Get the skew matrix and V inverse
            Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d wskew = D*(R-R.transpose());
            Eigen::Matrix3d Vinv = I_33 - 0.5*wskew+1/theta*(1-0.5*A/B)*wskew*wskew;

            // Calculate vector
            Eigen::Matrix<double,6,1> vec;
            vec.head<3>() << wskew(2, 1), wskew(0, 2), wskew(1, 0);
            vec.tail<3>() = Vinv*t;
            return vec;

        }



    };


}

#endif //OV_CORE_BSPLINESE3_H
