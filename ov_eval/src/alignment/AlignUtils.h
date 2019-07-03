#ifndef OV_EVAL_ALIGNUTILS_H
#define OV_EVAL_ALIGNUTILS_H


#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>


namespace ov_eval {


    /**
     * @brief Helper functions for the trajectory alignment class.
     *
     * The key function is an implementation of Umeyama's [Least-squares estimation of transformation parameters between two point patterns](https://ieeexplore.ieee.org/document/88573).
     * This is what allows us to find the transform between the two trajectories without worrying about singularities for the absolute trajectory error.
     *
     */
    class AlignUtils {

    public:

        /**
         * @brief Gets roll, pitch, yaw of argument rotation (in that order)
         * @param rot Rotation matrix
         */
        static inline Eigen::Matrix<double, 3, 1> rot2rpy(const Eigen::Matrix<double, 3, 3> &rot) {
            Eigen::Matrix<double, 3, 1> rpy;
            rpy(1, 0) = atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
            if (abs(cos(rpy(1, 0)) > 1.0e-12)) {
                rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
                rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
            } else {
                rpy(2, 0) = 0;
                rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
            }
            return rpy;
        }

        /**
         * @brief Construct rotation matrix from given yaw
         * @param t yaw angle
         */
        static inline Eigen::Matrix<double, 3, 3> rot_z(double t) {
            Eigen::Matrix<double, 3, 3> r;
            double ct = cos(t);
            double st = sin(t);
            r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
            return r;
        }

        /**
         * @brief Skew-symmetric matrix from a given 3x1 vector
         *
         * This is based on equation 6 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf):
         * \f{align*}{
         *  \lfloor\mathbf{v}\times\rfloor =
         *  \begin{bmatrix}
         *  0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0
         *  \end{bmatrix}
         * @f}
         *
         * @param[in] w 3x1 vector to be made a skew-symmetric
         * @return 3x3 skew-symmetric matrix
         */
        static inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
            Eigen::Matrix<double, 3, 3> w_x;
            w_x << 0, -w(2), w(1),
                    w(2), 0, -w(0),
                    -w(1), w(0), 0;
            return w_x;
        }

        /**
         * @brief Converts JPL quaterion to SO(3) rotation matrix
         *
         * This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf):
         * \f{align*}{
         *  \mathbf{R} = (2q_4^2-1)\mathbf{I}_3-2q_4\lfloor\mathbf{q}\times\rfloor+2\mathbf{q}^\top\mathbf{q}
         * @f}
         *
         * @param[in] q JPL quaternion
         * @return 3x3 SO(3) rotation matrix
         */
        static inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
            Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
            Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3)
                                  - 2 * q(3, 0) * q_x +
                                  2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
            return Rot;
        }

        /**
         * @brief Gets best yaw from Frobenius problem.
         * Equation (17-(18) in [Zhang et al. 2018 IROS](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) paper.
         * @param C Data matrix
         */
        static inline double get_best_yaw(const Eigen::Matrix<double, 3, 3> &C) {
            double A = C(0, 1) - C(1, 0);
            double B = C(0, 0) + C(1, 1);
            //return M_PI_2 - atan2(B, A);
            return atan2(A, B);
        }

        /**
         * @brief Gets mean of the vector of data
         * @param data Vector of data
         * @return Mean value
         */
        static inline Eigen::Matrix<double, 3, 1> get_mean(const std::vector<Eigen::Matrix<double, 3, 1>> &data) {
            Eigen::Matrix<double, 3, 1> mean = Eigen::Matrix<double, 3, 1>::Zero();
            for (size_t i = 0; i < data.size(); i++) {
                mean.noalias() += data[i];
            }
            mean /= data.size();
            return mean;
        }

        /**
         * @brief Given a set of points in a model frame and a set of points in a data frame,
         * finds best transform between frames (subject to constraints).
         *
         * @param model Vector of points in model frame (i.e., gt)
         * @param data Vector of points in data frame (i.e., estimates)
         * @param R Output rotation from data frame to model frame
         * @param t Output translation from data frame to model frame
         * @param s Output scale from data frame to model frame
         * @param known_scale Whether to fix scale
         * @param yaw_only Whether to only use yaw orientation (such as when frames are already gravity aligned)
         */
        static void align_umeyama(const std::vector<Eigen::Matrix<double, 3, 1>> &model,
                           const std::vector<Eigen::Matrix<double, 3, 1>> &data,
                           Eigen::Matrix<double, 3, 3> &R, Eigen::Matrix<double, 3, 1> &t,
                           double &s, bool known_scale, bool yaw_only);




    };


}

#endif //OV_EVAL_ALIGNUTILS_H
