#ifndef OV_CORE_QUAT_OPS_H
#define OV_CORE_QUAT_OPS_H

/**
 * @file quat_ops.h
 * @brief JPL quaternion math utilities
 *
 * @section Summary
 * This file contains the common utility functions for operating on JPL quaternions.
 * We need to take special care to handle edge cases when converting to and from other rotation formats.
 * All equations are based on the following tech report:
 * > Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman filter for 3D attitude estimation."
 * > University of Minnesota, Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 * > http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 *
 * @section JPL Quaternion Definition
 *
 * We define the quaternion as the following linear combination:
 * @f[
 *  \bar{q} = q_4+q_1\mathbf{i}+q_2\mathbf{j}+q_3\mathbf{k}
 * @f]
 * Where i,j,k are defined as the following:
 * @f[
 *  \mathbf{i}^2=-1~,~\mathbf{j}^2=-1~,~\mathbf{k}^2=-1
 * @f]
 * @f[
 *  -\mathbf{i}\mathbf{j}=\mathbf{j}\mathbf{i}=\mathbf{k}
 *  ~,~
 *  -\mathbf{j}\mathbf{k}=\mathbf{k}\mathbf{j}=\mathbf{i}
 *  ~,~
 *  -\mathbf{k}\mathbf{i}=\mathbf{i}\mathbf{k}=\mathbf{j}
 * @f]
 * As noted in [Trawny2005] this does not correspond to the Hamilton notation, and follows the "JPL Proposed Standard Conventions".
 * The q_4 quantity is the "scalar" portion of the quaternion, while q_1,q_2,q_3 are part of the "vector" portion.
 * We split the 4x1 vector into the following convention:
 * @f[
 *  \bar{q} = \begin{bmatrix}q_1\\q_2\\q_3\\q_4\end{bmatrix} = \begin{bmatrix}\mathbf{q}\\q_4\end{bmatrix}
 * @f]
 * It is also important to note that the quaternion is constrainted to the unit circle:
 * @f[
 *  |\bar{q}| = \sqrt{\bar{q}^\top\bar{q}} = \sqrt{|\mathbf{q}|^2+q_4^2} = 1
 * @f]
 *
 */


#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>


using namespace std;


/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {


    /**
     * \brief Returns a JPL quaternion from a rotation matrix
     * \param[in] rot 3x3 rotation matrix
     * \return 4x1 quaternion
     *
     * This is based on the equation 74 in [Trawny2005].
     * In the implementation, we have 4 statements so that we avoid a division by zero and
     * instead always divide by the largest diagonal element. This all comes from the
     * definition of a rotation matrix, using the diagonal elements and an off-diagonal.
     * \f[
     *  \mathbf{R}(\bar{q})=
     *  \begin{bmatrix}
     *  q_1^2-q_2^2-q_3^2+q_4^2 & 2(q_1q_2+q_3q_4) & 2(q_1q_3-q_2q_4) \\
     *  2(q_1q_2-q_3q_4) & -q_2^2+q_2^2-q_3^2+q_4^2 & 2(q_2q_3+q_1q_4) \\
     *  2(q_1q_3+q_2q_4) & 2(q_2q_3-q_1q_4) & -q_1^2-q_2^2+q_3^2+q_4^2
     *  \end{bmatrix}
     * \f]
     */
    inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::Matrix<double, 3, 3> &rot) {
        Eigen::Matrix<double, 4, 1> q;
        double T = rot.trace();
        if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2))) {
            //cout << "case 1- " << endl;
            q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
            q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
            q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
            q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

        } else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
            //cout << "case 2- " << endl;
            q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
            q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
            q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
            q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
        } else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1))) {
            //cout << "case 3- " << endl;
            q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
            q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
            q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
            q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
        } else {
            //cout << "case 4- " << endl;
            q(3) = sqrt((1 + T) / 4);
            q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
            q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
            q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
        }
        if (q(3) < 0) {
            q = -q;
        }
        // normalize and return
        q = q / (q.norm());
        return q;
    }

    /**
     * @brief Skew-symmetric matrix from a given 3x1 vector
     * @param[in] w 3x1 vector to be made a skew-symmetric
     * @return 3x3 skew-symmetric matrix
     *
     * This is based on equation 6 in [Trawny2005]:
     * @f[
     *  \lfloor\mathbf{v}\times\rfloor =
     *  \begin{bmatrix}
     *  0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0
     *  \end{bmatrix}
     * @f]
     */
    inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
        Eigen::Matrix<double, 3, 3> w_x;
        w_x << 0, -w(2), w(1),
                w(2), 0, -w(0),
                -w(1), w(0), 0;
        return w_x;
    }


    /**
     * @brief Converts JPL quaterion to SO(3) rotation matrix
     * @param[in] q JPL quaternion
     * @return 3x3 SO(3) rotation matrix
     *
     * This is based on equation 62 in [Trawny2005]:
     * @f[
     *  \mathbf{R} = (2q_4^2-1)\mathbf{I}_3-2q_4\lfloor\mathbf{q}\times\rfloor+2\mathbf{q}^\top\mathbf{q}
     * @f]
     */
    inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
        Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
        Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3)
                              - 2 * q(3, 0) * q_x +
                              2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
        return Rot;
    }


    /**
     * @brief Multiply two JPL quaternions
     * @param[in] q First JPL quaternion
     * @param[in] p Second JPL quaternion
     * @return 4x1 resulting p*q quaternion
     *
     * This is based on equation 9 in [Trawny2005].
     * We also enforce that the quaternion is unique by having q_4 be greater than zero.
     * @f[
     *  \bar{q}\otimes\bar{p}=
     *  \mathcal{L}(\bar{q})\bar{p}=
     *  \begin{bmatrix}
     *  q_4\mathbf{I}_3+\lfloor\mathbf{q}\times\rfloor & \mathbf{q} \\
     *  -\mathbf{q}^\top & q_4
     *  \end{bmatrix}
     *  \begin{bmatrix}
     *  \mathbf{p} \\ p_4
     *  \end{bmatrix}
     * @f]
     */
    inline Eigen::Matrix<double, 4, 1> quat_multiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p) {
        Eigen::Matrix<double, 4, 1> q_t;
        Eigen::Matrix<double, 4, 4> Qm;
        // create big L matrix
        Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
        Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
        Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
        Qm(3, 3) = q(3, 0);
        q_t = Qm * p;
        // ensure unique by forcing q_4 to be >0
        if (q_t(3, 0) < 0) {
            q_t *= -1;
        }
        // normalize and return
        return q_t / q_t.norm();
    }


    /**
     * @brief Returns vector portion of skew-symmetric
     * @param[in] w_x skew-symmetric matrix
     * @return 3x1 vector portion of skew
     *
     * See skew_x() for details.
     */
    inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x) {
        Eigen::Matrix<double, 3, 1> w;
        w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
        return w;
    }


    /**
     * @brief SO(3) matrix exponential
     * @param[in] w 3x1 vector we will take the exponential of
     * @return SO(3) rotation matrix
     *
     * SO(3) matrix exponential mapping from the vector to SO(3) lie group.
     * This formula ends up being the [Rodrigues formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula).
     * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 15.
     * http://ethaneade.com/lie.pdf
     *
     * @f[
     * \exp\colon\mathfrak{so}(3)\to SO(3)
     * @f]
     * @f[
     * \exp(\mathbf{v}) =
     * \mathbf{I}
     * +\frac{\sin{\theta}}{\theta}\lfloor\mathbf{v}\times\rfloor
     * +\frac{1-\cos{\theta}}{\theta^2}\lfloor\mathbf{v}\times\rfloor^2
     * @f]
     * @f[
     * \mathrm{where}\quad \theta^2 = \mathbf{v}^\top\mathbf{v}
     * @f]
     *
     * @m_class{m-block m-warning}
     *
     * @par TODO
     * Ethan Eade notes that one should use the Taylor series expansion of the coefficients
     * of the second and third terms when theta is small. Should we use that instead??
     * Right now we just handle if theta is zero, we set the exponential to be identity.
     */
    inline Eigen::Matrix<double, 3, 3> Exp(const Eigen::Matrix<double, 3, 1> &w) {

        Eigen::Matrix<double, 3, 3> w_x = skew_x(w);

        double theta = w.norm();

        Eigen::Matrix<double, 3, 3> R;

        if (theta == 0) {
            R = Eigen::MatrixXd::Identity(3, 3);
        } else {
            R = Eigen::MatrixXd::Identity(3, 3) + (sin(theta) / theta) * (w_x) +
                ((1 - cos(theta)) / pow(theta, 2)) * w_x * w_x;
        }
        return R;

    }


    /**
     * @brief SO(3) matrix logarithm
     * @param[in] R 3x3 SO(3) rotation matrix
     * @return 3x3 skew-symmetric matrix
     *
     * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 17 & 18.
     * http://ethaneade.com/lie.pdf
     *
     * @f[
     * \theta = \textrm{arccos}(0.5(\textrm{trace}(\mathbf{R})-1))
     * @f]
     * @f[
     * \lfloor\mathbf{v}\times\rfloor = \frac{\theta}{2\sin{\theta}}(\mathbf{R}-\mathbf{R}^\top)
     * @f]
     *
     * @m_class{m-block m-warning}
     *
     * @par TODO
     * Ethan Eade notes that one should use the Taylor series expansion of the coefficients
     * of the second and third terms when theta is small. Should we use that instead??
     * Right now we just handle if near the identity, we set the log to be all zeros.
     */
    inline Eigen::Matrix<double, 3, 3> Log(const Eigen::Matrix<double, 3, 3> &R) {
        Eigen::Matrix<double, 3, 3> w_x;
        // magnitude of the skew elements
        double theta = std::acos(0.5 * (R.trace() - 1));
        // calculate the skew symetric matrix
        w_x = (theta / (2 * sin(theta))) * (R - R.transpose());
        // check if we are near the identity
        if (R != Eigen::MatrixXd::Identity(3, 3)) {
            return w_x;
        } else {
            return Eigen::MatrixXd::Zero(3, 3);
        }
    }

    /**
     * @brief JPL Quaternion inverse
     * @param[in] q quaternion we want to change
     * @return inversed quaternion
     *
     * See equation 21 in [Trawny2005].
     * \f[
     *  \bar{q}^{-1} = \begin{bmatrix} -\mathbf{q} \\ q_4 \end{bmatrix}
     * \f]
     */
    inline Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q) {
        Eigen::Matrix<double, 4, 1> qinv;

        qinv.block(0, 0, 3, 1) = -q.block(0, 0, 3, 1);
        qinv(3, 0) = q(3, 0);
        return qinv;

    }

    /**
     * @brief equation (48) of trawny tech report [Trawny2005]
     */
    inline Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w) {
        Eigen::Matrix<double, 4, 4> mat;
        mat.block(0, 0, 3, 3) = -skew_x(w);
        mat.block(3, 0, 1, 3) = -w.transpose();
        mat.block(0, 3, 3, 1) = w;
        mat(3, 3) = 0;
        return mat;
    };

    /**
     * @brief Normalizes a quaternion to make sure it is unit norm
     */
    inline Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
        if (q_t(3, 0) < 0) {
            q_t *= -1;
        }
        return q_t / q_t.norm();
    };

    /**
     * @brief Computes left Jacobian of SO(3)
     * @param w axis-angle
     */
    inline Eigen::Matrix<double, 3, 3> Jl_so3(Eigen::Matrix<double, 3, 1> w) {

        double theta = w.norm();

        if (theta < 1e-12) {
            return Eigen::MatrixXd::Identity(3, 3);
        } else {
            Eigen::Matrix<double, 3, 1> a = w / theta;
            Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) +
                                            (1 - sin(theta) / theta) * a * a.transpose() +
                                            ((1 - cos(theta)) / theta) * skew_x(a);
            return J;

        }
    };

    /**
     * @brief Computes right Jacobian of SO(3)
     * @param w axis-angle
     */
    inline Eigen::Matrix<double, 3, 3> Jr_so3(Eigen::Matrix<double, 3, 1> w) {

        return Jl_so3(-w);
    };


}


#endif /* OV_CORE_QUAT_OPS_H */