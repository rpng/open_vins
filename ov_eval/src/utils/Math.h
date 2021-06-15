/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
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


#ifndef OV_EVAL_MATH_H
#define OV_EVAL_MATH_H

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace ov_eval {

/**
 * @brief JPL quaternion math utilities
 *
 * This file contains the common utility functions for operating on JPL quaternions.
 * We need to take special care to handle edge cases when converting to and from other rotation formats.
 * All equations are based on the following tech report:
 * > Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman filter for 3D attitude estimation."
 * > University of Minnesota, Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
 * > http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 *
 */
class Math {

public:
  /**
   * @brief Returns a JPL quaternion from a rotation matrix
   *
   * This is based on the equation 74 in [Indirect Kalman Filter for 3D Attitude
   * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf). In the implementation, we have 4 statements so that we avoid a division
   * by zero and instead always divide by the largest diagonal element. This all comes from the definition of a rotation matrix, using the
   * diagonal elements and an off-diagonal. \f{align*}{ \mathbf{R}(\bar{q})= \begin{bmatrix}
   *  q_1^2-q_2^2-q_3^2+q_4^2 & 2(q_1q_2+q_3q_4) & 2(q_1q_3-q_2q_4) \\
   *  2(q_1q_2-q_3q_4) & -q_2^2+q_2^2-q_3^2+q_4^2 & 2(q_2q_3+q_1q_4) \\
   *  2(q_1q_3+q_2q_4) & 2(q_2q_3-q_1q_4) & -q_1^2-q_2^2+q_3^2+q_4^2
   *  \end{bmatrix}
   * \f}
   *
   * @param[in] rot 3x3 rotation matrix
   * @return 4x1 quaternion
   */
  static inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::Matrix<double, 3, 3> &rot) {
    Eigen::Matrix<double, 4, 1> q;
    double T = rot.trace();
    if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2))) {
      // cout << "case 1- " << endl;
      q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
      q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
      q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
      q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

    } else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
      // cout << "case 2- " << endl;
      q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
      q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
      q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
      q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
    } else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1))) {
      // cout << "case 3- " << endl;
      q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
      q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
      q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
      q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
    } else {
      // cout << "case 4- " << endl;
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
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
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
    Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
                          2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
    return Rot;
  }

  /**
   * @brief Multiply two JPL quaternions
   *
   * This is based on equation 9 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
   * We also enforce that the quaternion is unique by having q_4 be greater than zero.
   * \f{align*}{
   *  \bar{q}\otimes\bar{p}=
   *  \mathcal{L}(\bar{q})\bar{p}=
   *  \begin{bmatrix}
   *  q_4\mathbf{I}_3+\lfloor\mathbf{q}\times\rfloor & \mathbf{q} \\
   *  -\mathbf{q}^\top & q_4
   *  \end{bmatrix}
   *  \begin{bmatrix}
   *  \mathbf{p} \\ p_4
   *  \end{bmatrix}
   * @f}
   *
   * @param[in] q First JPL quaternion
   * @param[in] p Second JPL quaternion
   * @return 4x1 resulting p*q quaternion
   */
  static inline Eigen::Matrix<double, 4, 1> quat_multiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p) {
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
   *
   * See skew_x() for details.
   *
   * @param[in] w_x skew-symmetric matrix
   * @return 3x1 vector portion of skew
   */
  static inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x) {
    Eigen::Matrix<double, 3, 1> w;
    w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
    return w;
  }

  /**
   * @brief SO(3) matrix exponential
   *
   * SO(3) matrix exponential mapping from the vector to SO(3) lie group.
   * This formula ends up being the [Rodrigues formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula).
   * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 15.
   * http://ethaneade.com/lie.pdf
   *
   * \f{align*}{
   * \exp\colon\mathfrak{so}(3)&\to SO(3) \\
   * \exp(\mathbf{v}) &=
   * \mathbf{I}
   * +\frac{\sin{\theta}}{\theta}\lfloor\mathbf{v}\times\rfloor
   * +\frac{1-\cos{\theta}}{\theta^2}\lfloor\mathbf{v}\times\rfloor^2 \\
   * \mathrm{where}&\quad \theta^2 = \mathbf{v}^\top\mathbf{v}
   * @f}
   *
   * @param[in] w 3x1 vector we will take the exponential of
   * @return SO(3) rotation matrix
   */
  static inline Eigen::Matrix<double, 3, 3> exp_so3(const Eigen::Matrix<double, 3, 1> &w) {
    // get theta
    Eigen::Matrix<double, 3, 3> w_x = skew_x(w);
    double theta = w.norm();
    // Handle small angle values
    double A, B;
    if (theta < 1e-12) {
      A = 1;
      B = 0.5;
    } else {
      A = sin(theta) / theta;
      B = (1 - cos(theta)) / (theta * theta);
    }
    // compute so(3) rotation
    Eigen::Matrix<double, 3, 3> R;
    if (theta == 0) {
      R = Eigen::MatrixXd::Identity(3, 3);
    } else {
      R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
    }
    return R;
  }

  /**
   * @brief SO(3) matrix logarithm
   *
   * This definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan Eade equation 17 & 18.
   * http://ethaneade.com/lie.pdf
   * \f{align*}{
   * \theta &= \textrm{arccos}(0.5(\textrm{trace}(\mathbf{R})-1)) \\
   * \lfloor\mathbf{v}\times\rfloor &= \frac{\theta}{2\sin{\theta}}(\mathbf{R}-\mathbf{R}^\top)
   * @f}
   *
   * @param[in] R 3x3 SO(3) rotation matrix
   * @return 3x1 in the se(3) space [omegax, omegay, omegaz]
   */
  static inline Eigen::Matrix<double, 3, 1> log_so3(const Eigen::Matrix<double, 3, 3> &R) {
    // magnitude of the skew elements (handle edge case where we sometimes have a>1...)
    double a = 0.5 * (R.trace() - 1);
    double theta = (a > 1) ? acos(1) : ((a < -1) ? acos(-1) : acos(a));
    // Handle small angle values
    double D;
    if (theta < 1e-12) {
      D = 0.5;
    } else {
      D = theta / (2 * sin(theta));
    }
    // calculate the skew symetric matrix
    Eigen::Matrix<double, 3, 3> w_x = D * (R - R.transpose());
    // check if we are near the identity
    if (R != Eigen::MatrixXd::Identity(3, 3)) {
      Eigen::Vector3d vec;
      vec << w_x(2, 1), w_x(0, 2), w_x(1, 0);
      return vec;
    } else {
      return Eigen::Vector3d::Zero();
    }
  }

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
  static inline Eigen::Matrix4d exp_se3(Eigen::Matrix<double, 6, 1> vec) {

    // Precompute our values
    Eigen::Vector3d w = vec.head(3);
    Eigen::Vector3d u = vec.tail(3);
    double theta = sqrt(w.dot(w));
    Eigen::Matrix3d wskew;
    wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

    // Handle small angle values
    double A, B, C;
    if (theta < 1e-12) {
      A = 1;
      B = 0.5;
      C = 1.0 / 6.0;
    } else {
      A = sin(theta) / theta;
      B = (1 - cos(theta)) / (theta * theta);
      C = (1 - A) / (theta * theta);
    }

    // Matrices we need V and Identity
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d V = I_33 + B * wskew + C * wskew * wskew;

    // Get the final matrix to return
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block(0, 0, 3, 3) = I_33 + A * wskew + B * wskew * wskew;
    mat.block(0, 3, 3, 1) = V * u;
    mat(3, 3) = 1;
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
   * \mathbf V^{-1} &= \mathbf I - \frac{1}{2} \lfloor \boldsymbol\omega \times\rfloor + \frac{1}{\theta^2}\Big(1-\frac{A}{2B}\Big)\lfloor
   * \boldsymbol\omega \times\rfloor^2 \f}
   *
   * @param mat 4x4 SE(3) matrix
   * @return 6x1 in the se(3) space [omega, u]
   */
  static inline Eigen::Matrix<double, 6, 1> log_se3(Eigen::Matrix4d mat) {

    // Get sub-matrices
    Eigen::Matrix3d R = mat.block(0, 0, 3, 3);
    Eigen::Vector3d t = mat.block(0, 3, 3, 1);

    // Get theta (handle edge case where we sometimes have a>1...)
    double a = 0.5 * (R.trace() - 1);
    double theta = (a > 1) ? acos(1) : ((a < -1) ? acos(-1) : acos(a));

    // Handle small angle values
    double A, B, D, E;
    if (theta < 1e-12) {
      A = 1;
      B = 0.5;
      D = 0.5;
      E = 1.0 / 12.0;
    } else {
      A = sin(theta) / theta;
      B = (1 - cos(theta)) / (theta * theta);
      D = theta / (2 * sin(theta));
      E = 1 / (theta * theta) * (1 - 0.5 * A / B);
    }

    // Get the skew matrix and V inverse
    Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d wskew = D * (R - R.transpose());
    Eigen::Matrix3d Vinv = I_33 - 0.5 * wskew + E * wskew * wskew;

    // Calculate vector
    Eigen::Matrix<double, 6, 1> vec;
    vec.head(3) << wskew(2, 1), wskew(0, 2), wskew(1, 0);
    vec.tail(3) = Vinv * t;
    return vec;
  }

  /**
   * @brief Hat operator for R^6 -> Lie Algebra se(3)
   *
   * \f{align*}{
   * \boldsymbol\Omega^{\wedge} = \begin{bmatrix} \lfloor \boldsymbol\omega \times\rfloor & \mathbf u \\ \mathbf 0 & 0 \end{bmatrix}
   * \f}
   *
   * @param vec 6x1 in the se(3) space [omega, u]
   * @return Lie algebra se(3) 4x4 matrix
   */
  static inline Eigen::Matrix4d hat_se3(const Eigen::Matrix<double, 6, 1> &vec) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block(0, 0, 3, 3) = skew_x(vec.head(3));
    mat.block(0, 3, 3, 1) = vec.tail(3);
    return mat;
  }

  /**
   * @brief SE(3) matrix analytical inverse
   *
   * It seems that using the .inverse() function is not a good way.
   * This should be used in all cases we need the inverse instead of numerical inverse.
   * https://github.com/rpng/open_vins/issues/12
   * \f{align*}{
   * \mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^\top & -\mathbf{R}^\top\mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix}
   * \f}
   *
   * @param[in] T SE(3) matrix
   * @return inversed SE(3) matrix
   */
  static inline Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T) {
    Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
    Tinv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
    Tinv.block(0, 3, 3, 1) = -Tinv.block(0, 0, 3, 3) * T.block(0, 3, 3, 1);
    return Tinv;
  }

  /**
   * @brief JPL Quaternion inverse
   *
   * See equation 21 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
   * \f{align*}{
   *  \bar{q}^{-1} = \begin{bmatrix} -\mathbf{q} \\ q_4 \end{bmatrix}
   * \f}
   *
   * @param[in] q quaternion we want to change
   * @return inversed quaternion
   */
  static inline Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q) {
    Eigen::Matrix<double, 4, 1> qinv;
    qinv.block(0, 0, 3, 1) = -q.block(0, 0, 3, 1);
    qinv(3, 0) = q(3, 0);
    return qinv;
  }

  /**
   * @brief Integrated quaternion from angular velocity
   *
   * See equation (48) of trawny tech report [Indirect Kalman Filter for 3D Attitude
   * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
   *
   */
  static inline Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 4, 4> mat;
    mat.block(0, 0, 3, 3) = -skew_x(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
  }

  /**
   * @brief Normalizes a quaternion to make sure it is unit norm
   * @param q_t Quaternion to normalized
   * @return Normalized quaterion
   */
  static inline Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
    if (q_t(3, 0) < 0) {
      q_t *= -1;
    }
    return q_t / q_t.norm();
  }

  /**
   * @brief Gets roll, pitch, yaw of argument rotation (in that order).
   * To recover the matrix: R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
   * @param rot Rotation matrix
   */
  static inline Eigen::Matrix<double, 3, 1> rot2rpy(const Eigen::Matrix<double, 3, 3> &rot) {
    Eigen::Matrix<double, 3, 1> rpy;
    rpy(1, 0) = atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
    if (std::abs(cos(rpy(1, 0)) > 1.0e-12)) {
      rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
      rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
    } else {
      rpy(2, 0) = 0;
      rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
    }
    return rpy;
  }

  /**
   * @brief Construct rotation matrix from given roll
   * @param t roll angle
   */
  static inline Eigen::Matrix<double, 3, 3> rot_x(double t) {
    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
    return r;
  }

  /**
   * @brief Construct rotation matrix from given pitch
   * @param t pitch angle
   */
  static inline Eigen::Matrix<double, 3, 3> rot_y(double t) {
    Eigen::Matrix<double, 3, 3> r;
    double ct = cos(t);
    double st = sin(t);
    r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
    return r;
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
};

} // namespace ov_eval

#endif /* OV_EVAL_MATH_H */