/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#ifndef OV_CORE_QUAT_OPS_H
#define OV_CORE_QUAT_OPS_H

/*
 * @section Summary
 * This file contains the common utility functions for operating on JPL quaternions.
 * We need to take special care to handle edge cases when converting to and from other rotation formats.
 * All equations are based on the following tech report @cite Trawny2005TR :
 *
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
 * As noted in @cite Trawny2005TR this does not correspond to the Hamilton notation, and follows the "JPL Proposed Standard Conventions".
 * The q_4 quantity is the "scalar" portion of the quaternion, while q_1,q_2,q_3 are part of the "vector" portion.
 * We split the 4x1 vector into the following convention:
 * @f[
 *  \bar{q} = \begin{bmatrix}q_1\\q_2\\q_3\\q_4\end{bmatrix} = \begin{bmatrix}\mathbf{q}\\q_4\end{bmatrix}
 * @f]
 * It is also important to note that the quaternion is constrained to the unit circle:
 * @f[
 *  |\bar{q}| = \sqrt{\bar{q}^\top\bar{q}} = \sqrt{|\mathbf{q}|^2+q_4^2} = 1
 * @f]
 *
 */

#include <Eigen/Eigen>

namespace ov_core {

/**
 * @brief Returns a JPL quaternion from a rotation matrix
 *
 * This is based on the equation 74 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 * In the implementation, we have 4 statements so that we avoid a division by zero and
 * instead always divide by the largest diagonal element. This all comes from the
 * definition of a rotation matrix, using the diagonal elements and an off-diagonal.
 * \f{align*}{
 *  \mathbf{R}(\bar{q})=
 *  \begin{bmatrix}
 *  q_1^2-q_2^2-q_3^2+q_4^2 & 2(q_1q_2+q_3q_4) & 2(q_1q_3-q_2q_4) \\
 *  2(q_1q_2-q_3q_4) & -q_2^2+q_2^2-q_3^2+q_4^2 & 2(q_2q_3+q_1q_4) \\
 *  2(q_1q_3+q_2q_4) & 2(q_2q_3-q_1q_4) & -q_1^2-q_2^2+q_3^2+q_4^2
 *  \end{bmatrix}
 * \f}
 *
 * @param[in] rot 3x3 rotation matrix
 * @return 4x1 quaternion
 */
inline Eigen::Matrix<double, 4, 1> rot_2_quat(const Eigen::Matrix<double, 3, 3> &rot) {
  Eigen::Matrix<double, 4, 1> q;
  double T = rot.trace();
  if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) && (rot(0, 0) >= rot(2, 2))) {
    q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
    q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
    q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

  } else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) && (rot(1, 1) >= rot(2, 2))) {
    q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
    q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
  } else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) && (rot(2, 2) >= rot(1, 1))) {
    q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
    q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
    q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
  } else {
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
inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

/**
 * @brief Converts JPL quaterion to SO(3) rotation matrix
 *
 * This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf):
 * \f{align*}{
 *  \mathbf{R} = (2q_4^2-1)\mathbf{I}_3-2q_4\lfloor\mathbf{q}\times\rfloor+2\mathbf{q}\mathbf{q}^\top
 * @f}
 *
 * @param[in] q JPL quaternion
 * @return 3x3 SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
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
 * @return 4x1 resulting q*p quaternion
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
 *
 * See skew_x() for details.
 *
 * @param[in] w_x skew-symmetric matrix
 * @return 3x1 vector portion of skew
 */
inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x) {
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
 * @param[in] w 3x1 vector in R(3) we will take the exponential of
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> exp_so3(const Eigen::Matrix<double, 3, 1> &w) {
  // get theta
  Eigen::Matrix<double, 3, 3> w_x = skew_x(w);
  double theta = w.norm();
  // Handle small angle values
  double A, B;
  if (theta < 1e-7) {
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
 * This function is based on the GTSAM one as the original implementation was a bit unstable.
 * See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param[in] R 3x3 SO(3) rotation matrix
 * @return 3x1 in the R(3) space [omegax, omegay, omegaz]
 */
inline Eigen::Matrix<double, 3, 1> log_so3(const Eigen::Matrix<double, 3, 3> &R) {

  // note switch to base 1
  double R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  double R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  double R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();
  Eigen::Vector3d omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
      omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0;
    }
    omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
  }

  return omega;
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
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return 4x4 SE(3) matrix
 */
inline Eigen::Matrix4d exp_se3(Eigen::Matrix<double, 6, 1> vec) {

  // Precompute our values
  Eigen::Vector3d w = vec.head(3);
  Eigen::Vector3d u = vec.tail(3);
  double theta = sqrt(w.dot(w));
  Eigen::Matrix3d wskew;
  wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Handle small angle values
  double A, B, C;
  if (theta < 1e-7) {
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
 * This function is based on the GTSAM one as the original implementation was a bit unstable.
 * See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param mat 4x4 SE(3) matrix
 * @return 6x1 in the R(6) space [omega, u]
 */
inline Eigen::Matrix<double, 6, 1> log_se3(Eigen::Matrix4d mat) {
  Eigen::Vector3d w = log_so3(mat.block<3, 3>(0, 0));
  Eigen::Vector3d T = mat.block<3, 1>(0, 3);
  const double t = w.norm();
  if (t < 1e-10) {
    Eigen::Matrix<double, 6, 1> log;
    log << w, T;
    return log;
  } else {
    Eigen::Matrix3d W = skew_x(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    double Tan = tan(0.5 * t);
    Eigen::Vector3d WT = W * T;
    Eigen::Vector3d u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    Eigen::Matrix<double, 6, 1> log;
    log << w, u;
    return log;
  }
}

/**
 * @brief Hat operator for R^6 -> Lie Algebra se(3)
 *
 * \f{align*}{
 * \boldsymbol\Omega^{\wedge} = \begin{bmatrix} \lfloor \boldsymbol\omega \times\rfloor & \mathbf u \\ \mathbf 0 & 0 \end{bmatrix}
 * \f}
 *
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return Lie algebra se(3) 4x4 matrix
 */
inline Eigen::Matrix4d hat_se3(const Eigen::Matrix<double, 6, 1> &vec) {
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
inline Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T) {
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
inline Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q) {
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
 * This matrix is derived in Section 1.5 of the report by finding the Quaterion Time Derivative.
 *
 * \f{align*}{
 * \boldsymbol{\Omega}(\boldsymbol{\omega}) &=
 * \begin{bmatrix}
 * -\lfloor{\boldsymbol{\omega}}  \rfloor & \boldsymbol{\omega} \\
 * -\boldsymbol{\omega}^\top & 0
 * \end{bmatrix}
 * \f}
 *
 * @param w Angular velocity
 * @return The matrix \f$\boldsymbol{\Omega}\f$
 */
inline Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w) {
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
inline Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
  if (q_t(3, 0) < 0) {
    q_t *= -1;
  }
  return q_t / q_t.norm();
}

/**
 * @brief Computes left Jacobian of SO(3)
 *
 * The left Jacobian of SO(3) is defined equation (7.77b) in [State Estimation for
 * Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by Timothy D. Barfoot @cite Barfoot2017. Specifically it is the
 * following (with \f$\theta=|\boldsymbol\theta|\f$ and \f$\mathbf a=\boldsymbol\theta/|\boldsymbol\theta|\f$): \f{align*}{
 * J_l(\boldsymbol\theta) = \frac{\sin\theta}{\theta}\mathbf I + \Big(1-\frac{\sin\theta}{\theta}\Big)\mathbf a \mathbf a^\top +
 * \frac{1-\cos\theta}{\theta}\lfloor \mathbf a \times\rfloor \f}
 *
 * @param w axis-angle
 * @return The left Jacobian of SO(3)
 */
inline Eigen::Matrix<double, 3, 3> Jl_so3(const Eigen::Matrix<double, 3, 1> &w) {
  double theta = w.norm();
  if (theta < 1e-6) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J = sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) + (1 - sin(theta) / theta) * a * a.transpose() +
                                    ((1 - cos(theta)) / theta) * skew_x(a);
    return J;
  }
}

/**
 * @brief Computes right Jacobian of SO(3)
 *
 * The right Jacobian of SO(3) is related to the left by Jl(-w)=Jr(w).
 * See equation (7.87) in [State Estimation for Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by Timothy D. Barfoot
 * @cite Barfoot2017. See @ref Jl_so3() for the definition of the left Jacobian of SO(3).
 *
 * @param w axis-angle
 * @return The right Jacobian of SO(3)
 */
inline Eigen::Matrix<double, 3, 3> Jr_so3(const Eigen::Matrix<double, 3, 1> &w) { return Jl_so3(-w); }

/**
 * @brief Gets roll, pitch, yaw of argument rotation (in that order).
 *
 * To recover the matrix: R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
 * If you are interested in how to compute Jacobians checkout this report:
 * http://mars.cs.umn.edu/tr/reports/Trawny05c.pdf
 *
 * @param rot SO(3) rotation matrix
 * @return roll, pitch, yaw values (in that order)
 */
inline Eigen::Matrix<double, 3, 1> rot2rpy(const Eigen::Matrix<double, 3, 3> &rot) {
  Eigen::Matrix<double, 3, 1> rpy;
  rpy(1, 0) = atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  if (std::abs(cos(rpy(1, 0))) > 1.0e-12) {
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
 *
 * \f{align*}{
 * \mathbf{R}_x(t) &=
 * \begin{bmatrix}
 * 1 & 0 & 0 \\
 * 0 & \cos(t) & -\sin(t) \\
 * 0 & \sin(t) & \cos(t)
 * \end{bmatrix}
 * \f}
 *
 * @param t roll angle
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> rot_x(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
  return r;
}

/**
 * @brief Construct rotation matrix from given pitch
 *
 * \f{align*}{
 * \mathbf{R}_y(t) &=
 * \begin{bmatrix}
 * \cos(t) & 0 & \sin(t) \\
 * 0 & 1 & 0 \\
 * -\sin(t) & 0 & \cos(t)
 * \end{bmatrix}
 * \f}
 *
 * @param t pitch angle
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> rot_y(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
  return r;
}

/**
 * @brief Construct rotation matrix from given yaw
 *
 * \f{align*}{
 * \mathbf{R}_z(t) &=
 * \begin{bmatrix}
 * \cos(t) & -\sin(t) & 0 \\
 * \sin(t) & \cos(t) & 0 \\
 * 0 & 0 & 1
 * \end{bmatrix}
 * \f}
 *
 * @param t yaw angle
 * @return SO(3) rotation matrix
 */
inline Eigen::Matrix<double, 3, 3> rot_z(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
  return r;
}

} // namespace ov_core

#endif /* OV_CORE_QUAT_OPS_H */