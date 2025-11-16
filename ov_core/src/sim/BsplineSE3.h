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

#ifndef OV_CORE_BSPLINESE3_H
#define OV_CORE_BSPLINESE3_H

#include <Eigen/Eigen>
#include <map>
#include <vector>

#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_core {

/**
 * @brief B-Spline which performs interpolation over SE(3) manifold.
 *
 * This class implements the b-spline functionality that allows for interpolation over the \f$\mathbb{SE}(3)\f$ manifold.
 * This is based off of the derivations from [Continuous-Time Visual-Inertial Odometry for Event
 * Cameras](https://ieeexplore.ieee.org/abstract/document/8432102/) and [A Spline-Based Trajectory Representation for Sensor Fusion and
 * Rolling Shutter Cameras](https://link.springer.com/article/10.1007/s11263-015-0811-3) with some additional derivations being available in
 * [these notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf). The use of b-splines for \f$\mathbb{SE}(3)\f$
 * interpolation has the following properties:
 *
 * 1. Local control, allowing the system to function online as well as in batch
 * 2. \f$C^2\f$-continuity to enable inertial predictions and calculations
 * 3. Good approximation of minimal torque trajectories
 * 4. A parameterization of rigid-body motion devoid of singularities
 *
 * The key idea is to convert a set of trajectory points into a continuous-time *uniform cubic cumulative* b-spline.
 * As compared to standard b-spline representations, the cumulative form ensures local continuity which is needed for on-manifold
 * interpolation. We leverage the cubic b-spline to ensure \f$C^2\f$-continuity to ensure that we can calculate accelerations at any point
 * along the trajectory. The general equations are the following
 *
 * \f{align*}{
 *  {}^{w}_{s}\mathbf{T}(u(t))
 *  &= {}^{w}_{i-1}\mathbf{T}~\mathbf{A}_0~\mathbf{A}_1~\mathbf{A}_2 \\
 * \empty
 *  {}^{w}_{s}\dot{\mathbf{T}}(u(t)) &=
 *  {}^{w}_{i-1}\mathbf{T}
 *  \Big(
 *  \dot{\mathbf{A}}_0~\mathbf{A}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\dot{\mathbf{A}}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\mathbf{A}_1~\dot{\mathbf{A}}_2
 *  \Big) \\
 * \empty
 *  {}^{w}_{s}\ddot{\mathbf{T}}(u(t)) &=
 *  {}^{w}_{i-1}\mathbf{T}
 *  \Big(
 *  \ddot{\mathbf{A}}_0~\mathbf{A}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\ddot{\mathbf{A}}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\mathbf{A}_1~\ddot{\mathbf{A}}_2 \nonumber\\
 *  &\hspace{4cm}
 *  + 2\dot{\mathbf{A}}_0\dot{\mathbf{A}}_1\mathbf{A}_2 +
 *  2\mathbf{A}_0\dot{\mathbf{A}}_1\dot{\mathbf{A}}_2 +
 *  2\dot{\mathbf{A}}_0\mathbf{A}_1\dot{\mathbf{A}}_2
 *  \Big)  \\[1em]
 * \empty
 *  {}^{i-1}_{i}\mathbf{\Omega} &= \mathrm{log}\big( {}^{w}_{i-1}\mathbf{T}^{-1}~{}^{w}_{i}\mathbf{T} \big) \\
 *  \mathbf{A}_j &= \mathrm{exp}\Big({B}_j(u(t))~{}^{i-1+j}_{i+j}\mathbf{\Omega} \Big) \\
 *  \dot{\mathbf{A}}_j &= \dot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\mathbf{A}_j \\
 *  \ddot{\mathbf{A}}_j &=
 *  \dot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\dot{\mathbf{A}}_j +
 *  \ddot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\mathbf{A}_j  \\[1em]
 * \empty
 *  {B}_0(u(t)) &= \frac{1}{3!}~(5+3u-3u^2+u^3) \\
 *  {B}_1(u(t)) &= \frac{1}{3!}~(1+3u+3u^2-2u^3) \\
 *  {B}_2(u(t)) &= \frac{1}{3!}~(u^3) \\[1em]
 * \empty
 *  \dot{{B}}_0(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3-6u+3u^2) \\
 *  \dot{{B}}_1(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3+6u-6u^2) \\
 *  \dot{{B}}_2(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3u^2) \\[1em]
 * \empty
 *  \ddot{{B}}_0(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(-6+6u) \\
 *  \ddot{{B}}_1(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(6-12u) \\
 *  \ddot{{B}}_2(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(6u)
 * \f}
 *
 * where \f$u(t_s)=(t_s-t_i)/\Delta t=(t_s-t_i)/(t_{i+1}-t_i)\f$ is used for all values of *u*.
 * Note that one needs to ensure that they use the SE(3) matrix expodential, logorithm, and hat operation for all above equations.
 * The indexes correspond to the the two poses that are older and two poses that are newer then the current time we want to get (i.e. i-1
 * and i are less than s, while i+1 and i+2 are both greater than time s). Some additional derivations are available in [these
 * notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf).
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
   * Our control points need to be uniformly spaced over the trajectory, thus given a trajectory we will
   * uniformly sample based on the average spacing between the pose points specified.
   *
   * @param traj_points Trajectory poses that we will convert into control points (timestamp(s), q_GtoI, p_IinG)
   */
  void feed_trajectory(std::vector<Eigen::VectorXd> traj_points);

  /**
   * @brief Gets the orientation and position at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @return False if we can't find it
   */
  bool get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG);

  /**
   * @brief Gets the angular and linear velocity at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @param w_IinI Angular velocity in the inertial frame
   * @param v_IinG Linear velocity in the global frame
   * @return False if we can't find it
   */
  bool get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI, Eigen::Vector3d &v_IinG);

  /**
   * @brief Gets the angular and linear acceleration at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @param w_IinI Angular velocity in the inertial frame
   * @param v_IinG Linear velocity in the global frame
   * @param alpha_IinI Angular acceleration in the inertial frame
   * @param a_IinG Linear acceleration in the global frame
   * @return False if we can't find it
   */
  bool get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI,
                        Eigen::Vector3d &v_IinG, Eigen::Vector3d &alpha_IinI, Eigen::Vector3d &a_IinG);

  /// Returns the simulation start time that we should start simulating from
  double get_start_time() { return timestamp_start; }

protected:
  /// Uniform sampling time for our control points
  double dt;

  /// Start time of the system
  double timestamp_start;

  /// Type defintion of our aligned eigen4d matrix: https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
  typedef std::map<double, Eigen::Matrix4d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4d>>>
      AlignedEigenMat4d;

  /// Our control SE3 control poses (R_ItoG, p_IinG)
  AlignedEigenMat4d control_points;

  /**
   * @brief Will find the two bounding poses for a given timestamp.
   *
   * This will loop through the passed map of poses and find two bounding poses.
   * If there are no bounding poses then this will return false.
   *
   * @param timestamp Desired timestamp we want to get two bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                  Eigen::Matrix4d &pose1);

  /**
   * @brief Will find two older poses and two newer poses for the current timestamp
   *
   * @param timestamp Desired timestamp we want to get four bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @param t2 Timestamp of the third pose
   * @param pose2 SE(3) pose of the third pose
   * @param t3 Timestamp of the fourth pose
   * @param pose3 SE(3) pose of the fourth pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                           double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                           Eigen::Matrix4d &pose3);
};

} // namespace ov_core

#endif // OV_CORE_BSPLINESE3_H
