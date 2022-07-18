#ifndef CPI_V1_H
#define CPI_V1_H

/*
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "CpiBase.h"

#include <Eigen/Dense>

namespace ov_core {

/**
 * @brief Model 1 of continuous preintegration.
 *
 * This model is the "piecewise constant measurement assumption" which was first presented in:
 * > Eckenhoff, Kevin, Patrick Geneva, and Guoquan Huang.
 * > "High-accuracy preintegration for visual inertial navigation."
 * > International Workshop on the Algorithmic Foundations of Robotics. 2016.
 * Please see the following publication for details on the theory @cite Eckenhoff2019IJRR :
 * > Continuous Preintegration Theory for Graph-based Visual-Inertial Navigation
 * > Authors: Kevin Eckenhoff, Patrick Geneva, and Guoquan Huang
 * > http://udel.edu/~ghuang/papers/tr_cpi.pdf
 *
 * The steps to use this preintegration class are as follows:
 * 1. call setLinearizationPoints() to set the bias/orientation linearization point
 * 2. call feed_IMU() will all IMU measurements you want to precompound over
 * 3. access public varibles, to get means, Jacobians, and measurement covariance
 */
class CpiV1 : public CpiBase {

public:
  /**
   * @brief Default constructor for our Model 1 preintegration (piecewise constant measurement assumption)
   * @param sigma_w gyroscope white noise density (rad/s/sqrt(hz))
   * @param sigma_wb gyroscope random walk (rad/s^2/sqrt(hz))
   * @param sigma_a accelerometer white noise density (m/s^2/sqrt(hz))
   * @param sigma_ab accelerometer random walk (m/s^3/sqrt(hz))
   * @param imu_avg_ if we want to average the imu measurements (IJRR paper did not do this)
   */
  CpiV1(double sigma_w, double sigma_wb, double sigma_a, double sigma_ab, bool imu_avg_ = false)
      : CpiBase(sigma_w, sigma_wb, sigma_a, sigma_ab, imu_avg_) {}

  virtual ~CpiV1() {}

  /**
   * @brief Our precompound function for Model 1
   * @param[in] t_0 first IMU timestamp
   * @param[in] t_1 second IMU timestamp
   * @param[in] w_m_0 first imu gyroscope measurement
   * @param[in] a_m_0 first imu acceleration measurement
   * @param[in] w_m_1 second imu gyroscope measurement
   * @param[in] a_m_1 second imu acceleration measurement
   *
   * We will first analytically integrate our meansurements and Jacobians.
   * Then we perform numerical integration for our measurement covariance.
   */
  void feed_IMU(double t_0, double t_1, Eigen::Matrix<double, 3, 1> w_m_0, Eigen::Matrix<double, 3, 1> a_m_0,
                Eigen::Matrix<double, 3, 1> w_m_1 = Eigen::Matrix<double, 3, 1>::Zero(),
                Eigen::Matrix<double, 3, 1> a_m_1 = Eigen::Matrix<double, 3, 1>::Zero());
};

} // namespace ov_core

#endif /* CPI_V1_H */