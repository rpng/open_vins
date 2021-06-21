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


#ifndef OV_MSCKF_STATE_PROPAGATOR_H
#define OV_MSCKF_STATE_PROPAGATOR_H

#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;

namespace ov_msckf {

/**
 * @brief Performs the state covariance and mean propagation using imu measurements
 *
 * We will first select what measurements we need to propagate with.
 * We then compute the state transition matrix at each step and update the state and covariance.
 * For derivations look at @ref propagation page which has detailed equations.
 */
class Propagator {

public:
  /**
   * @brief Struct of our imu noise parameters
   */
  struct NoiseManager {

    /// Gyroscope white noise (rad/s/sqrt(hz))
    double sigma_w = 1.6968e-04;

    /// Gyroscope white noise covariance
    double sigma_w_2 = pow(1.6968e-04, 2);

    /// Gyroscope random walk (rad/s^2/sqrt(hz))
    double sigma_wb = 1.9393e-05;

    /// Gyroscope random walk covariance
    double sigma_wb_2 = pow(1.9393e-05, 2);

    /// Accelerometer white noise (m/s^2/sqrt(hz))
    double sigma_a = 2.0000e-3;

    /// Accelerometer white noise covariance
    double sigma_a_2 = pow(2.0000e-3, 2);

    /// Accelerometer random walk (m/s^3/sqrt(hz))
    double sigma_ab = 3.0000e-03;

    /// Accelerometer random walk covariance
    double sigma_ab_2 = pow(3.0000e-03, 2);

    /// Nice print function of what parameters we have loaded
    void print() {
      printf("\t- gyroscope_noise_density: %.6f\n", sigma_w);
      printf("\t- accelerometer_noise_density: %.5f\n", sigma_a);
      printf("\t- gyroscope_random_walk: %.7f\n", sigma_wb);
      printf("\t- accelerometer_random_walk: %.6f\n", sigma_ab);
    }
  };

  /**
   * @brief Default constructor
   * @param noises imu noise characteristics (continuous time)
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   */
  Propagator(NoiseManager noises, double gravity_mag) : _noises(noises) {
    _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
    _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
    _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
    _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);
    last_prop_time_offset = 0.0;
    _gravity << 0.0, 0.0, gravity_mag;
  }

  /**
   * @brief Stores incoming inertial readings
   * @param message Contains our timestamp and inertial information
   */
  void feed_imu(const ov_core::ImuData &message) {

    // Append it to our vector
    imu_data.emplace_back(message);

    // Loop through and delete imu messages that are older then 10 seconds
    // TODO: we should probably have more elegant logic then this
    // TODO: but this prevents unbounded memory growth and slow prop with high freq imu
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (message.timestamp - (*it0).timestamp > 10) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Propagate state up to given timestamp and then clone
   *
   * This will first collect all imu readings that occured between the
   * *current* state time and the new time we want the state to be at.
   * If we don't have any imu readings we will try to extrapolate into the future.
   * After propagating the mean and covariance using our dynamics,
   * We clone the current imu pose as a new clone in our state.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to and clone at
   */
  void propagate_and_clone(std::shared_ptr<State> state, double timestamp);

  /**
   * @brief Gets what the state and its covariance will be at a given timestamp
   *
   * This can be used to find what the state will be in the "future" without propagating it.
   * This will propagate a clone of the current IMU state and its covariance matrix.
   * This is typically used to provide high frequency pose estimates between updates.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to
   * @param state_plus The propagated state (q_GtoI, p_IinG, v_IinG, w_IinI)
   */
  void fast_state_propagate(std::shared_ptr<State> state, double timestamp, Eigen::Matrix<double, 13, 1> &state_plus);

  /**
   * @brief Helper function that given current imu data, will select imu readings between the two times.
   *
   * This will create measurements that we will integrate with, and an extra measurement at the end.
   * We use the @ref interpolate_data() function to "cut" the imu readings at the begining and end of the integration.
   * The timestamps passed should already take into account the time offset values.
   *
   * @param imu_data IMU data we will select measurements from
   * @param time0 Start timestamp
   * @param time1 End timestamp
   * @param warn If we should warn if we don't have enough IMU to propagate with (e.g. fast prop will get warnings otherwise)
   * @return Vector of measurements (if we could compute them)
   */
  static std::vector<ov_core::ImuData> select_imu_readings(const std::vector<ov_core::ImuData> &imu_data, double time0, double time1,
                                                           bool warn = true);

  /**
   * @brief Nice helper function that will linearly interpolate between two imu messages.
   *
   * This should be used instead of just "cutting" imu messages that bound the camera times
   * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
   *
   * @param imu_1 imu at begining of interpolation interval
   * @param imu_2 imu at end of interpolation interval
   * @param timestamp Timestamp being interpolated to
   */
  static ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1, const ov_core::ImuData &imu_2, double timestamp) {
    // time-distance lambda
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    // cout << "lambda - " << lambda << endl;
    // interpolate between the two times
    ov_core::ImuData data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }

protected:
  /// Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  /**
   * @brief Propagates the state forward using the imu data and computes the noise covariance and state-transition
   * matrix of this interval.
   *
   * This function can be replaced with analytical/numerical integration or when using a different state representation.
   * This contains our state transition matrix along with how our noise evolves in time.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref error_prop page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param data_minus imu readings at beginning of interval
   * @param data_plus imu readings at end of interval
   * @param F State-transition matrix over the interval
   * @param Qd Discrete-time noise covariance over the interval
   */
  void predict_and_compute(std::shared_ptr<State> state, const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus,
                           Eigen::Matrix<double, 15, 15> &F, Eigen::Matrix<double, 15, 15> &Qd);

  /**
   * @brief Discrete imu mean propagation.
   *
   * See @ref propagation for details on these equations.
   * \f{align*}{
   * \text{}^{I_{k+1}}_{G}\hat{\bar{q}}
   * &= \exp\bigg(\frac{1}{2}\boldsymbol{\Omega}\big({\boldsymbol{\omega}}_{m,k}-\hat{\mathbf{b}}_{g,k}\big)\Delta t\bigg)
   * \text{}^{I_{k}}_{G}\hat{\bar{q}} \\
   * ^G\hat{\mathbf{v}}_{k+1} &= \text{}^G\hat{\mathbf{v}}_{I_k} - {}^G\mathbf{g}\Delta t
   * +\text{}^{I_k}_G\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t\\
   * ^G\hat{\mathbf{p}}_{I_{k+1}}
   * &= \text{}^G\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k} \Delta t
   * - \frac{1}{2}{}^G\mathbf{g}\Delta t^2
   * + \frac{1}{2} \text{}^{I_k}_{G}\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t^2
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                             const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v,
                             Eigen::Vector3d &new_p);

  /**
   * @brief RK4 imu mean propagation.
   *
   * See this wikipedia page on [Runge-Kutta Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).
   * We are doing a RK4 method, [this wolframe page](http://mathworld.wolfram.com/Runge-KuttaMethod.html) has the forth order equation
   * defined below. We define function \f$ f(t,y) \f$ where y is a function of time t, see @ref imu_kinematic for the definition of the
   * continous-time functions.
   *
   * \f{align*}{
   * {k_1} &= f({t_0}, y_0) \Delta t  \\
   * {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
   * {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
   * {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
   * y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1 \over 3}{k_3} + {1 \over 6}{k_4}} \right)
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_rk4(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                        const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v,
                        Eigen::Vector3d &new_p);

  /// Container for the noise values
  NoiseManager _noises;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;

  /// Gravity vector
  Eigen::Vector3d _gravity;
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_PROPAGATOR_H
