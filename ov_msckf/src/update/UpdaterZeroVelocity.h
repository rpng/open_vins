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

#ifndef OV_MSCKF_UPDATER_ZEROVELOCITY_H
#define OV_MSCKF_UPDATER_ZEROVELOCITY_H

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

namespace ov_msckf {

/**
 * @brief Will try to *detect* and then update using zero velocity assumption.
 *
 * Consider the case that a VIO unit remains stationary for a period time.
 * Typically this can cause issues in a monocular system without SLAM features since no features can be triangulated.
 * Additional, if features could be triangulated (e.g. stereo) the quality can be poor and hurt performance.
 * If we can detect the cases where we are stationary then we can leverage this to prevent the need to do feature update during this period.
 * The main application would be using this on a **wheeled vehicle** which needs to stop (e.g. stop lights or parking).
 */
class UpdaterZeroVelocity {

public:
  /**
   * @brief Default constructor for our zero velocity detector and updater.
   * @param options Updater options (chi2 multiplier)
   * @param noises imu noise characteristics (continuous time)
   * @param db Feature tracker database with all features in it
   * @param prop Propagator class object which can predict the state forward in time
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   * @param zupt_max_velocity Max velocity we should consider to do a update with
   * @param zupt_noise_multiplier Multiplier of our IMU noise matrix (default should be 1.0)
   * @param zupt_max_disparity Max disparity we should consider to do a update with
   */
  UpdaterZeroVelocity(UpdaterOptions &options, Propagator::NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                      std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity, double zupt_noise_multiplier,
                      double zupt_max_disparity)
      : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
        _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity) {

    // Gravity
    _gravity << 0.0, 0.0, gravity_mag;

    // Save our raw pixel noise squared
    _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
    _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
    _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
    _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

    // Initialize the chi squared test table with confidence level 0.95
    // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
    for (int i = 1; i < 1000; i++) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
  }

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   */
  void feed_imu(const ov_core::ImuData &message) {

    // Append it to our vector
    imu_data.emplace_back(message);

    // Sort our imu data (handles any out of order measurements)
    // std::sort(imu_data.begin(), imu_data.end(), [](const IMUDATA i, const IMUDATA j) {
    //    return i.timestamp < j.timestamp;
    //});

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
   * @brief Will first detect if the system is zero velocity, then will update.
   * @param state State of the filter
   * @param timestamp Next camera timestamp we want to see if we should propagate to.
   * @return True if the system is currently at zero velocity
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:
  /// Options used during update (chi2 multiplier)
  UpdaterOptions _options;

  /// Container for the imu noise values
  Propagator::NoiseManager _noises;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our propagator!
  std::shared_ptr<Propagator> _prop;

  /// Gravity vector
  Eigen::Vector3d _gravity;

  /// Max velocity (m/s) that we should consider a zupt with
  double _zupt_max_velocity = 1.0;

  /// Multiplier of our IMU noise matrix (default should be 1.0)
  double _zupt_noise_multiplier = 1.0;

  /// Max disparity (pixels) that we should consider a zupt with
  double _zupt_max_disparity = 1.0;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  /// Last timestamp we did zero velocity update with
  double last_zupt_state_timestamp = 0.0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_ZEROVELOCITY_H
