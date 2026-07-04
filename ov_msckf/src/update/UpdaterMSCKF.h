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

#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core

namespace ov_msckf {

class State;

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
 * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
 * After this we compress all the measurements to have an efficient update and update the state.
 */
class UpdaterMSCKF {

public:
  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Configure the IMU-residual per-feature noise inflation (Phase 1, Option C).
   *
   * When enabled, after triangulation each MSCKF feature's p_FinG is projected
   * into cam0 at the newest clone time and compared to the actual observation.
   * Dynamic features (large projection residual) get noise multiplier nm >> 1.
   *
   * @param use       Enable/disable the computation
   * @param alpha     Multiplier scale: nm = 1 + alpha*(1 - s_imu)
   * @param sigma_px  Decay scale for the exponential score (pixels, default 5.0)
   */
  void set_imu_residual_params(bool use, double alpha, double sigma_px) {
    _use_imu_residual = use;
    _imu_residual_alpha = alpha;
    _imu_residual_sigma_px = sigma_px;
  }

  /**
   * @brief Configure the per-feature CSV logger for JEPA dataset construction (Phase 2).
   *
   * When enabled, after each feature's nm and IMU projection are computed, a row is
   * appended to the CSV at @p path. Disabled by default — no file I/O when false.
   *
   * CSV columns:
   *   ts, feat_id, u_act, v_act, u_pred, v_pred, nm, r_px, depth,
   *   track_len, t_prev, u_prev, v_prev, dangle_x, dangle_y, dangle_z, dt
   *
   * @param enable  Turn logging on/off
   * @param path    Full output path (run number must be in the filename, e.g. feature_log_run_3.csv)
   */
  void set_feature_logger_params(bool enable, const std::string &path);

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  // IMU-residual noise inflation parameters (Phase 1, Option C)
  bool _use_imu_residual = false;
  double _imu_residual_alpha = 5.0;
  double _imu_residual_sigma_px = 5.0;

  // Feature logger (Phase 2 dataset collection — disabled by default)
  bool _log_features = false;
  std::ofstream _feat_log_file;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
