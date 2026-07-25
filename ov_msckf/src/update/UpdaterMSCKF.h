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

#include <opencv2/core.hpp>

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
   * @param state        State of the filter
   * @param feature_vec  Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Configure the IMU-residual per-feature noise inflation.
   *
   * Projects each feature's p_FinG into cam0 and computes a reprojection residual
   * used to inflate MSCKF measurement noise for dynamic features (nm > 1).
   *
   * Two modes selectable via use_variance:
   *   false (default) — single-frame: residual at the newest clone only.
   *   true  (variance) — std-dev of residuals across all sliding-window clones.
   *     Bias-invariant: triangulation error adds a constant to every r_k and
   *     cancels in variance. Only features that actually moved show high std_r.
   *     Falls back to nm=1 when fewer than 3 valid clone observations exist.
   *
   * @param use          Enable/disable
   * @param alpha        Scale: nm = 1 + alpha*(1 - exp(-signal/sigma_px))
   * @param sigma_px     Decay constant in pixels
   * @param use_variance Use cross-clone residual std-dev instead of single-frame residual
   * @param max_depth    Features beyond this depth (m) skip nm entirely (0 = disabled).
   *                     Suppresses false inflation on poorly-triangulated far features.
   *                     Stereo disparity at max_depth ≈ baseline*fx/max_depth.
   */
  void set_imu_residual_params(bool use, double alpha, double sigma_px,
                               bool use_variance = false, double max_depth = 0.0,
                               double max_tri_error = 0.0) {
    _use_imu_residual = use;
    _imu_residual_alpha = alpha;
    _imu_residual_sigma_px = sigma_px;
    _use_residual_variance = use_variance;
    _imu_residual_max_depth = max_depth;
    _imu_residual_max_tri_error = max_tri_error;
  }

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  // IMU-residual noise inflation parameters
  bool _use_imu_residual = false;
  double _imu_residual_alpha = 5.0;
  double _imu_residual_sigma_px = 5.0;
  bool _use_residual_variance = false;        // use cross-clone residual std-dev instead of single-frame
  double _imu_residual_max_depth = 0.0;      // depth gate in metres (0 = disabled)
  double _imu_residual_max_tri_error = 0.0;  // triangulation quality gate: RMS reprojection px (0 = disabled)

};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
