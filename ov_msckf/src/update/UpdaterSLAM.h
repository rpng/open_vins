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


#ifndef OV_MSCKF_UPDATER_SLAM_H
#define OV_MSCKF_UPDATER_SLAM_H

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"
#include <Eigen/Eigen>

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

namespace ov_msckf {

/**
 * @brief Will compute the system for our sparse SLAM features and update the filter.
 *
 * This class is responsible for performing delayed feature initialization, SLAM update, and
 * SLAM anchor change for anchored feature representations.
 */
class UpdaterSLAM {

public:
  /**
   * @brief Default constructor for our SLAM updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options_slam Updater options (include measurement noise value) for SLAM features
   * @param options_aruco Updater options (include measurement noise value) for ARUCO features
   * @param feat_init_options Feature initializer options
   */
  UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco, FeatureInitializerOptions &feat_init_options)
      : _options_slam(options_slam), _options_aruco(options_aruco) {

    // Save our raw pixel noise squared
    _options_slam.sigma_pix_sq = std::pow(_options_slam.sigma_pix, 2);
    _options_aruco.sigma_pix_sq = std::pow(_options_aruco.sigma_pix, 2);

    // Save our feature initializer
    initializer_feat = std::unique_ptr<FeatureInitializer>(new FeatureInitializer(feat_init_options));

    // Initialize the chi squared test table with confidence level 0.95
    // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
    for (int i = 1; i < 500; i++) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
  }

  /**
   * @brief Given tracked SLAM features, this will try to use them to update the state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec);

  /**
   * @brief Given max track features, this will try to use them to initialize them in the state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void delayed_init(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec);

  /**
   * @brief Will change SLAM feature anchors if it will be marginalized
   *
   * Makes sure that if any clone is about to be marginalized, it changes anchor representation.
   * By default, this will shift the anchor into the newest IMU clone and keep the camera calibration anchor the same.
   *
   * @param state State of the filter
   */
  void change_anchors(std::shared_ptr<State> state);

protected:
  /**
   * @brief Shifts landmark anchor to new clone
   * @param state State of filter
   * @param landmark landmark whose anchor is being shifter
   * @param new_anchor_timestamp Clone timestamp we want to move to
   * @param new_cam_id Which camera frame we want to move to
   */
  void perform_anchor_change(std::shared_ptr<State> state, std::shared_ptr<Landmark> landmark, double new_anchor_timestamp,
                             size_t new_cam_id);

  /// Options used during update for slam features
  UpdaterOptions _options_slam;

  /// Options used during update for aruco features
  UpdaterOptions _options_aruco;

  /// Feature initializer class object
  std::unique_ptr<FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_SLAM_H
