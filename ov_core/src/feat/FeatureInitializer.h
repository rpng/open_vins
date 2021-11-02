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

#ifndef OPEN_VINS_FEATUREINITIALIZER_H
#define OPEN_VINS_FEATUREINITIALIZER_H

#include <unordered_map>

#include "Feature.h"
#include "FeatureInitializerOptions.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_core {

/**
 * @brief Class that triangulates feature
 *
 * This class has the functions needed to triangulate and then refine a given 3D feature.
 * As in the standard MSCKF, we know the clones of the camera from propagation and past updates.
 * Thus, we just need to triangulate a feature in 3D with the known poses and then refine it.
 * One should first call the single_triangulation() function afterwhich single_gaussnewton() allows for refinement.
 * Please see the @ref update-featinit page for detailed derivations.
 */
class FeatureInitializer {

public:
  /**
   * @brief Structure which stores pose estimates for use in triangulation
   *
   * - R_GtoC - rotation from global to camera
   * - p_CinG - position of camera in global frame
   */
  struct ClonePose {

    /// Rotation
    Eigen::Matrix<double, 3, 3> _Rot;

    /// Position
    Eigen::Matrix<double, 3, 1> _pos;

    /// Constructs pose from rotation and position
    ClonePose(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &p) {
      _Rot = R;
      _pos = p;
    }

    /// Constructs pose from quaternion and position
    ClonePose(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &p) {
      _Rot = quat_2_Rot(q);
      _pos = p;
    }

    /// Default constructor
    ClonePose() {
      _Rot = Eigen::Matrix<double, 3, 3>::Identity();
      _pos = Eigen::Matrix<double, 3, 1>::Zero();
    }

    /// Accessor for rotation
    const Eigen::Matrix<double, 3, 3> &Rot() { return _Rot; }

    /// Accessor for position
    const Eigen::Matrix<double, 3, 1> &pos() { return _pos; }
  };

  /**
   * @brief Default constructor
   * @param options Options for the initializer
   */
  FeatureInitializer(FeatureInitializerOptions &options) : _options(options) {}

  /**
   * @brief Uses a linear triangulation to get initial estimate for the feature
   *
   * The derivations for this method can be found in the @ref featinit-linear documentation page.
   *
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to triangulate (based on the thresholds)
   */
  bool single_triangulation(Feature *feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief Uses a linear triangulation to get initial estimate for the feature, treating the anchor observation as a true bearing.
   *
   * The derivations for this method can be found in the @ref featinit-linear-1d documentation page.
   * This function should be used if you want speed, or know your anchor bearing is reasonably accurate.
   *
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to triangulate (based on the thresholds)
   */
  bool single_triangulation_1d(Feature *feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief Uses a nonlinear triangulation to refine initial linear estimate of the feature
   * @param feat Pointer to feature
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera
   * in global frame)
   * @return Returns false if it fails to be optimize (based on the thresholds)
   */
  bool single_gaussnewton(Feature *feat, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);

  /**
   * @brief Gets the current configuration of the feature initializer
   * @return Const feature initializer config
   */
  const FeatureInitializerOptions config() { return _options; }

protected:
  /// Contains options for the initializer process
  FeatureInitializerOptions _options;

  /**
   * @brief Helper function for the gauss newton method that computes error of the given estimate
   * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
   * @param feat Pointer to the feature
   * @param alpha x/z in anchor
   * @param beta y/z in anchor
   * @param rho 1/z inverse depth
   */
  double compute_error(std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM, Feature *feat, double alpha,
                       double beta, double rho);
};

} // namespace ov_core

#endif // OPEN_VINS_FEATUREINITIALIZER_H