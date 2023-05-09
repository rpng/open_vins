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

#ifndef OV_INIT_DYNAMICINITIALIZER_H
#define OV_INIT_DYNAMICINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
class PoseJPL;
class Landmark;
class Vec;
} // namespace ov_type

namespace ov_init {

/**
 * @brief Initializer for a dynamic visual-inertial system.
 *
 * This implementation that will try to recover the initial conditions of the system.
 * Additionally, we will try to recover the covariance of the system.
 * To initialize with arbitrary motion:
 * 1. Preintegrate our system to get the relative rotation change (biases assumed known)
 * 2. Construct linear system with features to recover velocity (solve with |g| constraint)
 * 3. Perform a large MLE with all calibration and recover the covariance.
 *
 * Method is based on this work (see this [tech report](https://pgeneva.com/downloads/reports/tr_init.pdf) for a high level walk through):
 *
 * > Dong-Si, Tue-Cuong, and Anastasios I. Mourikis.
 * > "Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration."
 * > 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
 *
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 *
 */
class DynamicInitializer {
public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param imu_data_ Shared pointer to our IMU vector of historical information
   */
  explicit DynamicInitializer(const InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                              std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief Try to get the initialized system
   *
   * @param[out] timestamp Timestamp we have initialized the state at (last imu state)
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param _imu Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
   * @param _clones_IMU Map between imaging times and clone poses (q_GtoIi, p_IiinG)
   * @param _features_SLAM Our current set of SLAM features (3d positions)
   * @return True if we have successfully initialized our system
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM);

private:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_DYNAMICINITIALIZER_H
