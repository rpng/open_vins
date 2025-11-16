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

#ifndef OV_CORE_TRACK_SIM_H
#define OV_CORE_TRACK_SIM_H

#include "TrackBase.h"

namespace ov_core {

/**
 * @brief Simulated tracker for when we already have uv measurements!
 *
 * This class should be used when we are using the @ref ov_msckf::Simulator class to generate measurements.
 * It simply takes the resulting simulation data and appends it to the internal feature database.
 */
class TrackSIM : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   */
  TrackSIM(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numaruco)
      : TrackBase(cameras, 0, numaruco, false, HistogramMethod::NONE) {}

  /**
   * @brief Process a new image
   * @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override {
    PRINT_ERROR(RED "[SIM]: SIM TRACKER FEED NEW CAMERA CALLED!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: THIS SHOULD NEVER HAPPEN!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  /**
   * @brief Feed function for a synchronized simulated cameras
   * @param timestamp Time that this image was collected
   * @param camids Camera ids that we have simulated measurements for
   * @param feats Raw uv simulated measurements
   */
  void feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                   const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_SIM_H */
