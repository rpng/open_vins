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
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   */
  TrackSIM(int numaruco) : TrackBase(0, numaruco) {}

  /**
   * @brief Set the width and height for the cameras
   * @param _camera_wh Width and height for each camera
   */
  void set_width_height(std::map<size_t, std::pair<int, int>> _camera_wh) { this->camera_wh = _camera_wh; }

  /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
  void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) override {
    printf(RED "[SIM]: SIM TRACKER FEED MONOCULAR CALLED!!!\n" RESET);
    printf(RED "[SIM]: THIS SHOULD NEVER HAPPEN!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
  void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) override {
    printf(RED "[SIM]: SIM TRACKER FEED STEREO CALLED!!!\n" RESET);
    printf(RED "[SIM]: THIS SHOULD NEVER HAPPEN!\n" RESET);
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

protected:
  /// Width and height of our cameras
  std::map<size_t, std::pair<int, int>> camera_wh;
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_SIM_H */
