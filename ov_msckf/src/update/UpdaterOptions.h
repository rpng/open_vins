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

#ifndef OV_MSCKF_UPDATER_OPTIONS_H
#define OV_MSCKF_UPDATER_OPTIONS_H

#include "utils/print.h"

namespace ov_msckf {

/**
 * @brief Struct which stores general updater options
 */
struct UpdaterOptions {

  /// What chi-squared multipler we should apply
  double chi2_multipler = 5;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  /// Covariance for our raw pixel measurements
  double sigma_pix_sq = 1;

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("    - chi2_multipler: %.1f\n", chi2_multipler);
    PRINT_DEBUG("    - sigma_pix: %.2f\n", sigma_pix);
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_OPTIONS_H