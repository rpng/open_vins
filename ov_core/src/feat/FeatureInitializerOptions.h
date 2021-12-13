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

#ifndef OV_CORE_INITIALIZEROPTIONS_H
#define OV_CORE_INITIALIZEROPTIONS_H

#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Struct which stores all our feature initializer options
 */
struct FeatureInitializerOptions {

  /// If we should perform 1d triangulation instead of 3d
  bool triangulate_1d = false;

  /// If we should perform Levenberg-Marquardt refinment
  bool refine_features = true;

  /// Max runs for Levenberg-Marquardt
  int max_runs = 5;

  /// Init lambda for Levenberg-Marquardt optimization
  double init_lamda = 1e-3;

  /// Max lambda for Levenberg-Marquardt optimization
  double max_lamda = 1e10;

  /// Cutoff for dx increment to consider as converged
  double min_dx = 1e-6;

  /// Cutoff for cost decrement to consider as converged
  double min_dcost = 1e-6;

  /// Multiplier to increase/decrease lambda
  double lam_mult = 10;

  /// Minimum distance to accept triangulated features
  double min_dist = 0.10;

  /// Minimum distance to accept triangulated features
  double max_dist = 60;

  /// Max baseline ratio to accept triangulated features
  double max_baseline = 40;

  /// Max condition number of linear triangulation matrix accept triangulated features
  double max_cond_number = 10000;

  /// Nice print function of what parameters we have loaded
  void print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("fi_triangulate_1d", triangulate_1d, false);
      parser->parse_config("fi_refine_features", refine_features, false);
      parser->parse_config("fi_max_runs", max_runs, false);
      parser->parse_config("fi_init_lamda", init_lamda, false);
      parser->parse_config("fi_max_lamda", max_lamda, false);
      parser->parse_config("fi_min_dx", min_dx, false);
      parser->parse_config("fi_min_dcost", min_dcost, false);
      parser->parse_config("fi_lam_mult", lam_mult, false);
      parser->parse_config("fi_min_dist", min_dist, false);
      parser->parse_config("fi_max_dist", max_dist, false);
      parser->parse_config("fi_max_baseline", max_baseline, false);
      parser->parse_config("fi_max_cond_number", max_cond_number, false);
    }
    PRINT_DEBUG("\t- triangulate_1d: %d\n", triangulate_1d);
    PRINT_DEBUG("\t- refine_features: %d\n", refine_features);
    PRINT_DEBUG("\t- max_runs: %d\n", max_runs);
    PRINT_DEBUG("\t- init_lamda: %.3f\n", init_lamda);
    PRINT_DEBUG("\t- max_lamda: %.3f\n", max_lamda);
    PRINT_DEBUG("\t- min_dx: %.7f\n", min_dx);
    PRINT_DEBUG("\t- min_dcost: %.7f\n", min_dcost);
    PRINT_DEBUG("\t- lam_mult: %.3f\n", lam_mult);
    PRINT_DEBUG("\t- min_dist: %.3f\n", min_dist);
    PRINT_DEBUG("\t- max_dist: %.3f\n", max_dist);
    PRINT_DEBUG("\t- max_baseline: %.3f\n", max_baseline);
    PRINT_DEBUG("\t- max_cond_number: %.3f\n", max_cond_number);
  }
};

} // namespace ov_core

#endif // OV_CORE_INITIALIZEROPTIONS_H