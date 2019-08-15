/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
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


namespace ov_core {


    /**
     * @brief Struct which stores all our feature initializer options
     */
    struct FeatureInitializerOptions {

        /// Max runs for Gauss Newton
        int max_runs = 20;

        /// Init lambda for LM optimization
        double init_lamda = 1e-3;

        /// Max lambda for LM optimization
        double max_lamda = 1e10;

        /// Cutoff for dx increment to consider as converged
        double min_dx = 1e-6;

        /// Cutoff for cost decrement to consider as converged
        double min_dcost = 1e-6;

        /// Multiplier to increase/decrease lambda
        double lam_mult = 10;

        /// Minimum distance to accept triangulated features
        double min_dist = 0.25;

        /// Minimum distance to accept triangulated features
        double max_dist = 40;

        /// Max baseline ratio to accept triangulated features
        double max_baseline = 40;

        /// Max condition number of linear triangulation matrix accept triangulated features
        double max_cond_number = 1000;


    };

}

#endif //OV_CORE_INITIALIZEROPTIONS_H