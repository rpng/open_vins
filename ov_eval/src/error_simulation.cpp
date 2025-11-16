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

#include "calc/ResultSimulation.h"
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have a path
  if (argc < 4) {
    PRINT_ERROR(RED "ERROR: ./error_simulation <file_est.txt> <file_std.txt> <file_gt.txt>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval error_simulation <file_est.txt> <file_std.txt> <file_gt.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Create our trajectory object
  ov_eval::ResultSimulation traj(argv[1], argv[2], argv[3]);

  // Plot the state errors
  PRINT_INFO("Plotting state variable errors...\n");
  traj.plot_state(true);

  // Plot time offset
  PRINT_INFO("Plotting time offset error...\n");
  traj.plot_timeoff(true, 10);

  // Plot camera intrinsics
  PRINT_INFO("Plotting camera intrinsics...\n");
  traj.plot_cam_instrinsics(true, 60);

  // Plot camera extrinsics
  PRINT_INFO("Plotting camera extrinsics...\n");
  traj.plot_cam_extrinsics(true, 60);

  // Plot IMU intrinsics
  PRINT_INFO("Plotting IMU intrinsics...\n");
  traj.plot_imu_intrinsics(true, 60);

#ifdef HAVE_PYTHONLIBS
  matplotlibcpp::show(true);
#endif

  // Done!
  return EXIT_SUCCESS;
}
