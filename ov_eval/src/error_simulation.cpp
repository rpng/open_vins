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



#include "calc/ResultSimulation.h"
#include "utils/Colors.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  // Ensure we have a path
  if (argc < 4) {
    printf(RED "ERROR: ./error_simulation <file_est.txt> <file_std.txt> <file_gt.txt>\n" RESET);
    printf(RED "ERROR: rosrun ov_eval error_simulation <file_est.txt> <file_std.txt> <file_gt.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Create our trajectory object
  ov_eval::ResultSimulation traj(argv[1], argv[2], argv[3]);

  // Plot the state errors
  printf("Plotting state variable errors...\n");
  traj.plot_state(true);

  // Plot time offset
  printf("Plotting time offset error...\n");
  traj.plot_timeoff(true, 10);

  // Plot camera intrinsics
  printf("Plotting camera intrinsics...\n");
  traj.plot_cam_instrinsics(true, 60);

  // Plot camera extrinsics
  printf("Plotting camera extrinsics...\n");
  traj.plot_cam_extrinsics(true, 60);

#ifdef HAVE_PYTHONLIBS
  matplotlibcpp::show(true);
#endif

  // Done!
  return EXIT_SUCCESS;
}
