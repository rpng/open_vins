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

#include <Eigen/Eigen>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "utils/Loader.h"
#include "utils/Statistics.h"
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
  if (argc < 3) {
    PRINT_ERROR(RED "ERROR: Please specify a timing file\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_histagram <file_times.txt> <num_bins>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_flamegraph <file_times.txt> <num_bins>\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  int nbins = atoi(argv[2]);

  // Load it!!
  std::vector<std::string> names;
  std::vector<double> times;
  std::vector<Eigen::VectorXd> timing_values;
  ov_eval::Loader::load_timing_flamegraph(argv[1], names, times, timing_values);
  PRINT_INFO("[TIME]: loaded %d timestamps from file (%d categories)!!\n", (int)times.size(), (int)names.size());

  // Our categories
  std::vector<ov_eval::Statistics> stats;
  for (size_t i = 0; i < names.size(); i++)
    stats.push_back(ov_eval::Statistics());

  // Loop through each and report the average timing information
  for (size_t i = 0; i < times.size(); i++) {
    for (size_t c = 0; c < names.size(); c++) {
      stats.at(c).timestamps.push_back(times.at(i));
      stats.at(c).values.push_back(1000.0 * timing_values.at(i)(c));
    }
  }

#ifdef HAVE_PYTHONLIBS

  // Valid colors
  // https://matplotlib.org/stable/tutorials/colors/colors.html
  // std::vector<std::string> colors_valid = {"blue","aqua","lightblue","lightgreen","yellowgreen","green"};
  std::vector<std::string> colors_valid = {"navy", "blue", "lightgreen", "green", "orange", "goldenrod", "red", "pink", "black"};

  // Plot each histogram
  for (size_t i = 0; i < names.size(); i++) {
    matplotlibcpp::figure_size(500, 300);
    matplotlibcpp::hist(stats.at(i).values, nbins, colors_valid.at(i % colors_valid.size()));
    matplotlibcpp::ylabel(names.at(i));
    matplotlibcpp::xlabel("execution time (ms)");
    matplotlibcpp::tight_layout();
  }

  // Display to the user
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
