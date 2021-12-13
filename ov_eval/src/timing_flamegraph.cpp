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
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify a timing file\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_flamegraph <file_times.txt>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_flamegraph <file_times.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

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
      stats.at(c).values.push_back(timing_values.at(i)(c));
    }
  }

  // Now print the statistic for this run
  for (size_t i = 0; i < names.size(); i++) {
    stats.at(i).calculate();
    PRINT_INFO("mean_time = %.4f | std = %.4f | 99th = %.4f  | max = %.4f (%s)\n", stats.at(i).mean, stats.at(i).std,
               stats.at(i).ninetynine, stats.at(i).max, names.at(i).c_str());
  }

#ifdef HAVE_PYTHONLIBS

  // Sub-sample the time
  int keep_every = 10;
  std::vector<double> times_skipped;
  for (size_t t = 0; t < times.size(); t++) {
    if (t % keep_every == 0) {
      times_skipped.push_back(times.at(t));
    }
  }

  // Zero our time arrays
  double starttime1 = (times_skipped.empty()) ? 0 : times_skipped.at(0);
  double endtime1 = (times_skipped.empty()) ? 0 : times_skipped.at(times_skipped.size() - 1);
  for (size_t j = 0; j < times_skipped.size(); j++) {
    times_skipped.at(j) -= starttime1;
  }

  // Valid colors
  // https://matplotlib.org/tutorials/colors/colors.html
  // std::vector<std::string> colors_valid = {"blue","aqua","lightblue","lightgreen","yellowgreen","green"};
  std::vector<std::string> colors_valid = {"navy", "blue", "lightgreen", "green", "gold", "goldenrod"};

  // Create vector for each category
  // NOTE we skip the last category since it is the "total" time by convention
  std::vector<std::string> labels;
  std::vector<std::string> colors;
  std::vector<std::vector<double>> timings;
  for (size_t i = 0; i < names.size() - 1; i++) {
    labels.push_back(names.at(i));
    colors.push_back(colors_valid.at(i % colors_valid.size()));
    std::vector<double> values_skipped;
    for (size_t t = 0; t < stats.at(i).values.size(); t++) {
      if (t % keep_every == 0) {
        values_skipped.push_back(stats.at(i).values.at(t));
      }
    }
    timings.push_back(values_skipped);
  }

  // Plot this figure
  matplotlibcpp::figure_size(1200, 400);
  matplotlibcpp::stackplot(times_skipped, timings, labels, colors, "zero");
  matplotlibcpp::ylabel("execution time (s)");
  matplotlibcpp::xlim(0.0, endtime1 - starttime1);
  // matplotlibcpp::ylim(0.0,stats.at(stats.size()-1).ninetynine);
  matplotlibcpp::ylim(0.0, stats.at(stats.size() - 1).max);
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::legend();
  matplotlibcpp::tight_layout();

  // Display to the user
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
