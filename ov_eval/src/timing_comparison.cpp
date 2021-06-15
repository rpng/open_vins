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

#include "utils/Colors.h"
#include "utils/Loader.h"
#include "utils/Statistics.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  // Ensure we have a path
  if (argc < 2) {
    printf(RED "ERROR: Please specify a timing file\n" RESET);
    printf(RED "ERROR: ./timing_comparison <file_times1.txt> ... <file_timesN.txt>\n" RESET);
    printf(RED "ERROR: rosrun ov_eval timing_comparison <file_times1.txt> ... <file_timesN.txt>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Read in all our trajectories from file
  std::vector<std::string> names;
  std::vector<ov_eval::Statistics> total_times;
  for (int z = 1; z < argc; z++) {

    // Parse the name of this timing
    boost::filesystem::path path(argv[z]);
    std::string name = path.stem().string();
    printf("======================================\n");
    printf("[TIME]: loading data for %s\n", name.c_str());

    // Load it!!
    std::vector<std::string> names_temp;
    std::vector<double> times;
    std::vector<Eigen::VectorXd> timing_values;
    ov_eval::Loader::load_timing_flamegraph(argv[z], names_temp, times, timing_values);
    printf("[TIME]: loaded %d timestamps from file (%d categories)!!\n", (int)times.size(), (int)names_temp.size());

    // Our categories
    std::vector<ov_eval::Statistics> stats;
    for (size_t i = 0; i < names_temp.size(); i++)
      stats.push_back(ov_eval::Statistics());

    // Loop through each and report the average timing information
    for (size_t i = 0; i < times.size(); i++) {
      for (size_t c = 0; c < names_temp.size(); c++) {
        stats.at(c).timestamps.push_back(times.at(i));
        stats.at(c).values.push_back(timing_values.at(i)(c));
      }
    }

    // Now print the statistic for this run
    for (size_t i = 0; i < names_temp.size(); i++) {
      stats.at(i).calculate();
      printf("mean_time = %.4f | std = %.4f | 99th = %.4f  | max = %.4f (%s)\n", stats.at(i).mean, stats.at(i).std, stats.at(i).ninetynine,
             stats.at(i).max, names_temp.at(i).c_str());
    }

    // Append the total stats to the big vector
    if (!stats.empty()) {
      names.push_back(name);
      total_times.push_back(stats.at(stats.size() - 1));
    } else {
      printf(RED "[TIME]: unable to load any data.....\n" RESET);
    }
    printf("======================================\n");
  }

#ifdef HAVE_PYTHONLIBS

  // Valid colors
  // https://matplotlib.org/tutorials/colors/colors.html
  // std::vector<std::string> colors = {"blue","aqua","lightblue","lightgreen","yellowgreen","green"};
  // std::vector<std::string> colors = {"navy","blue","lightgreen","green","gold","goldenrod"};
  std::vector<std::string> colors = {"black", "blue", "red", "green", "cyan", "magenta"};

  // Plot this figure
  matplotlibcpp::figure_size(1200, 400);

  // Zero our time arrays
  double starttime = (total_times.at(0).timestamps.empty()) ? 0 : total_times.at(0).timestamps.at(0);
  double endtime = (total_times.at(0).timestamps.empty()) ? 0 : total_times.at(0).timestamps.at(total_times.at(0).timestamps.size() - 1);
  for (size_t i = 0; i < total_times.size(); i++) {
    for (size_t j = 0; j < total_times.at(i).timestamps.size(); j++) {
      total_times.at(i).timestamps.at(j) -= starttime;
    }
  }

  // Now loop through each and plot it!
  for (size_t n = 0; n < names.size(); n++) {

    // Sub-sample the time and values
    int keep_every = 10;
    std::vector<double> times_skipped;
    for (size_t t = 0; t < total_times.at(n).timestamps.size(); t++) {
      if (t % keep_every == 0) {
        times_skipped.push_back(total_times.at(n).timestamps.at(t));
      }
    }
    std::vector<double> values_skipped;
    for (size_t t = 0; t < total_times.at(n).values.size(); t++) {
      if (t % keep_every == 0) {
        values_skipped.push_back(total_times.at(n).values.at(t));
      }
    }

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", names.at(n)});
    params.insert({"linestyle", "-"});
    params.insert({"color", colors.at(n % colors.size())});

    // Finally plot
    matplotlibcpp::plot(times_skipped, values_skipped, params);
  }

  // Finally add labels and show it
  matplotlibcpp::ylabel("execution time (s)");
  matplotlibcpp::xlim(0.0, endtime - starttime);
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::legend();
  matplotlibcpp::tight_layout();

  // Display to the user
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
