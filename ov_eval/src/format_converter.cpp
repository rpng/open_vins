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
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "utils/colors.h"
#include "utils/print.h"

/**
 * Given a CSV file this will convert it to our text file format.
 */
void process_csv(std::string infile) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Check if file paths are good
  std::ifstream file1;
  std::string line;
  file1.open(infile);
  PRINT_INFO("Opening file %s\n", boost::filesystem::path(infile).filename().c_str());

  // Check that it was successful
  if (!file1) {
    PRINT_ERROR(RED "ERROR: Unable to open input file...\n" RESET);
    PRINT_ERROR(RED "ERROR: %s\n" RESET, infile.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Loop through each line of this file
  std::vector<Eigen::VectorXd> traj_data;
  std::string current_line;
  while (std::getline(file1, current_line)) {

    // Skip if we start with a comment
    if (!current_line.find("#"))
      continue;

    // Loop variables
    int i = 0;
    std::istringstream s(current_line);
    std::string field;
    Eigen::Matrix<double, 8, 1> data;

    // Loop through this line (timestamp(ns) tx ty tz qw qx qy qz)
    while (std::getline(s, field, ',')) {
      // Skip if empty
      if (field.empty() || i >= data.rows())
        continue;
      // save the data to our vector
      data(i) = std::atof(field.c_str());
      i++;
    }

    // Only a valid line if we have all the parameters
    if (i > 7) {
      traj_data.push_back(data);
      // std::stringstream ss;
      // ss << std::setprecision(5) << data.transpose() << std::endl;
      // PRINT_DEBUG(ss.str().c_str());
    }
  }

  // Finally close the file
  file1.close();

  // Error if we don't have any data
  if (traj_data.empty()) {
    PRINT_ERROR(RED "ERROR: Could not parse any data from the file!!\n" RESET);
    PRINT_ERROR(RED "ERROR: %s\n" RESET, infile.c_str());
    std::exit(EXIT_FAILURE);
  }
  PRINT_INFO("\t- Loaded %d poses from file\n", (int)traj_data.size());

  // If file exists already then crash
  std::string outfile = infile.substr(0, infile.find_last_of('.')) + ".txt";
  if (boost::filesystem::exists(outfile)) {
    PRINT_ERROR(RED "\t- ERROR: Output file already exists, please delete and re-run this script!!\n" RESET);
    PRINT_ERROR(RED "\t- ERROR: %s\n" RESET, outfile.c_str());
    return;
  }

  // Open this file we want to write to
  std::ofstream file2;
  file2.open(outfile.c_str());
  if (file2.fail()) {
    PRINT_ERROR(RED "ERROR: Unable to open output file!!\n" RESET);
    PRINT_ERROR(RED "ERROR: %s\n" RESET, outfile.c_str());
    std::exit(EXIT_FAILURE);
  }
  file2 << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;

  // Write to disk in the correct order!
  for (size_t i = 0; i < traj_data.size(); i++) {
    file2.precision(5);
    file2.setf(std::ios::fixed, std::ios::floatfield);
    file2 << 1e-9 * traj_data.at(i)(0) << " ";
    file2.precision(6);
    file2 << traj_data.at(i)(1) << " " << traj_data.at(i)(2) << " " << traj_data.at(i)(3) << " " << traj_data.at(i)(5) << " "
          << traj_data.at(i)(6) << " " << traj_data.at(i)(7) << " " << traj_data.at(i)(4) << std::endl;
  }
  PRINT_INFO("\t- Saved to file %s\n", boost::filesystem::path(outfile).filename().c_str());

  // Finally close the file
  file2.close();
}

int main(int argc, char **argv) {

  // Ensure we have a path
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify a file to convert\n" RESET);
    PRINT_ERROR(RED "ERROR: ./format_converter <file.csv or folder\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval format_converter <file.csv or folder>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // If we do not have a wildcard, then process this one csv
  if (boost::algorithm::ends_with(argv[1], "csv")) {

    // Process this single file
    process_csv(argv[1]);

  } else {

    // Loop through this directory
    boost::filesystem::path infolder(argv[1]);
    for (auto &p : boost::filesystem::recursive_directory_iterator(infolder)) {
      if (p.path().extension() == ".csv") {
        process_csv(p.path().string());
      }
    }
  }

  // Done!
  return EXIT_SUCCESS;
}
