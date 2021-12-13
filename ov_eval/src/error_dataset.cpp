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
#include <boost/filesystem.hpp>
#include <string>

#include "calc/ResultTrajectory.h"
#include "utils/Loader.h"
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
    PRINT_ERROR(RED "ERROR: Please specify a align mode, folder, and algorithms\n" RESET);
    PRINT_ERROR(RED "ERROR: ./error_dataset <align_mode> <file_gt.txt> <folder_algorithms>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval error_dataset <align_mode> <file_gt.txt> <folder_algorithms>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Load it!
  boost::filesystem::path path_gt(argv[2]);
  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 7, 1>> poses;
  std::vector<Eigen::Matrix3d> cov_ori, cov_pos;
  ov_eval::Loader::load_data(argv[2], times, poses, cov_ori, cov_pos);
  // Print its length and stats
  double length = ov_eval::Loader::get_total_length(poses);
  PRINT_DEBUG("[COMP]: %d poses in %s => length of %.2f meters\n", (int)times.size(), path_gt.stem().string().c_str(), length);

  // Get the algorithms we will process
  // Also create empty statistic objects for each of our datasets
  std::string path_algos(argv[3]);
  std::vector<boost::filesystem::path> path_algorithms;
  for (const auto &entry : boost::filesystem::directory_iterator(path_algos)) {
    if (boost::filesystem::is_directory(entry)) {
      path_algorithms.push_back(entry.path());
    }
  }
  std::sort(path_algorithms.begin(), path_algorithms.end());

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Relative pose error segment lengths
  // std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
  std::vector<double> segments = {7.0, 14.0, 21.0, 28.0, 35.0};

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Loop through each algorithm type
  for (size_t i = 0; i < path_algorithms.size(); i++) {

    // Debug print
    PRINT_INFO("======================================\n");
    PRINT_INFO("[COMP]: processing %s algorithm\n", path_algorithms.at(i).filename().c_str());

    // Get the list of datasets this algorithm records
    std::map<std::string, boost::filesystem::path> path_algo_datasets;
    for (auto &entry : boost::filesystem::directory_iterator(path_algorithms.at(i))) {
      if (boost::filesystem::is_directory(entry)) {
        path_algo_datasets.insert({entry.path().filename().string(), entry.path()});
      }
    }

    // Check if we have runs for our dataset
    if (path_algo_datasets.find(path_gt.stem().string()) == path_algo_datasets.end()) {
      PRINT_ERROR(RED "[COMP]: %s dataset does not have any runs for %s!!!!!\n" RESET, path_algorithms.at(i).filename().c_str(),
                  path_gt.stem().c_str());
      continue;
    }

    // Errors for this specific dataset (i.e. our averages over the total runs)
    ov_eval::Statistics ate_dataset_ori, ate_dataset_pos;
    ov_eval::Statistics ate_2d_dataset_ori, ate_2d_dataset_pos;
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> rpe_dataset;
    for (const auto &len : segments) {
      rpe_dataset.insert({len, {ov_eval::Statistics(), ov_eval::Statistics()}});
    }
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> rmse_dataset;
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> rmse_2d_dataset;
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> nees_dataset;

    // Loop though the different runs for this dataset
    std::vector<std::string> file_paths;
    for (auto &entry : boost::filesystem::directory_iterator(path_algo_datasets.at(path_gt.stem().string()))) {
      if (entry.path().extension() != ".txt")
        continue;
      file_paths.push_back(entry.path().string());
    }
    std::sort(file_paths.begin(), file_paths.end());

    // Check if we have runs
    if (file_paths.empty()) {
      PRINT_ERROR(RED "\tERROR: No runs found for %s, is the folder structure right??\n" RESET, path_algorithms.at(i).filename().c_str());
      continue;
    }

    // Loop though the different runs for this dataset
    for (auto &path_esttxt : file_paths) {

      // Create our trajectory object
      std::string path_gttxt = path_gt.string();
      ov_eval::ResultTrajectory traj(path_esttxt, path_gttxt, argv[1]);

      // Calculate ATE error for this dataset
      ov_eval::Statistics error_ori, error_pos;
      traj.calculate_ate(error_ori, error_pos);
      ate_dataset_ori.values.push_back(error_ori.rmse);
      ate_dataset_pos.values.push_back(error_pos.rmse);
      for (size_t j = 0; j < error_ori.values.size(); j++) {
        rmse_dataset[error_ori.timestamps.at(j)].first.values.push_back(error_ori.values.at(j));
        rmse_dataset[error_pos.timestamps.at(j)].second.values.push_back(error_pos.values.at(j));
        assert(error_ori.timestamps.at(j) == error_pos.timestamps.at(j));
      }

      // Calculate ATE 2D error for this dataset
      ov_eval::Statistics error_ori_2d, error_pos_2d;
      traj.calculate_ate_2d(error_ori_2d, error_pos_2d);
      ate_2d_dataset_ori.values.push_back(error_ori_2d.rmse);
      ate_2d_dataset_pos.values.push_back(error_pos_2d.rmse);
      for (size_t j = 0; j < error_ori_2d.values.size(); j++) {
        rmse_2d_dataset[error_ori_2d.timestamps.at(j)].first.values.push_back(error_ori_2d.values.at(j));
        rmse_2d_dataset[error_pos_2d.timestamps.at(j)].second.values.push_back(error_pos_2d.values.at(j));
        assert(error_ori_2d.timestamps.at(j) == error_pos_2d.timestamps.at(j));
      }

      // NEES error for this dataset
      ov_eval::Statistics nees_ori, nees_pos;
      traj.calculate_nees(nees_ori, nees_pos);
      for (size_t j = 0; j < nees_ori.values.size(); j++) {
        nees_dataset[nees_ori.timestamps.at(j)].first.values.push_back(nees_ori.values.at(j));
        nees_dataset[nees_ori.timestamps.at(j)].second.values.push_back(nees_pos.values.at(j));
        assert(nees_ori.timestamps.at(j) == nees_pos.timestamps.at(j));
      }

      // Calculate RPE error for this dataset
      std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> error_rpe;
      traj.calculate_rpe(segments, error_rpe);
      for (const auto &elm : error_rpe) {
        rpe_dataset.at(elm.first).first.values.insert(rpe_dataset.at(elm.first).first.values.end(), elm.second.first.values.begin(),
                                                      elm.second.first.values.end());
        rpe_dataset.at(elm.first).first.timestamps.insert(rpe_dataset.at(elm.first).first.timestamps.end(),
                                                          elm.second.first.timestamps.begin(), elm.second.first.timestamps.end());
        rpe_dataset.at(elm.first).second.values.insert(rpe_dataset.at(elm.first).second.values.end(), elm.second.second.values.begin(),
                                                       elm.second.second.values.end());
        rpe_dataset.at(elm.first).second.timestamps.insert(rpe_dataset.at(elm.first).second.timestamps.end(),
                                                           elm.second.second.timestamps.begin(), elm.second.second.timestamps.end());
      }
    }

    // Compute our mean ATE score
    ate_dataset_ori.calculate();
    ate_dataset_pos.calculate();
    ate_2d_dataset_ori.calculate();
    ate_2d_dataset_pos.calculate();

    // Print stats for this specific dataset
    std::string prefix = (ate_dataset_ori.mean > 10 || ate_dataset_pos.mean > 10) ? RED : "";
    PRINT_INFO("%s\tATE: mean_ori = %.3f | mean_pos = %.3f (%d runs)\n" RESET, prefix.c_str(), ate_dataset_ori.mean, ate_dataset_pos.mean,
               (int)ate_dataset_ori.values.size());
    PRINT_INFO("\tATE: std_ori  = %.5f | std_pos  = %.5f\n", ate_dataset_ori.std, ate_dataset_pos.std);
    PRINT_INFO("\tATE 2D: mean_ori = %.3f | mean_pos = %.3f (%d runs)\n", ate_2d_dataset_ori.mean, ate_2d_dataset_pos.mean,
               (int)ate_2d_dataset_ori.values.size());
    PRINT_INFO("\tATE 2D: std_ori  = %.5f | std_pos  = %.5f\n", ate_2d_dataset_ori.std, ate_2d_dataset_pos.std);
    for (auto &seg : rpe_dataset) {
      seg.second.first.calculate();
      seg.second.second.calculate();
      PRINT_INFO("\tRPE: seg %d - mean_ori = %.3f | mean_pos = %.3f (%d samples)\n", (int)seg.first, seg.second.first.mean,
                 seg.second.second.mean, (int)seg.second.second.values.size());
      // PRINT_DEBUG("RPE: seg %d - std_ori  = %.3f | std_pos  = %.3f\n",(int)seg.first,seg.second.first.std,seg.second.second.std);
    }

    // RMSE: Convert into the right format (only use times where all runs have an error)
    ov_eval::Statistics rmse_ori, rmse_pos;
    for (auto &elm : rmse_dataset) {
      if (elm.second.first.values.size() == file_paths.size()) {
        elm.second.first.calculate();
        elm.second.second.calculate();
        rmse_ori.timestamps.push_back(elm.first);
        rmse_ori.values.push_back(elm.second.first.rmse);
        rmse_pos.timestamps.push_back(elm.first);
        rmse_pos.values.push_back(elm.second.second.rmse);
      }
    }
    rmse_ori.calculate();
    rmse_pos.calculate();
    PRINT_INFO("\tRMSE: mean_ori = %.3f | mean_pos = %.3f\n", rmse_ori.mean, rmse_pos.mean);

    // RMSE: Convert into the right format (only use times where all runs have an error)
    ov_eval::Statistics rmse_2d_ori, rmse_2d_pos;
    for (auto &elm : rmse_2d_dataset) {
      if (elm.second.first.values.size() == file_paths.size()) {
        elm.second.first.calculate();
        elm.second.second.calculate();
        rmse_2d_ori.timestamps.push_back(elm.first);
        rmse_2d_ori.values.push_back(elm.second.first.rmse);
        rmse_2d_pos.timestamps.push_back(elm.first);
        rmse_2d_pos.values.push_back(elm.second.second.rmse);
      }
    }
    rmse_2d_ori.calculate();
    rmse_2d_pos.calculate();
    PRINT_INFO("\tRMSE 2D: mean_ori = %.3f | mean_pos = %.3f\n", rmse_2d_ori.mean, rmse_2d_pos.mean);

    // NEES: Convert into the right format (only use times where all runs have an error)
    ov_eval::Statistics nees_ori, nees_pos;
    for (auto &elm : nees_dataset) {
      if (elm.second.first.values.size() == file_paths.size()) {
        elm.second.first.calculate();
        elm.second.second.calculate();
        nees_ori.timestamps.push_back(elm.first);
        nees_ori.values.push_back(elm.second.first.mean);
        nees_pos.timestamps.push_back(elm.first);
        nees_pos.values.push_back(elm.second.second.mean);
      }
    }
    nees_ori.calculate();
    nees_pos.calculate();
    PRINT_INFO("\tNEES: mean_ori = %.3f | mean_pos = %.3f\n", nees_ori.mean, nees_pos.mean);

#ifdef HAVE_PYTHONLIBS

    //=====================================================
    // RMSE plot at each timestep
    matplotlibcpp::figure_size(1000, 600);

    // Zero our time arrays
    double starttime1 = (rmse_ori.timestamps.empty()) ? 0 : rmse_ori.timestamps.at(0);
    double endtime1 = (rmse_ori.timestamps.empty()) ? 0 : rmse_ori.timestamps.at(rmse_ori.timestamps.size() - 1);
    for (size_t j = 0; j < rmse_ori.timestamps.size(); j++) {
      rmse_ori.timestamps.at(j) -= starttime1;
      rmse_pos.timestamps.at(j) -= starttime1;
    }

    // Update the title and axis labels
    matplotlibcpp::subplot(2, 1, 1);
    matplotlibcpp::title("Root Mean Squared Error - " + path_algorithms.at(i).filename().string());
    matplotlibcpp::ylabel("Error Orientation (deg)");
    matplotlibcpp::plot(rmse_ori.timestamps, rmse_ori.values);
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);
    matplotlibcpp::subplot(2, 1, 2);
    matplotlibcpp::ylabel("Error Position (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::plot(rmse_pos.timestamps, rmse_pos.values);
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);

    //=====================================================

    if (!nees_ori.values.empty() && !nees_pos.values.empty()) {
      // NEES plot at each timestep
      matplotlibcpp::figure_size(1000, 600);

      // Zero our time arrays
      double starttime2 = (nees_ori.timestamps.empty()) ? 0 : nees_ori.timestamps.at(0);
      double endtime2 = (nees_ori.timestamps.empty()) ? 0 : nees_ori.timestamps.at(nees_ori.timestamps.size() - 1);
      for (size_t j = 0; j < nees_ori.timestamps.size(); j++) {
        nees_ori.timestamps.at(j) -= starttime2;
        nees_pos.timestamps.at(j) -= starttime2;
      }

      // Update the title and axis labels
      matplotlibcpp::subplot(2, 1, 1);
      matplotlibcpp::title("Normalized Estimation Error Squared - " + path_algorithms.at(i).filename().string());
      matplotlibcpp::ylabel("NEES Orientation");
      matplotlibcpp::plot(nees_ori.timestamps, nees_ori.values);
      matplotlibcpp::xlim(0.0, endtime2 - starttime2);
      matplotlibcpp::subplot(2, 1, 2);
      matplotlibcpp::ylabel("NEES Position");
      matplotlibcpp::xlabel("dataset time (s)");
      matplotlibcpp::plot(nees_pos.timestamps, nees_pos.values);
      matplotlibcpp::xlim(0.0, endtime2 - starttime2);

      // Display to the user
      matplotlibcpp::tight_layout();
      matplotlibcpp::show(false);
    }

#endif
  }

  // Final line for our printed stats
  PRINT_DEBUG("============================================\n");

#ifdef HAVE_PYTHONLIBS

  // Wait till the user kills this node
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
