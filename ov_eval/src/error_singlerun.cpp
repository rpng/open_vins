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
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

// Will plot three error values in three sub-plots in our current figure
void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz) {

  // Parameters that define the line styles
  std::map<std::string, std::string> params_value, params_bound;
  params_value.insert({"label", "error"});
  params_value.insert({"linestyle", "-"});
  params_value.insert({"color", "blue"});
  params_bound.insert({"label", "3 sigma bound"});
  params_bound.insert({"linestyle", "--"});
  params_bound.insert({"color", "red"});

  // Plot our error value
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
  if (!sx.values_bound.empty()) {
    matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    for (size_t i = 0; i < sx.timestamps.size(); i++) {
      sx.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sx.timestamps, sx.values_bound, "r--");
  }

  // Plot our error value
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
  if (!sy.values_bound.empty()) {
    matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    for (size_t i = 0; i < sy.timestamps.size(); i++) {
      sy.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sy.timestamps, sy.values_bound, "r--");
  }

  // Plot our error value
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
  if (!sz.values_bound.empty()) {
    matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    for (size_t i = 0; i < sz.timestamps.size(); i++) {
      sz.values_bound.at(i) *= -1;
    }
    matplotlibcpp::plot(sz.timestamps, sz.values_bound, "r--");
  }
}

#endif

int main(int argc, char **argv) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have a path
  if (argc < 4) {
    PRINT_ERROR(RED "ERROR: Please specify a align mode, groudtruth, and algorithm run file\n" RESET);
    PRINT_ERROR(RED "ERROR: ./error_singlerun <align_mode> <file_gt.txt> <file_est.txt>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval error_singlerun <align_mode> <file_gt.txt> <file_est.txt>\n" RESET);
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

  // Create our trajectory object
  ov_eval::ResultTrajectory traj(argv[3], argv[2], argv[1]);

  //===========================================================
  // Absolute trajectory error
  //===========================================================

  // Calculate
  ov_eval::Statistics error_ori, error_pos;
  traj.calculate_ate(error_ori, error_pos);

  // Print it
  PRINT_INFO("======================================\n");
  PRINT_INFO("Absolute Trajectory Error\n");
  PRINT_INFO("======================================\n");
  PRINT_INFO("rmse_ori = %.3f | rmse_pos = %.3f\n", error_ori.rmse, error_pos.rmse);
  PRINT_INFO("mean_ori = %.3f | mean_pos = %.3f\n", error_ori.mean, error_pos.mean);
  PRINT_INFO("min_ori  = %.3f | min_pos  = %.3f\n", error_ori.min, error_pos.min);
  PRINT_INFO("max_ori  = %.3f | max_pos  = %.3f\n", error_ori.max, error_pos.max);
  PRINT_INFO("std_ori  = %.3f | std_pos  = %.3f\n", error_ori.std, error_pos.std);

  //===========================================================
  // Relative pose error
  //===========================================================

  // Calculate
  std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
  std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> error_rpe;
  traj.calculate_rpe(segments, error_rpe);

  // Print it
  PRINT_INFO("======================================\n");
  PRINT_INFO("Relative Pose Error\n");
  PRINT_INFO("======================================\n");
  for (const auto &seg : error_rpe) {
    PRINT_INFO("seg %d - median_ori = %.3f | median_pos = %.3f (%d samples)\n", (int)seg.first, seg.second.first.median,
               seg.second.second.median, (int)seg.second.second.values.size());
    // PRINT_DEBUG("seg %d - std_ori  = %.3f | std_pos  = %.3f\n",(int)seg.first,seg.second.first.std,seg.second.second.std);
  }

#ifdef HAVE_PYTHONLIBS

  // Parameters
  std::map<std::string, std::string> params_rpe;
  params_rpe.insert({"notch", "true"});
  params_rpe.insert({"sym", ""});

  // Plot this figure
  matplotlibcpp::figure_size(800, 600);

  // Plot each RPE next to each other
  double ct = 1;
  double width = 0.50;
  std::vector<double> xticks;
  std::vector<std::string> labels;
  for (const auto &seg : error_rpe) {
    xticks.push_back(ct);
    labels.push_back(std::to_string((int)seg.first));
    matplotlibcpp::boxplot(seg.second.first.values, ct++, width, "blue", "-", params_rpe);
  }

  // Display to the user
  matplotlibcpp::xlim(0.5, ct - 0.5);
  matplotlibcpp::xticks(xticks, labels);
  matplotlibcpp::title("Relative Orientation Error");
  matplotlibcpp::ylabel("orientation error (deg)");
  matplotlibcpp::xlabel("sub-segment lengths (m)");
  matplotlibcpp::show(false);

  // Plot this figure
  matplotlibcpp::figure_size(800, 600);

  // Plot each RPE next to each other
  ct = 1;
  for (const auto &seg : error_rpe) {
    matplotlibcpp::boxplot(seg.second.second.values, ct++, width, "blue", "-", params_rpe);
  }

  // Display to the user
  matplotlibcpp::xlim(0.5, ct - 0.5);
  matplotlibcpp::xticks(xticks, labels);
  matplotlibcpp::title("Relative Position Error");
  matplotlibcpp::ylabel("translation error (m)");
  matplotlibcpp::xlabel("sub-segment lengths (m)");
  matplotlibcpp::show(false);

#endif

  //===========================================================
  // Normalized Estimation Error Squared
  //===========================================================

  // Calculate
  ov_eval::Statistics nees_ori, nees_pos;
  traj.calculate_nees(nees_ori, nees_pos);

  // Print it
  PRINT_INFO("======================================\n");
  PRINT_INFO("Normalized Estimation Error Squared\n");
  PRINT_INFO("======================================\n");
  PRINT_INFO("mean_ori = %.3f | mean_pos = %.3f\n", nees_ori.mean, nees_pos.mean);
  PRINT_INFO("min_ori  = %.3f | min_pos  = %.3f\n", nees_ori.min, nees_pos.min);
  PRINT_INFO("max_ori  = %.3f | max_pos  = %.3f\n", nees_ori.max, nees_pos.max);
  PRINT_INFO("std_ori  = %.3f | std_pos  = %.3f\n", nees_ori.std, nees_pos.std);
  PRINT_INFO("======================================\n");

#ifdef HAVE_PYTHONLIBS

  if (!nees_ori.values.empty() && !nees_pos.values.empty()) {
    // Zero our time arrays
    double starttime1 = (nees_ori.timestamps.empty()) ? 0 : nees_ori.timestamps.at(0);
    double endtime1 = (nees_ori.timestamps.empty()) ? 0 : nees_ori.timestamps.at(nees_ori.timestamps.size() - 1);
    for (size_t i = 0; i < nees_ori.timestamps.size(); i++) {
      nees_ori.timestamps.at(i) -= starttime1;
      nees_pos.timestamps.at(i) -= starttime1;
    }

    // Plot this figure
    matplotlibcpp::figure_size(1000, 600);

    // Parameters that define the line styles
    std::map<std::string, std::string> params_neesp, params_neeso;
    params_neesp.insert({"label", "nees position"});
    params_neesp.insert({"linestyle", "-"});
    params_neesp.insert({"color", "blue"});
    params_neeso.insert({"label", "nees orientation"});
    params_neeso.insert({"linestyle", "-"});
    params_neeso.insert({"color", "blue"});

    // Update the title and axis labels
    matplotlibcpp::subplot(2, 1, 1);
    matplotlibcpp::title("Normalized Estimation Error Squared");
    matplotlibcpp::ylabel("NEES Orientation");
    matplotlibcpp::plot(nees_ori.timestamps, nees_ori.values, params_neeso);
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);
    matplotlibcpp::subplot(2, 1, 2);
    matplotlibcpp::ylabel("NEES Position");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::plot(nees_pos.timestamps, nees_pos.values, params_neesp);
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Display to the user
    matplotlibcpp::tight_layout();
    matplotlibcpp::show(false);
  }

#endif

  //===========================================================
  // Plot the error if we have matplotlib to plot!
  //===========================================================

  // Calculate
  ov_eval::Statistics posx, posy, posz;
  ov_eval::Statistics orix, oriy, oriz;
  ov_eval::Statistics roll, pitch, yaw;
  traj.calculate_error(posx, posy, posz, orix, oriy, oriz, roll, pitch, yaw);

  // Zero our time arrays
  double starttime2 = (posx.timestamps.empty()) ? 0 : posx.timestamps.at(0);
  double endtime2 = (posx.timestamps.empty()) ? 0 : posx.timestamps.at(posx.timestamps.size() - 1);
  for (size_t i = 0; i < posx.timestamps.size(); i++) {
    posx.timestamps.at(i) -= starttime2;
    posy.timestamps.at(i) -= starttime2;
    posz.timestamps.at(i) -= starttime2;
    orix.timestamps.at(i) -= starttime2;
    oriy.timestamps.at(i) -= starttime2;
    oriz.timestamps.at(i) -= starttime2;
    roll.timestamps.at(i) -= starttime2;
    pitch.timestamps.at(i) -= starttime2;
    yaw.timestamps.at(i) -= starttime2;
  }

#ifdef HAVE_PYTHONLIBS

  //=====================================================
  // Plot this figure
  matplotlibcpp::figure_size(1000, 600);
  plot_3errors(posx, posy, posz);

  // Update the title and axis labels
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::title("IMU Position Error");
  matplotlibcpp::ylabel("x-error (m)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::ylabel("y-error (m)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::ylabel("z-error (m)");
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);

  // Display to the user
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(false);

  //=====================================================
  // Plot this figure
  matplotlibcpp::figure_size(1000, 600);
  plot_3errors(orix, oriy, oriz);

  // Update the title and axis labels
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::title("IMU Orientation Error");
  matplotlibcpp::ylabel("x-error (deg)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::ylabel("y-error (deg)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::ylabel("z-error (deg)");
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);

  // Display to the user
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(false);

  //=====================================================
  // Plot this figure
  matplotlibcpp::figure_size(1000, 600);
  plot_3errors(roll, pitch, yaw);

  // Update the title and axis labels
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::title("Global Orientation RPY Error");
  matplotlibcpp::ylabel("roll error (deg)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::ylabel("pitch error (deg)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::ylabel("yaw error (deg)");
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::xlim(0.0, endtime2 - starttime2);

  // Display to the user
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
