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

#ifndef OV_EVAL_SIMULATION_H
#define OV_EVAL_SIMULATION_H

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>

#include "utils/Loader.h"
#include "utils/Statistics.h"

#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

namespace ov_eval {

/**
 * @brief A single simulation run (the full state not just pose).
 *
 * This should match the recording logic that is in the ov_msckf::RosVisualizer in which we write both estimate, their deviation, and
 * groundtruth to three files. We enforce that these files first contain the current IMU state, then time offset, number of cameras, then
 * the camera calibration states. If we are not performing calibration these should all be written to file, just their deviation should be
 * zero as they are 100% certain.
 */
class ResultSimulation {

public:
  /**
   * @brief Default constructor that will load our data from file
   * @param path_est Path to the estimate text file
   * @param path_std Path to the standard deviation file
   * @param path_gt Path to the groundtruth text file
   */
  ResultSimulation(std::string path_est, std::string path_std, std::string path_gt);

  /**
   * @brief Will plot the state error and its three sigma bounds
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_state(bool doplotting, double max_time = INFINITY);

  /**
   * @brief Will plot the state imu camera offset and its sigma bound
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_timeoff(bool doplotting, double max_time = INFINITY);

  /**
   * @brief Will plot the camera calibration intrinsics
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_cam_instrinsics(bool doplotting, double max_time = INFINITY);

  /**
   * @brief Will plot the camera calibration extrinsic transform
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_cam_extrinsics(bool doplotting, double max_time = INFINITY);

  /**
   * @brief Will plot the imu intrinsic errors
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_imu_intrinsics(bool doplotting, double max_time = INFINITY);

protected:
  // Trajectory data (loaded from file and timestamp intersected)
  std::vector<Eigen::VectorXd> est_state, gt_state;
  std::vector<Eigen::VectorXd> state_cov;

#ifdef HAVE_PYTHONLIBS

  /**
   * @brief Plots three different statistic values and sigma bounds
   * @param sx X-axis error
   * @param sy Y-axis error
   * @param sz Z-axis error
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, std::string color_err, std::string color_std) {

    // Zero our time arrays
    double starttime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps.at(0);
    double endtime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps.at(sx.timestamps.size() - 1);
    for (size_t i = 0; i < sx.timestamps.size(); i++) {
      sx.timestamps.at(i) -= starttime1;
    }
    double starttime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps.at(0);
    double endtime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps.at(sy.timestamps.size() - 1);
    for (size_t i = 0; i < sy.timestamps.size(); i++) {
      sy.timestamps.at(i) -= starttime2;
    }
    double starttime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps.at(0);
    double endtime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps.at(sz.timestamps.size() - 1);
    for (size_t i = 0; i < sz.timestamps.size(); i++) {
      sz.timestamps.at(i) -= starttime3;
    }

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
    if (!sx.values_bound.empty()) {
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
      for (size_t i = 0; i < sx.timestamps.size(); i++) {
        sx.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
    if (!sy.values_bound.empty()) {
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
      for (size_t i = 0; i < sy.timestamps.size(); i++) {
        sy.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
    if (!sz.values_bound.empty()) {
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
      for (size_t i = 0; i < sz.timestamps.size(); i++) {
        sz.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);
  }

  /**
   * @brief Plots four different statistic values and sigma bounds
   * @param sx Error one
   * @param sy Error two
   * @param sz Error three
   * @param sk Error four
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_4errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, ov_eval::Statistics sk, std::string color_err,
                    std::string color_std) {

    // Zero our time arrays
    double starttime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps.at(0);
    double endtime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps.at(sx.timestamps.size() - 1);
    for (size_t i = 0; i < sx.timestamps.size(); i++) {
      sx.timestamps.at(i) -= starttime1;
    }
    double starttime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps.at(0);
    double endtime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps.at(sy.timestamps.size() - 1);
    for (size_t i = 0; i < sy.timestamps.size(); i++) {
      sy.timestamps.at(i) -= starttime2;
    }
    double starttime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps.at(0);
    double endtime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps.at(sz.timestamps.size() - 1);
    for (size_t i = 0; i < sz.timestamps.size(); i++) {
      sz.timestamps.at(i) -= starttime3;
    }
    double starttime4 = (sk.timestamps.empty()) ? 0 : sk.timestamps.at(0);
    double endtime4 = (sk.timestamps.empty()) ? 0 : sk.timestamps.at(sk.timestamps.size() - 1);
    for (size_t i = 0; i < sk.timestamps.size(); i++) {
      sk.timestamps.at(i) -= starttime4;
    }

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 1);
    matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
    if (!sx.values_bound.empty()) {
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
      for (size_t i = 0; i < sx.timestamps.size(); i++) {
        sx.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 2);
    matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
    if (!sy.values_bound.empty()) {
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
      for (size_t i = 0; i < sy.timestamps.size(); i++) {
        sy.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 3);
    matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
    if (!sz.values_bound.empty()) {
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
      for (size_t i = 0; i < sz.timestamps.size(); i++) {
        sz.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 4);
    matplotlibcpp::plot(sk.timestamps, sk.values, params_value);
    if (!sk.values_bound.empty()) {
      matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
      for (size_t i = 0; i < sk.timestamps.size(); i++) {
        sk.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime4 - starttime4);
  }
  /**
   * @brief Plots six different statistic values and sigma bounds
   * @param s1 Error one
   * @param s2 Error two
   * @param s3 Error three
   * @param s4 Error four
   * @param s5 Error five
   * @param s6 Error six
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_6errors(ov_eval::Statistics s1, ov_eval::Statistics s2, ov_eval::Statistics s3, ov_eval::Statistics s4, ov_eval::Statistics s5,
                    ov_eval::Statistics s6, std::string color_err, std::string color_std) {

    // Zero our time arrays
    double starttime1 = (s1.timestamps.empty()) ? 0 : s1.timestamps.at(0);
    double endtime1 = (s1.timestamps.empty()) ? 0 : s1.timestamps.at(s1.timestamps.size() - 1);
    for (size_t i = 0; i < s1.timestamps.size(); i++) {
      s1.timestamps.at(i) -= starttime1;
    }
    double starttime2 = (s2.timestamps.empty()) ? 0 : s2.timestamps.at(0);
    double endtime2 = (s2.timestamps.empty()) ? 0 : s2.timestamps.at(s2.timestamps.size() - 1);
    for (size_t i = 0; i < s2.timestamps.size(); i++) {
      s2.timestamps.at(i) -= starttime2;
    }
    double starttime3 = (s3.timestamps.empty()) ? 0 : s3.timestamps.at(0);
    double endtime3 = (s3.timestamps.empty()) ? 0 : s3.timestamps.at(s3.timestamps.size() - 1);
    for (size_t i = 0; i < s3.timestamps.size(); i++) {
      s3.timestamps.at(i) -= starttime3;
    }
    double starttime4 = (s4.timestamps.empty()) ? 0 : s4.timestamps.at(0);
    double endtime4 = (s4.timestamps.empty()) ? 0 : s4.timestamps.at(s4.timestamps.size() - 1);
    for (size_t i = 0; i < s4.timestamps.size(); i++) {
      s4.timestamps.at(i) -= starttime4;
    }
    double starttime5 = (s5.timestamps.empty()) ? 0 : s5.timestamps.at(0);
    double endtime5 = (s5.timestamps.empty()) ? 0 : s5.timestamps.at(s5.timestamps.size() - 1);
    for (size_t i = 0; i < s5.timestamps.size(); i++) {
      s5.timestamps.at(i) -= starttime5;
    }
    double starttime6 = (s6.timestamps.empty()) ? 0 : s6.timestamps.at(0);
    double endtime6 = (s6.timestamps.empty()) ? 0 : s6.timestamps.at(s6.timestamps.size() - 1);
    for (size_t i = 0; i < s6.timestamps.size(); i++) {
      s6.timestamps.at(i) -= starttime6;
    }

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 1);
    matplotlibcpp::plot(s1.timestamps, s1.values, params_value);
    if (!s1.values_bound.empty()) {
      matplotlibcpp::plot(s1.timestamps, s1.values_bound, params_bound);
      for (size_t i = 0; i < s1.timestamps.size(); i++) {
        s1.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s1.timestamps, s1.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 2);
    matplotlibcpp::plot(s2.timestamps, s2.values, params_value);
    if (!s2.values_bound.empty()) {
      matplotlibcpp::plot(s2.timestamps, s2.values_bound, params_bound);
      for (size_t i = 0; i < s2.timestamps.size(); i++) {
        s2.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s2.timestamps, s2.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 3);
    matplotlibcpp::plot(s3.timestamps, s3.values, params_value);
    if (!s3.values_bound.empty()) {
      matplotlibcpp::plot(s3.timestamps, s3.values_bound, params_bound);
      for (size_t i = 0; i < s3.timestamps.size(); i++) {
        s3.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s3.timestamps, s3.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 4);
    matplotlibcpp::plot(s4.timestamps, s4.values, params_value);
    if (!s4.values_bound.empty()) {
      matplotlibcpp::plot(s4.timestamps, s4.values_bound, params_bound);
      for (size_t i = 0; i < s4.timestamps.size(); i++) {
        s4.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s4.timestamps, s4.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime4 - starttime4);

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 5);
    matplotlibcpp::plot(s5.timestamps, s5.values, params_value);
    if (!s5.values_bound.empty()) {
      matplotlibcpp::plot(s5.timestamps, s5.values_bound, params_bound);
      for (size_t i = 0; i < s5.timestamps.size(); i++) {
        s5.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s5.timestamps, s5.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime5 - starttime5);

    // Plot our error value
    matplotlibcpp::subplot(3, 2, 6);
    matplotlibcpp::plot(s6.timestamps, s6.values, params_value);
    if (!s6.values_bound.empty()) {
      matplotlibcpp::plot(s6.timestamps, s6.values_bound, params_bound);
      for (size_t i = 0; i < s6.timestamps.size(); i++) {
        s6.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s6.timestamps, s6.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime6 - starttime6);
  }

  /**
   * @brief Plots six different statistic values and sigma bounds
   * @param s1 Error one
   * @param s2 Error two
   * @param s3 Error three
   * @param s4 Error four
   * @param s5 Error five
   * @param s6 Error six
   * @param s7 Error four
   * @param s8 Error five
   * @param s9 Error six
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_9errors(ov_eval::Statistics s1, ov_eval::Statistics s2, ov_eval::Statistics s3, ov_eval::Statistics s4, ov_eval::Statistics s5,
                    ov_eval::Statistics s6, ov_eval::Statistics s7, ov_eval::Statistics s8, ov_eval::Statistics s9, std::string color_err,
                    std::string color_std) {

    // Zero our time arrays
    double starttime1 = (s1.timestamps.empty()) ? 0 : s1.timestamps.at(0);
    double endtime1 = (s1.timestamps.empty()) ? 0 : s1.timestamps.at(s1.timestamps.size() - 1);
    for (size_t i = 0; i < s1.timestamps.size(); i++) {
      s1.timestamps.at(i) -= starttime1;
    }
    double starttime2 = (s2.timestamps.empty()) ? 0 : s2.timestamps.at(0);
    double endtime2 = (s2.timestamps.empty()) ? 0 : s2.timestamps.at(s2.timestamps.size() - 1);
    for (size_t i = 0; i < s2.timestamps.size(); i++) {
      s2.timestamps.at(i) -= starttime2;
    }
    double starttime3 = (s3.timestamps.empty()) ? 0 : s3.timestamps.at(0);
    double endtime3 = (s3.timestamps.empty()) ? 0 : s3.timestamps.at(s3.timestamps.size() - 1);
    for (size_t i = 0; i < s3.timestamps.size(); i++) {
      s3.timestamps.at(i) -= starttime3;
    }
    double starttime4 = (s4.timestamps.empty()) ? 0 : s4.timestamps.at(0);
    double endtime4 = (s4.timestamps.empty()) ? 0 : s4.timestamps.at(s4.timestamps.size() - 1);
    for (size_t i = 0; i < s4.timestamps.size(); i++) {
      s4.timestamps.at(i) -= starttime4;
    }
    double starttime5 = (s5.timestamps.empty()) ? 0 : s5.timestamps.at(0);
    double endtime5 = (s5.timestamps.empty()) ? 0 : s5.timestamps.at(s5.timestamps.size() - 1);
    for (size_t i = 0; i < s5.timestamps.size(); i++) {
      s5.timestamps.at(i) -= starttime5;
    }
    double starttime6 = (s6.timestamps.empty()) ? 0 : s6.timestamps.at(0);
    double endtime6 = (s6.timestamps.empty()) ? 0 : s6.timestamps.at(s6.timestamps.size() - 1);
    for (size_t i = 0; i < s6.timestamps.size(); i++) {
      s6.timestamps.at(i) -= starttime6;
    }
    double starttime7 = (s7.timestamps.empty()) ? 0 : s7.timestamps.at(0);
    double endtime7 = (s7.timestamps.empty()) ? 0 : s7.timestamps.at(s7.timestamps.size() - 1);
    for (size_t i = 0; i < s7.timestamps.size(); i++) {
      s7.timestamps.at(i) -= starttime7;
    }
    double starttime8 = (s8.timestamps.empty()) ? 0 : s8.timestamps.at(0);
    double endtime8 = (s8.timestamps.empty()) ? 0 : s8.timestamps.at(s8.timestamps.size() - 1);
    for (size_t i = 0; i < s8.timestamps.size(); i++) {
      s8.timestamps.at(i) -= starttime8;
    }
    double starttime9 = (s9.timestamps.empty()) ? 0 : s9.timestamps.at(0);
    double endtime9 = (s9.timestamps.empty()) ? 0 : s9.timestamps.at(s9.timestamps.size() - 1);
    for (size_t i = 0; i < s9.timestamps.size(); i++) {
      s9.timestamps.at(i) -= starttime9;
    }

    // Parameters that define the line styles
    std::map<std::string, std::string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 1);
    matplotlibcpp::plot(s1.timestamps, s1.values, params_value);
    if (!s1.values_bound.empty()) {
      matplotlibcpp::plot(s1.timestamps, s1.values_bound, params_bound);
      for (size_t i = 0; i < s1.timestamps.size(); i++) {
        s1.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s1.timestamps, s1.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 2);
    matplotlibcpp::plot(s2.timestamps, s2.values, params_value);
    if (!s2.values_bound.empty()) {
      matplotlibcpp::plot(s2.timestamps, s2.values_bound, params_bound);
      for (size_t i = 0; i < s2.timestamps.size(); i++) {
        s2.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s2.timestamps, s2.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 3);
    matplotlibcpp::plot(s3.timestamps, s3.values, params_value);
    if (!s3.values_bound.empty()) {
      matplotlibcpp::plot(s3.timestamps, s3.values_bound, params_bound);
      for (size_t i = 0; i < s3.timestamps.size(); i++) {
        s3.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s3.timestamps, s3.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 4);
    matplotlibcpp::plot(s4.timestamps, s4.values, params_value);
    if (!s4.values_bound.empty()) {
      matplotlibcpp::plot(s4.timestamps, s4.values_bound, params_bound);
      for (size_t i = 0; i < s4.timestamps.size(); i++) {
        s4.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s4.timestamps, s4.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime4 - starttime4);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 5);
    matplotlibcpp::plot(s5.timestamps, s5.values, params_value);
    if (!s5.values_bound.empty()) {
      matplotlibcpp::plot(s5.timestamps, s5.values_bound, params_bound);
      for (size_t i = 0; i < s5.timestamps.size(); i++) {
        s5.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s5.timestamps, s5.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime5 - starttime5);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 6);
    matplotlibcpp::plot(s6.timestamps, s6.values, params_value);
    if (!s6.values_bound.empty()) {
      matplotlibcpp::plot(s6.timestamps, s6.values_bound, params_bound);
      for (size_t i = 0; i < s6.timestamps.size(); i++) {
        s6.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s6.timestamps, s6.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime6 - starttime6);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 7);
    matplotlibcpp::plot(s7.timestamps, s7.values, params_value);
    if (!s7.values_bound.empty()) {
      matplotlibcpp::plot(s7.timestamps, s7.values_bound, params_bound);
      for (size_t i = 0; i < s7.timestamps.size(); i++) {
        s7.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s7.timestamps, s7.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime7 - starttime7);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 8);
    matplotlibcpp::plot(s8.timestamps, s8.values, params_value);
    if (!s8.values_bound.empty()) {
      matplotlibcpp::plot(s8.timestamps, s8.values_bound, params_bound);
      for (size_t i = 0; i < s8.timestamps.size(); i++) {
        s8.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s8.timestamps, s8.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime8 - starttime8);

    // Plot our error value
    matplotlibcpp::subplot(3, 3, 9);
    matplotlibcpp::plot(s9.timestamps, s9.values, params_value);
    if (!s9.values_bound.empty()) {
      matplotlibcpp::plot(s9.timestamps, s9.values_bound, params_bound);
      for (size_t i = 0; i < s9.timestamps.size(); i++) {
        s9.values_bound.at(i) *= -1;
      }
      matplotlibcpp::plot(s9.timestamps, s9.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime9 - starttime9);
  }

#endif
};

} // namespace ov_eval

#endif // OV_EVAL_SIMULATION_H
