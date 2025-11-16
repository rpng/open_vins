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

#include <cmath>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "utils/print.h"

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

void print_stats(std::string title, std::vector<double> times, std::string title2 = "", std::vector<int> stats = {}) {

  // Compute mean and rmse
  double mean = 0.0;
  double rmse = 0.0;
  for (size_t i = 0; i < times.size(); i++) {
    assert(!std::isnan(times.at(i)));
    mean += times.at(i);
    rmse += times.at(i) * times.at(i);
  }
  mean /= times.size();
  rmse = std::sqrt(rmse / times.size());

  // Using mean, compute standard deviation
  double std = 0;
  for (size_t i = 0; i < times.size(); i++) {
    std += std::pow(times.at(i) - mean, 2);
  }
  std = std::sqrt(std / (times.size() - 1));

  // Print!
  if (stats.empty()) {
    PRINT_INFO("%s: %.3f +- %.3fms\n", title.c_str(), mean, std);
  } else {
    double mean2 = 0.0;
    for (size_t i = 0; i < stats.size(); i++) {
      assert(!std::isnan(stats.at(i)));
      mean2 += stats.at(i);
    }
    mean2 /= stats.size();
    PRINT_INFO("%s: %.3f +- %.3f ms (%.2f avg %s)\n", title.c_str(), mean, std, mean2, title2.c_str());
  }
}

// Main function
int main(int argc, char **argv) {

  // Verbosity
  std::string verbosity = "INFO";
  ov_core::Printer::setPrintLevel(verbosity);

  // Parameters used in all algorithms
  int num_trials = 100;
  int big_matrix = 1000;
  int big_matrix_eigen1 = 300;
  int big_matrix_eigen2 = 900;
  int pyr_levels = 5;
  int fast_threshold = 30;
  int max_features = 500;
  cv::Size win_size = cv::Size(15, 15);
  cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);

  // Helper data structures
  cv::Mat img1(big_matrix, big_matrix, CV_8UC1);
  cv::Mat img3(big_matrix, big_matrix, CV_8UC3);
  std::vector<double> times_ms;
  std::vector<int> extra_stats;

  // OPENCV: RANDOM BIG IMAGE
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    cv::randu(img3, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("OPENCV: RANDOM BIG IMAGE", times_ms);

  // OPENCV: HISTOGRAM EQUALIZATION
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    cv::randu(img1, cv::Scalar(0), cv::Scalar(255));
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    cv::equalizeHist(img1, img1);
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("OPENCV: HISTOGRAM EQUALIZATION", times_ms);

  // OPENCV: BUILD OPTICAL FLOW PYRAMID
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    cv::randu(img1, cv::Scalar(0), cv::Scalar(255));
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    std::vector<cv::Mat> imgpyr;
    cv::buildOpticalFlowPyramid(img1, imgpyr, win_size, pyr_levels);
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("OPENCV: BUILD OPTICAL FLOW PYRAMID", times_ms);

  // OPENCV: FAST FEATURE EXTRACTION
  times_ms.clear();
  extra_stats.clear();
  for (int i = 0; i < num_trials; i++) {
    cv::randu(img1, cv::Scalar(0), cv::Scalar(255));
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    std::vector<cv::KeyPoint> pts_new;
    cv::FAST(img1, pts_new, fast_threshold, true);
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
    extra_stats.push_back((int)pts_new.size());
  }
  print_stats("OPENCV: FAST FEATURE EXTRACTION", times_ms, "feats", extra_stats);

  // OPENCV: KLT OPTICAL FLOW
  times_ms.clear();
  extra_stats.clear();
  for (int i = 0; i < num_trials; i++) {
    std::vector<cv::Mat> imgpyr1, imgpyr2;
    cv::buildOpticalFlowPyramid(img1, imgpyr1, win_size, pyr_levels);
    std::vector<cv::KeyPoint> pts0_tmp;
    cv::FAST(img1, pts0_tmp, fast_threshold, true);
    std::sort(pts0_tmp.begin(), pts0_tmp.end(), [](cv::KeyPoint first, cv::KeyPoint second) { return first.response > second.response; });
    std::vector<cv::Point2f> pts0;
    for (size_t j = 0; j < pts0_tmp.size() && (int)j < max_features; j++)
      pts0.push_back(pts0_tmp.at(j).pt);
    cv::randu(img1, cv::Scalar(0), cv::Scalar(255)); // second image
    cv::buildOpticalFlowPyramid(img1, imgpyr2, win_size, pyr_levels);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    std::vector<uchar> mask_klt;
    std::vector<float> error;
    std::vector<cv::Point2f> pts1, pts1_good;
    cv::calcOpticalFlowPyrLK(imgpyr1, imgpyr2, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit);
    for (size_t j = 0; j < mask_klt.size(); j++) {
      if (mask_klt.at(j))
        pts1_good.push_back(pts1.at(j));
    }
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
    extra_stats.push_back((int)pts1_good.size());
  }
  print_stats("OPENCV: KLT OPTICAL FLOW", times_ms, "feats", extra_stats);

  //=====================================================================================
  //=====================================================================================
  //=====================================================================================

  // EIGEN3(double): RANDOM BIG MATRIX
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen1, big_matrix_eigen1);
    Eigen::MatrixXd mat2 = Eigen::MatrixXd::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): RANDOM BIG MATRIX", times_ms);

  // EIGEN3(double): MATRIX MULTIPLY
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen1, big_matrix_eigen1);
    Eigen::MatrixXd mat2 = Eigen::MatrixXd::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXd mat3 = mat1 * mat2;
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): MATRIX MULTIPLY", times_ms);

  // EIGEN3(double): MATRIX INVERSION
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    mat1 = mat1.inverse();
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): MATRIX INVERSION", times_ms);

  // EIGEN3(double): HOUSEHOLDER QR FULL
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen2, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXd QR = mat1.householderQr().matrixQR();
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): HOUSEHOLDER QR FULL", times_ms);

  // EIGEN3(double): HOUSEHOLDER QR PIV
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen2, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXd R = mat1.colPivHouseholderQr().matrixR();
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): HOUSEHOLDER QR PIV", times_ms);

  // EIGEN3(double): HOUSEHOLDER QR CUSTOM
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXd mat1 = Eigen::MatrixXd::Random(big_matrix_eigen2, big_matrix_eigen1);
    Eigen::VectorXd tempV1 = Eigen::VectorXd::Zero(big_matrix_eigen2 * big_matrix_eigen1, 1);
    Eigen::VectorXd tempV2;
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    for (int k = 0; k < mat1.cols(); k++) {
      int rows_left = mat1.rows() - k;
      double beta, tau;
      mat1.col(k).segment(k, rows_left).makeHouseholder(tempV2, tau, beta);
      mat1.block(k, k, rows_left, mat1.cols() - k).applyHouseholderOnTheLeft(tempV2, tau, tempV1.data());
    }
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(double): HOUSEHOLDER QR CUSTOM", times_ms);

  //=====================================================================================
  //=====================================================================================
  //=====================================================================================

  // EIGEN3(float): RANDOM BIG MATRIX
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen1, big_matrix_eigen1);
    Eigen::MatrixXf mat2 = Eigen::MatrixXf::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): RANDOM BIG MATRIX", times_ms);

  // EIGEN3(float): MATRIX MULTIPLY
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen1, big_matrix_eigen1);
    Eigen::MatrixXf mat2 = Eigen::MatrixXf::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXf mat3 = mat1 * mat2;
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): MATRIX MULTIPLY", times_ms);

  // EIGEN3(float): MATRIX INVERSION
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen1, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    mat1 = mat1.inverse();
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): MATRIX INVERSION", times_ms);

  // EIGEN3(float): HOUSEHOLDER QR FULL
  times_ms.clear();
  Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen2, big_matrix_eigen1);
  for (int i = 0; i < num_trials; i++) {
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXf Q = mat1.householderQr().householderQ();
    Eigen::MatrixXf R = Q.transpose() * mat1;
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): HOUSEHOLDER QR FULL", times_ms);

  // EIGEN3(float): HOUSEHOLDER QR PIV
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen2, big_matrix_eigen1);
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    Eigen::MatrixXf R = mat1.colPivHouseholderQr().matrixR().triangularView<Eigen::Upper>();
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): HOUSEHOLDER QR PIV", times_ms);

  // EIGEN3(float): HOUSEHOLDER QR CUSTOM
  times_ms.clear();
  for (int i = 0; i < num_trials; i++) {
    Eigen::MatrixXf mat1 = Eigen::MatrixXf::Random(big_matrix_eigen2, big_matrix_eigen1);
    Eigen::VectorXf tempV1 = Eigen::VectorXf::Zero(big_matrix_eigen2 * big_matrix_eigen1, 1);
    Eigen::VectorXf tempV2;
    auto rT1 = boost::posix_time::microsec_clock::local_time();
    for (int k = 0; k < mat1.cols(); k++) {
      int rows_left = mat1.rows() - k;
      float beta, tau;
      mat1.col(k).segment(k, rows_left).makeHouseholder(tempV2, tau, beta);
      mat1.block(k, k, rows_left, mat1.cols() - k).applyHouseholderOnTheLeft(tempV2, tau, tempV1.data());
    }
    auto rT2 = boost::posix_time::microsec_clock::local_time();
    times_ms.push_back((rT2 - rT1).total_microseconds() * 1e-3);
  }
  print_stats("EIGEN3(float): HOUSEHOLDER QR CUSTOM", times_ms);

  // Done!
  return EXIT_SUCCESS;
}
