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
#include <csignal>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include "cam/CamRadtan.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

#if ROS_AVAILABLE == 1
#include <ros/ros.h>
#endif

using namespace ov_core;

// Our feature extractor
TrackBase *extractor;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path.txt";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Initialize this as a ROS node
  ros::init(argc, argv, "test_webcam");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);
#endif

  // Load parameters
  auto parser = std::make_shared<ov_core::YamlParser>(config_path, false);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Defaults
  int num_pts = 500;
  int num_aruco = 1024;
  int clone_states = 20;
  int fast_threshold = 10;
  int grid_x = 5;
  int grid_y = 4;
  int min_px_dist = 20;
  double knn_ratio = 0.85;
  bool do_downsizing = false;
  bool use_stereo = false;
  parser->parse_config("num_pts", num_pts, false);
  parser->parse_config("num_aruco", num_aruco, false);
  parser->parse_config("clone_states", clone_states, false);
  parser->parse_config("fast_threshold", fast_threshold, false);
  parser->parse_config("grid_x", grid_x, false);
  parser->parse_config("grid_y", grid_y, false);
  parser->parse_config("min_px_dist", min_px_dist, false);
  parser->parse_config("knn_ratio", knn_ratio, false);
  parser->parse_config("do_downsizing", do_downsizing, false);
  parser->parse_config("use_stereo", use_stereo, false);

  // Histogram method
  ov_core::TrackBase::HistogramMethod method;
  std::string histogram_method_str = "HISTOGRAM";
  parser->parse_config("histogram_method", histogram_method_str, false);
  if (histogram_method_str == "NONE") {
    method = ov_core::TrackBase::NONE;
  } else if (histogram_method_str == "HISTOGRAM") {
    method = ov_core::TrackBase::HISTOGRAM;
  } else if (histogram_method_str == "CLAHE") {
    method = ov_core::TrackBase::CLAHE;
  } else {
    printf(RED "invalid feature histogram specified:\n" RESET);
    printf(RED "\t- NONE\n" RESET);
    printf(RED "\t- HISTOGRAM\n" RESET);
    printf(RED "\t- CLAHE\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Debug print!
  PRINT_DEBUG("max features: %d\n", num_pts);
  PRINT_DEBUG("max aruco: %d\n", num_aruco);
  PRINT_DEBUG("clone states: %d\n", clone_states);
  PRINT_DEBUG("grid size: %d x %d\n", grid_x, grid_y);
  PRINT_DEBUG("fast threshold: %d\n", fast_threshold);
  PRINT_DEBUG("min pixel distance: %d\n", min_px_dist);
  PRINT_DEBUG("downsize aruco image: %d\n", do_downsizing);

  // Fake camera info (we don't need this, as we are not using the normalized coordinates for anything)
  std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras;
  for (int i = 0; i < 2; i++) {
    Eigen::Matrix<double, 8, 1> cam0_calib;
    cam0_calib << 1, 1, 0, 0, 0, 0, 0, 0;
    std::shared_ptr<CamBase> camera_calib = std::make_shared<CamRadtan>(100, 100);
    camera_calib->set_value(cam0_calib);
    cameras.insert({i, camera_calib});
  }

  // Lets make a feature extractor
  extractor = new TrackKLT(cameras, num_pts, num_aruco, !use_stereo, method, fast_threshold, grid_x, grid_y, min_px_dist);
  // extractor = new TrackDescriptor(cameras, num_pts, num_aruco, !use_stereo, method, fast_threshold, grid_x, grid_y, min_px_dist,
  // knn_ratio); extractor = new TrackAruco(cameras, num_aruco, !use_stereo, method, do_downsizing);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Open the first webcam (0=laptop cam, 1=usb device)
  cv::VideoCapture cap;
  if (!cap.open(0)) {
    PRINT_ERROR(RED "Unable to open a webcam feed!\n" RESET);
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Loop forever until we break out
  double current_time = 0.0;
  std::deque<double> clonetimes;
  signal(SIGINT, signal_callback_handler);
#if ROS_AVAILABLE == 1
  while (ros::ok()) {
#else
  while (true) {
#endif

    // Get the next frame (and fake advance time forward)
    cv::Mat frame;
    cap >> frame;
    current_time += 1.0 / 30.0;

    // Stop capture if no more image feed
    if (frame.empty())
      break;

    // Stop capturing by pressing ESC
    if (cv::waitKey(10) == 27)
      break;

    // Convert to grayscale if not
    if (frame.channels() != 1)
      cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);

    // Else lets track this image
    ov_core::CameraData message;
    message.timestamp = current_time;
    message.sensor_ids.push_back(0);
    message.images.push_back(frame);
    message.masks.push_back(cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC1));
    extractor->feed_new_camera(message);

    // Display the resulting tracks
    cv::Mat img_active, img_history;
    extractor->display_active(img_active, 255, 0, 0, 0, 0, 255);
    extractor->display_history(img_history, 255, 255, 0, 255, 255, 255);

    // Show our image!
    cv::imshow("Active Tracks", img_active);
    cv::imshow("Track History", img_history);
    cv::waitKey(1);

    // Get lost tracks
    std::shared_ptr<FeatureDatabase> database = extractor->get_feature_database();
    std::vector<std::shared_ptr<Feature>> feats_lost = database->features_not_containing_newer(current_time);

    // Mark theses feature pointers as deleted
    for (size_t i = 0; i < feats_lost.size(); i++) {
      // Total number of measurements
      int total_meas = 0;
      for (auto const &pair : feats_lost[i]->timestamps) {
        total_meas += (int)pair.second.size();
      }
      // Update stats
      feats_lost[i]->to_delete = true;
    }

    // Push back the current time, as a clone time
    clonetimes.push_back(current_time);

    // Marginalized features if we have reached 5 frame tracks
    if ((int)clonetimes.size() >= clone_states) {
      // Remove features that have reached their max track length
      double margtime = clonetimes.at(0);
      clonetimes.pop_front();
      std::vector<std::shared_ptr<Feature>> feats_marg = database->features_containing(margtime);
      // Delete theses feature pointers
      for (size_t i = 0; i < feats_marg.size(); i++) {
        feats_marg[i]->to_delete = true;
      }
    }

    // Tell the feature database to delete old features
    database->cleanup();
  }

  // Done!
  return EXIT_SUCCESS;
}
