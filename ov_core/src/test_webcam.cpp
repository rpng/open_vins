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


#include <cmath>
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

#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "utils/CLI11.hpp"

using namespace ov_core;

// Our feature extractor
TrackBase *extractor;

// Main function
int main(int argc, char **argv) {

  // Create our command line parser
  CLI::App app{"test_webcam"};

  // Defaults
  int num_pts = 500;
  int num_aruco = 1024;
  int clone_states = 20;
  int fast_threshold = 10;
  int grid_x = 5;
  int grid_y = 3;
  int min_px_dist = 10;
  double knn_ratio = 0.85;
  bool do_downsizing = false;

  // Parameters for our extractor
  app.add_option("--num_pts", num_pts, "Number of feature tracks");
  app.add_option("--num_aruco", num_aruco, "Number of aruco tag ids we have");
  app.add_option("--clone_states", clone_states, "Amount of clones to visualize track length with");
  app.add_option("--fast_threshold", fast_threshold, "Fast extraction threshold");
  app.add_option("--grid_x", grid_x, "Grid x size");
  app.add_option("--grid_y", grid_y, "Grid y size");
  app.add_option("--min_px_dist", min_px_dist, "Minimum number of pixels between different tracks");
  app.add_option("--knn_ratio", knn_ratio, "Knn descriptor ratio threshold");
  app.add_option("--do_downsizing", do_downsizing, "If we should downsize our arucotag images");

  // Finally actually parse the command line and load it
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  // Debug print!
  printf("max features: %d\n", num_pts);
  printf("max aruco: %d\n", num_aruco);
  printf("clone states: %d\n", clone_states);
  printf("grid size: %d x %d\n", grid_x, grid_y);
  printf("fast threshold: %d\n", fast_threshold);
  printf("min pixel distance: %d\n", min_px_dist);
  printf("downsize aruco image: %d\n", do_downsizing);

  // Fake camera info (we don't need this, as we are not using the normalized coordinates for anything)
  Eigen::Matrix<double, 8, 1> cam0_calib;
  cam0_calib << 1, 1, 0, 0, 0, 0, 0, 0;

  // Create our n-camera vectors
  std::map<size_t, bool> camera_fisheye;
  std::map<size_t, Eigen::VectorXd> camera_calibration;
  camera_fisheye.insert({0, false});
  camera_calibration.insert({0, cam0_calib});
  camera_fisheye.insert({1, false});
  camera_calibration.insert({1, cam0_calib});

  // Lets make a feature extractor
  extractor = new TrackKLT(num_pts, num_aruco, fast_threshold, grid_x, grid_y, min_px_dist);
  // extractor = new TrackDescriptor(num_pts,num_aruco,true,fast_threshold,grid_x,grid_y,knn_ratio);
  // extractor = new TrackAruco(num_aruco,true,do_downsizing);
  extractor->set_calibration(camera_calibration, camera_fisheye);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Open the first webcam (0=laptop cam, 1=usb device)
  cv::VideoCapture cap;
  if (!cap.open(0)) {
    printf(RED "Unable to open a webcam feed!\n" RESET);
    return EXIT_FAILURE;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Loop forever until we break out
  double current_time = 0.0;
  std::deque<double> clonetimes;
  while (true) {

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
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2RGB);

    // Else lets track this image
    extractor->feed_monocular(current_time, frame, 0);

    // Display the resulting tracks
    cv::Mat img_active, img_history;
    extractor->display_active(img_active, 255, 0, 0, 0, 0, 255);
    extractor->display_history(img_history, 0, 255, 255, 255, 255, 255);

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
