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

#include "TrackAruco.h"

#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "utils/opencv_lambda_body.h"

using namespace ov_core;

void TrackAruco::feed_new_camera(const CameraData &message) {

  // Error check that we have all the data
  if (message.sensor_ids.empty() || message.sensor_ids.size() != message.images.size() || message.images.size() != message.masks.size()) {
    PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
    PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
    PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
    PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
    std::exit(EXIT_FAILURE);
  }

  // There is not such thing as stereo tracking for aruco
  // Thus here we should just call the monocular version two times
#if ENABLE_ARUCO_TAGS
  size_t num_images = message.images.size();
  parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    perform_tracking(message.timestamp, message.images.at(i), message.sensor_ids.at(i), message.masks.at(i));
                  }
                }));
#else
  PRINT_ERROR(RED "[ERROR]: you have not compiled with aruco tag support!!!\n" RESET);
  std::exit(EXIT_FAILURE);
#endif
}

#if ENABLE_ARUCO_TAGS

void TrackAruco::perform_tracking(double timestamp, const cv::Mat &imgin, size_t cam_id, const cv::Mat &maskin) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

  // Histogram equalize
  cv::Mat img;
  if (histogram_method == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(imgin, img);
  } else if (histogram_method == HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(imgin, img);
  } else {
    img = imgin;
  }

  // Clear the old data from the last timestep
  ids_aruco[cam_id].clear();
  corners[cam_id].clear();
  rejects[cam_id].clear();

  // If we are downsizing, then downsize
  cv::Mat img0;
  if (do_downsizing) {
    cv::pyrDown(img, img0, cv::Size(img.cols / 2, img.rows / 2));
  } else {
    img0 = img;
  }

  //===================================================================================
  //===================================================================================

  // Perform extraction
#if CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
  aruco_detector.detectMarkers(img0, corners[cam_id], ids_aruco[cam_id], rejects[cam_id]);
#else
  cv::aruco::detectMarkers(img0, aruco_dict, corners[cam_id], ids_aruco[cam_id], aruco_params, rejects[cam_id]);
#endif
  rT2 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // If we downsized, scale all our u,v measurements by a factor of two
  // Note: we do this so we can use these results for visulization later
  // Note: and so that the uv added is in the same image size
  if (do_downsizing) {
    for (size_t i = 0; i < corners[cam_id].size(); i++) {
      for (size_t j = 0; j < corners[cam_id].at(i).size(); j++) {
        corners[cam_id].at(i).at(j).x *= 2;
        corners[cam_id].at(i).at(j).y *= 2;
      }
    }
    for (size_t i = 0; i < rejects[cam_id].size(); i++) {
      for (size_t j = 0; j < rejects[cam_id].at(i).size(); j++) {
        rejects[cam_id].at(i).at(j).x *= 2;
        rejects[cam_id].at(i).at(j).y *= 2;
      }
    }
  }

  //===================================================================================
  //===================================================================================

  // ID vectors, of all currently tracked IDs
  std::vector<size_t> ids_new;
  std::vector<cv::KeyPoint> pts_new;

  // Append to our feature database this new information
  for (size_t i = 0; i < ids_aruco[cam_id].size(); i++) {
    // Skip if ID is greater then our max
    if (ids_aruco[cam_id].at(i) > max_tag_id)
      continue;
    // Assert we have 4 points (we will only use one of them)
    assert(corners[cam_id].at(i).size() == 4);
    for (int n = 0; n < 4; n++) {
      // Check if it is in the mask
      // NOTE: mask has max value of 255 (white) if it should be
      if (maskin.at<uint8_t>((int)corners[cam_id].at(i).at(n).y, (int)corners[cam_id].at(i).at(n).x) > 127)
        continue;
      // Try to undistort the point
      cv::Point2f npt_l = camera_calib.at(cam_id)->undistort_cv(corners[cam_id].at(i).at(n));
      // Append to the ids vector and database
      size_t tmp_id = (size_t)ids_aruco[cam_id].at(i) + n * max_tag_id;
      database->update_feature(tmp_id, timestamp, cam_id, corners[cam_id].at(i).at(n).x, corners[cam_id].at(i).at(n).y, npt_l.x, npt_l.y);
      // Append to active tracked point list
      cv::KeyPoint kpt;
      kpt.pt = corners[cam_id].at(i).at(n);
      ids_new.push_back(tmp_id);
      pts_new.push_back(kpt);
    }
  }

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = maskin;
    ids_last[cam_id] = ids_new;
    pts_last[cam_id] = pts_new;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  PRINT_ALL("[TIME-ARUCO]: %.4f seconds for detection\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-ARUCO]: %.4f seconds for feature DB update (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6,
            (int)ids_new.size());
  PRINT_ALL("[TIME-ARUCO]: %.4f seconds for total\n", (rT3 - rT1).total_microseconds() * 1e-6);
}

void TrackAruco::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay) {

  // Cache the images to prevent other threads from editing while we viz (which can be slow)
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last_cache = img_last;
    img_mask_last_cache = img_mask_last;
  }
  auto ids_aruco_cache = ids_aruco;
  auto corners_cache = corners;
  auto rejects_cache = rejects;

  // Get the largest width and height
  int max_width = -1;
  int max_height = -1;
  for (auto const &pair : img_last_cache) {
    if (max_width < pair.second.cols)
      max_width = pair.second.cols;
    if (max_height < pair.second.rows)
      max_height = pair.second.rows;
  }

  // Return if we didn't have a last image
  if (img_last_cache.empty() || max_width == -1 || max_height == -1)
    return;

  // If the image is "small" thus we shoudl use smaller display codes
  bool is_small = (std::min(max_width, max_height) < 400);

  // If the image is "new" then draw the images from scratch
  // Otherwise, we grab the subset of the main image and draw on top of it
  bool image_new = ((int)img_last_cache.size() * max_width != img_out.cols || max_height != img_out.rows);

  // If new, then resize the current image
  if (image_new)
    img_out = cv::Mat(max_height, (int)img_last_cache.size() * max_width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Loop through each image, and draw
  int index_cam = 0;
  for (auto const &pair : img_last_cache) {
    // select the subset of the image
    cv::Mat img_temp;
    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
    // draw...
    if (!ids_aruco_cache[pair.first].empty())
      cv::aruco::drawDetectedMarkers(img_temp, corners_cache[pair.first], ids_aruco_cache[pair.first]);
    if (!rejects_cache[pair.first].empty())
      cv::aruco::drawDetectedMarkers(img_temp, rejects_cache[pair.first], cv::noArray(), cv::Scalar(100, 0, 255));
    // Draw what camera this is
    auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
    if (overlay == "") {
      cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0,
                  cv::Scalar(0, 255, 0), 3);
    } else {
      cv::putText(img_temp, overlay, txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 0, 255), 3);
    }
    // Overlay the mask
    cv::Mat mask = cv::Mat::zeros(img_mask_last_cache[pair.first].rows, img_mask_last_cache[pair.first].cols, CV_8UC3);
    mask.setTo(cv::Scalar(0, 0, 255), img_mask_last_cache[pair.first]);
    cv::addWeighted(mask, 0.1, img_temp, 1.0, 0.0, img_temp);
    // Replace the output image
    img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
    index_cam++;
  }
}

#endif