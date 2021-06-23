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

#include "TrackBase.h"

using namespace ov_core;

void TrackBase::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) {

  // Cache the images to prevent other threads from editing while we viz (which can be slow)
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  for (auto const &pair : img_last) {
    img_last_cache.insert({pair.first, pair.second.clone()});
  }
  for (auto const &pair : img_mask_last) {
    img_mask_last_cache.insert({pair.first, pair.second.clone()});
  }

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
  if (max_width == -1 || max_height == -1)
    return;

  // If the image is "small" thus we should use smaller display codes
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
    // Lock this image
    std::unique_lock<std::mutex> lck(mtx_feeds.at(pair.first));
    // select the subset of the image
    cv::Mat img_temp;
    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
    // draw, loop through all keypoints
    for (size_t i = 0; i < pts_last[pair.first].size(); i++) {
      // Get bounding pts for our boxes
      cv::Point2f pt_l = pts_last[pair.first].at(i).pt;
      // Draw the extracted points and ID
      cv::circle(img_temp, pt_l, (is_small) ? 1 : 2, cv::Scalar(r1, g1, b1), cv::FILLED);
      // cv::putText(img_out, std::to_string(ids_left_last.at(i)), pt_l, cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255),1,cv::LINE_AA);
      // Draw rectangle around the point
      cv::Point2f pt_l_top = cv::Point2f(pt_l.x - 5, pt_l.y - 5);
      cv::Point2f pt_l_bot = cv::Point2f(pt_l.x + 5, pt_l.y + 5);
      cv::rectangle(img_temp, pt_l_top, pt_l_bot, cv::Scalar(r2, g2, b2), 1);
    }
    // Draw what camera this is
    auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
    cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0,
                cv::Scalar(0, 255, 0), 3);
    // Overlay the mask
    cv::Mat mask = cv::Mat::zeros(img_mask_last_cache[pair.first].rows, img_mask_last_cache[pair.first].cols, CV_8UC3);
    mask.setTo(cv::Scalar(0, 0, 255), img_mask_last_cache[pair.first]);
    cv::addWeighted(mask, 0.1, img_temp, 1.0, 0.0, img_temp);
    // Replace the output image
    img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
    index_cam++;
  }
}

void TrackBase::display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::vector<size_t> highlighted) {

  // Cache the images to prevent other threads from editing while we viz (which can be slow)
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  for (auto const &pair : img_last) {
    img_last_cache.insert({pair.first, pair.second.clone()});
  }
  for (auto const &pair : img_mask_last) {
    img_mask_last_cache.insert({pair.first, pair.second.clone()});
  }

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
  if (max_width == -1 || max_height == -1)
    return;

  // If the image is "small" thus we shoudl use smaller display codes
  bool is_small = (std::min(max_width, max_height) < 400);

  // If the image is "new" then draw the images from scratch
  // Otherwise, we grab the subset of the main image and draw on top of it
  bool image_new = ((int)img_last_cache.size() * max_width != img_out.cols || max_height != img_out.rows);

  // If new, then resize the current image
  if (image_new)
    img_out = cv::Mat(max_height, (int)img_last_cache.size() * max_width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Max tracks to show (otherwise it clutters up the screen)
  size_t maxtracks = 50;

  // Loop through each image, and draw
  int index_cam = 0;
  for (auto const &pair : img_last_cache) {
    // Lock this image
    std::unique_lock<std::mutex> lck(mtx_feeds.at(pair.first));
    // select the subset of the image
    cv::Mat img_temp;
    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
    // draw, loop through all keypoints
    for (size_t i = 0; i < ids_last[pair.first].size(); i++) {
      // If a highlighted point, then put a nice box around it
      if (std::find(highlighted.begin(), highlighted.end(), ids_last[pair.first].at(i)) != highlighted.end()) {
        cv::Point2f pt_c = pts_last[pair.first].at(i).pt;
        cv::Point2f pt_l_top = cv::Point2f(pt_c.x - ((is_small) ? 3 : 5), pt_c.y - ((is_small) ? 3 : 5));
        cv::Point2f pt_l_bot = cv::Point2f(pt_c.x + ((is_small) ? 3 : 5), pt_c.y + ((is_small) ? 3 : 5));
        cv::rectangle(img_temp, pt_l_top, pt_l_bot, cv::Scalar(0, 255, 0), 1);
        cv::circle(img_temp, pt_c, (is_small) ? 1 : 2, cv::Scalar(0, 255, 0), cv::FILLED);
      }
      // Get the feature from the database
      std::shared_ptr<Feature> feat = database->get_feature(ids_last[pair.first].at(i));
      // Skip if the feature is null
      if (feat == nullptr || feat->uvs[pair.first].empty() || feat->to_delete)
        continue;
      // Draw the history of this point (start at the last inserted one)
      for (size_t z = feat->uvs[pair.first].size() - 1; z > 0; z--) {
        // Check if we have reached the max
        if (feat->uvs[pair.first].size() - z > maxtracks)
          break;
        // Calculate what color we are drawing in
        bool is_stereo = (feat->uvs.size() > 1);
        int color_r = (is_stereo ? b2 : r2) - (int)((is_stereo ? b1 : r1) / feat->uvs[pair.first].size() * z);
        int color_g = (is_stereo ? r2 : g2) - (int)((is_stereo ? r1 : g1) / feat->uvs[pair.first].size() * z);
        int color_b = (is_stereo ? g2 : b2) - (int)((is_stereo ? g1 : b1) / feat->uvs[pair.first].size() * z);
        // Draw current point
        cv::Point2f pt_c(feat->uvs[pair.first].at(z)(0), feat->uvs[pair.first].at(z)(1));
        cv::circle(img_temp, pt_c, (is_small) ? 1 : 2, cv::Scalar(color_r, color_g, color_b), cv::FILLED);
        // If there is a next point, then display the line from this point to the next
        if (z + 1 < feat->uvs[pair.first].size()) {
          cv::Point2f pt_n(feat->uvs[pair.first].at(z + 1)(0), feat->uvs[pair.first].at(z + 1)(1));
          cv::line(img_temp, pt_c, pt_n, cv::Scalar(color_r, color_g, color_b));
        }
        // If the first point, display the ID
        if (z == feat->uvs[pair.first].size() - 1) {
          // cv::putText(img_out0, std::to_string(feat->featid), pt_c, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
          // cv::LINE_AA); cv::circle(img_out0, pt_c, 2, cv::Scalar(color,color,255), CV_FILLED);
        }
      }
    }
    // Draw what camera this is
    auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
    cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0,
                cv::Scalar(0, 255, 0), 3);
    // Overlay the mask
    cv::Mat mask = cv::Mat::zeros(img_mask_last_cache[pair.first].rows, img_mask_last_cache[pair.first].cols, CV_8UC3);
    mask.setTo(cv::Scalar(0, 0, 255), img_mask_last_cache[pair.first]);
    cv::addWeighted(mask, 0.1, img_temp, 1.0, 0.0, img_temp);
    // Replace the output image
    img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
    index_cam++;
  }
}
