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

#include "TrackBase.h"

#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"

using namespace ov_core;

TrackBase::TrackBase(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numfeats, int numaruco, bool stereo,
                     HistogramMethod histmethod)
    : camera_calib(cameras), database(new FeatureDatabase()), num_features(numfeats), use_stereo(stereo), histogram_method(histmethod) {
  // Our current feature ID should be larger then the number of aruco tags we have (each has 4 corners)
  currid = 4 * (size_t)numaruco + 1;
  // Create our mutex array based on the number of cameras we have
  // See https://stackoverflow.com/a/24170141/7718197
  if (mtx_feeds.empty() || mtx_feeds.size() != camera_calib.size()) {
    std::vector<std::mutex> list(camera_calib.size());
    mtx_feeds.swap(list);
  }
}

void TrackBase::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay) {

  // Cache the images to prevent other threads from editing while we viz (which can be slow)
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last_cache;
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last_cache = img_last;
    img_mask_last_cache = img_mask_last;
    pts_last_cache = pts_last;
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
  if (img_last_cache.empty() || max_width == -1 || max_height == -1)
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
    // select the subset of the image
    cv::Mat img_temp;
    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
    // draw, loop through all keypoints
    for (size_t i = 0; i < pts_last_cache[pair.first].size(); i++) {
      // Get bounding pts for our boxes
      cv::Point2f pt_l = pts_last_cache[pair.first].at(i).pt;
      // Draw the extracted points and ID
      cv::circle(img_temp, pt_l, (is_small) ? 1 : 2, cv::Scalar(r1, g1, b1), cv::FILLED);
      // cv::putText(img_out, std::to_string(ids_left_last.at(i)), pt_l, cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255),1,cv::LINE_AA);
      // Draw rectangle around the point
      cv::Point2f pt_l_top = cv::Point2f(pt_l.x - 3, pt_l.y - 3);
      cv::Point2f pt_l_bot = cv::Point2f(pt_l.x + 3, pt_l.y + 3);
      cv::rectangle(img_temp, pt_l_top, pt_l_bot, cv::Scalar(r2, g2, b2), 1);
    }
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

void TrackBase::display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::vector<size_t> highlighted,
                                std::string overlay) {

  // Cache the images to prevent other threads from editing while we viz (which can be slow)
  std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
  std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last_cache;
  std::unordered_map<size_t, std::vector<size_t>> ids_last_cache;
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last_cache = img_last;
    img_mask_last_cache = img_mask_last;
    pts_last_cache = pts_last;
    ids_last_cache = ids_last;
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

  // Max tracks to show (otherwise it clutters up the screen)
  size_t maxtracks = 50;

  // Loop through each image, and draw
  int index_cam = 0;
  for (auto const &pair : img_last_cache) {
    // select the subset of the image
    cv::Mat img_temp;
    if (image_new)
      cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
    else
      img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
    // draw, loop through all keypoints
    for (size_t i = 0; i < ids_last_cache[pair.first].size(); i++) {
      // If a highlighted point, then put a nice box around it
      if (std::find(highlighted.begin(), highlighted.end(), ids_last_cache[pair.first].at(i)) != highlighted.end()) {
        cv::Point2f pt_c = pts_last_cache[pair.first].at(i).pt;
        cv::Point2f pt_l_top = cv::Point2f(pt_c.x - ((is_small) ? 3 : 5), pt_c.y - ((is_small) ? 3 : 5));
        cv::Point2f pt_l_bot = cv::Point2f(pt_c.x + ((is_small) ? 3 : 5), pt_c.y + ((is_small) ? 3 : 5));
        cv::rectangle(img_temp, pt_l_top, pt_l_bot, cv::Scalar(0, 255, 0), 1);
        cv::circle(img_temp, pt_c, (is_small) ? 1 : 2, cv::Scalar(0, 255, 0), cv::FILLED);
      }
      // Get the feature from the database
      Feature feat;
      if (!database->get_feature_clone(ids_last_cache[pair.first].at(i), feat))
        continue;
      if (feat.uvs.empty() || feat.uvs[pair.first].empty() || feat.to_delete)
        continue;
      // Draw the history of this point (start at the last inserted one)
      for (size_t z = feat.uvs[pair.first].size() - 1; z > 0; z--) {
        // Check if we have reached the max
        if (feat.uvs[pair.first].size() - z > maxtracks)
          break;
        // Calculate what color we are drawing in
        bool is_stereo = (feat.uvs.size() > 1);
        int color_r = (is_stereo ? b2 : r2) - (int)(1.0 * (is_stereo ? b1 : r1) / feat.uvs[pair.first].size() * z);
        int color_g = (is_stereo ? r2 : g2) - (int)(1.0 * (is_stereo ? r1 : g1) / feat.uvs[pair.first].size() * z);
        int color_b = (is_stereo ? g2 : b2) - (int)(1.0 * (is_stereo ? g1 : b1) / feat.uvs[pair.first].size() * z);
        // Draw current point
        cv::Point2f pt_c(feat.uvs[pair.first].at(z)(0), feat.uvs[pair.first].at(z)(1));
        cv::circle(img_temp, pt_c, (is_small) ? 1 : 2, cv::Scalar(color_r, color_g, color_b), cv::FILLED);
        // If there is a next point, then display the line from this point to the next
        if (z + 1 < feat.uvs[pair.first].size()) {
          cv::Point2f pt_n(feat.uvs[pair.first].at(z + 1)(0), feat.uvs[pair.first].at(z + 1)(1));
          cv::line(img_temp, pt_c, pt_n, cv::Scalar(color_r, color_g, color_b));
        }
        // If the first point, display the ID
        if (z == feat.uvs[pair.first].size() - 1) {
          // cv::putText(img_out0, std::to_string(feat->featid), pt_c, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1,
          // cv::LINE_AA); cv::circle(img_out0, pt_c, 2, cv::Scalar(color,color,255), CV_FILLED);
        }
      }
    }
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

void TrackBase::change_feat_id(size_t id_old, size_t id_new) {

  // If found in db then replace
  if (database->get_internal_data().find(id_old) != database->get_internal_data().end()) {
    std::shared_ptr<Feature> feat = database->get_internal_data().at(id_old);
    database->get_internal_data().erase(id_old);
    feat->featid = id_new;
    database->get_internal_data().insert({id_new, feat});
  }

  // Update current track IDs
  for (auto &cam_ids_pair : ids_last) {
    for (size_t i = 0; i < cam_ids_pair.second.size(); i++) {
      if (cam_ids_pair.second.at(i) == id_old) {
        ids_last.at(cam_ids_pair.first).at(i) = id_new;
      }
    }
  }
}