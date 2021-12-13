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

#include "TrackDescriptor.h"
#include "utils/print.h"

using namespace ov_core;

void TrackDescriptor::feed_new_camera(const CameraData &message) {

  // Error check that we have all the data
  if (message.sensor_ids.empty() || message.sensor_ids.size() != message.images.size() || message.images.size() != message.masks.size()) {
    PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
    PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
    PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
    PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
    std::exit(EXIT_FAILURE);
  }

  // Either call our stereo or monocular version
  // If we are doing binocular tracking, then we should parallize our tracking
  size_t num_images = message.images.size();
  if (num_images == 1) {
    feed_monocular(message, 0);
  } else if (num_images == 2 && use_stereo) {
    feed_stereo(message, 0, 1);
  } else if (!use_stereo) {
    parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
                    for (int i = range.start; i < range.end; i++) {
                      feed_monocular(message, i);
                    }
                  }));
  } else {
    PRINT_ERROR(RED "[ERROR]: invalid number of images passed %zu, we only support mono or stereo tracking", num_images);
    std::exit(EXIT_FAILURE);
  }
}

void TrackDescriptor::feed_monocular(const CameraData &message, size_t msg_id) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  size_t cam_id = message.sensor_ids.at(msg_id);
  std::unique_lock<std::mutex> lck(mtx_feeds.at(cam_id));

  // Histogram equalize
  cv::Mat img, mask;
  if (histogram_method == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(message.images.at(msg_id), img);
  } else if (histogram_method == HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(message.images.at(msg_id), img);
  } else {
    img = message.images.at(msg_id);
  }
  mask = message.masks.at(msg_id);

  // If we are the first frame (or have lost tracking), initialize our descriptors
  if (pts_last.find(cam_id) == pts_last.end() || pts_last[cam_id].empty()) {
    perform_detection_monocular(img, mask, pts_last[cam_id], desc_last[cam_id], ids_last[cam_id]);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = mask;
    return;
  }

  // Our new keypoints and descriptor for the new image
  std::vector<cv::KeyPoint> pts_new;
  cv::Mat desc_new;
  std::vector<size_t> ids_new;

  // First, extract new descriptors for this new image
  perform_detection_monocular(img, mask, pts_new, desc_new, ids_new);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Our matches temporally
  std::vector<cv::DMatch> matches_ll;

  // Lets match temporally
  robust_match(pts_last[cam_id], pts_new, desc_last[cam_id], desc_new, cam_id, cam_id, matches_ll);
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left;
  std::vector<size_t> good_ids_left;
  cv::Mat good_desc_left;

  // Count how many we have tracked from the last time
  int num_tracklast = 0;

  // Loop through all current left to right points
  // We want to see if any of theses have matches to the previous frame
  // If we have a match new->old then we want to use that ID instead of the new one
  for (size_t i = 0; i < pts_new.size(); i++) {

    // Loop through all left matches, and find the old "train" id
    int idll = -1;
    for (size_t j = 0; j < matches_ll.size(); j++) {
      if (matches_ll[j].trainIdx == (int)i) {
        idll = matches_ll[j].queryIdx;
      }
    }

    // Then lets replace the current ID with the old ID if found
    // Else just append the current feature and its unique ID
    good_left.push_back(pts_new[i]);
    good_desc_left.push_back(desc_new.row((int)i));
    if (idll != -1) {
      good_ids_left.push_back(ids_last[cam_id][idll]);
      num_tracklast++;
    } else {
      good_ids_left.push_back(ids_new[i]);
    }
  }
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l = camera_calib.at(cam_id)->undistort_cv(good_left.at(i).pt);
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x, npt_l.y);
  }

  // Debug info
  // PRINT_DEBUG("LtoL = %d | good = %d | fromlast = %d\n",(int)matches_ll.size(),(int)good_left.size(),num_tracklast);

  // Move forward in time
  img_last[cam_id] = img;
  img_mask_last[cam_id] = mask;
  pts_last[cam_id] = good_left;
  ids_last[cam_id] = good_ids_left;
  desc_last[cam_id] = good_desc_left;
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Our timing information
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for detection\n",(rT2-rT1).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for matching\n",(rT3-rT2).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for merging\n",(rT4-rT3).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for feature DB update (%d features)\n",(rT5-rT4).total_microseconds() * 1e-6,
  // (int)good_left.size()); PRINT_DEBUG("[TIME-DESC]: %.4f seconds for total\n",(rT5-rT1).total_microseconds() * 1e-6);
}

void TrackDescriptor::feed_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  size_t cam_id_left = message.sensor_ids.at(msg_id_left);
  size_t cam_id_right = message.sensor_ids.at(msg_id_right);
  std::unique_lock<std::mutex> lck1(mtx_feeds.at(cam_id_left));
  std::unique_lock<std::mutex> lck2(mtx_feeds.at(cam_id_right));

  // Histogram equalize images
  cv::Mat img_left, img_right, mask_left, mask_right;
  if (histogram_method == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(message.images.at(msg_id_left), img_left);
    cv::equalizeHist(message.images.at(msg_id_right), img_right);
  } else if (histogram_method == HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(message.images.at(msg_id_left), img_left);
    clahe->apply(message.images.at(msg_id_right), img_right);
  } else {
    img_left = message.images.at(msg_id_left);
    img_right = message.images.at(msg_id_right);
  }
  mask_left = message.masks.at(msg_id_left);
  mask_right = message.masks.at(msg_id_right);

  // If we are the first frame (or have lost tracking), initialize our descriptors
  if (pts_last[cam_id_left].empty() || pts_last[cam_id_right].empty()) {
    perform_detection_stereo(img_left, img_right, mask_left, mask_right, pts_last[cam_id_left], pts_last[cam_id_right],
                             desc_last[cam_id_left], desc_last[cam_id_right], cam_id_left, cam_id_right, ids_last[cam_id_left],
                             ids_last[cam_id_right]);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    return;
  }

  // Our new keypoints and descriptor for the new image
  std::vector<cv::KeyPoint> pts_left_new, pts_right_new;
  cv::Mat desc_left_new, desc_right_new;
  std::vector<size_t> ids_left_new, ids_right_new;

  // First, extract new descriptors for this new image
  perform_detection_stereo(img_left, img_right, mask_left, mask_right, pts_left_new, pts_right_new, desc_left_new, desc_right_new,
                           cam_id_left, cam_id_right, ids_left_new, ids_right_new);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Our matches temporally
  std::vector<cv::DMatch> matches_ll, matches_rr;
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    robust_match(pts_last[is_left ? cam_id_left : cam_id_right], is_left ? pts_left_new : pts_right_new,
                                 desc_last[is_left ? cam_id_left : cam_id_right], is_left ? desc_left_new : desc_right_new,
                                 is_left ? cam_id_left : cam_id_right, is_left ? cam_id_left : cam_id_right,
                                 is_left ? matches_ll : matches_rr);
                  }
                }));
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left, good_right;
  std::vector<size_t> good_ids_left, good_ids_right;
  cv::Mat good_desc_left, good_desc_right;

  // Points must be of equal size
  assert(pts_last[cam_id_left].size() == pts_last[cam_id_right].size());
  assert(pts_left_new.size() == pts_right_new.size());

  // Count how many we have tracked from the last time
  int num_tracklast = 0;

  // Loop through all current left to right points
  // We want to see if any of theses have matches to the previous frame
  // If we have a match new->old then we want to use that ID instead of the new one
  for (size_t i = 0; i < pts_left_new.size(); i++) {

    // Loop through all left matches, and find the old "train" id
    int idll = -1;
    for (size_t j = 0; j < matches_ll.size(); j++) {
      if (matches_ll[j].trainIdx == (int)i) {
        idll = matches_ll[j].queryIdx;
      }
    }

    // Loop through all left matches, and find the old "train" id
    int idrr = -1;
    for (size_t j = 0; j < matches_rr.size(); j++) {
      if (matches_rr[j].trainIdx == (int)i) {
        idrr = matches_rr[j].queryIdx;
      }
    }

    // If we found a good stereo track from left to left, and right to right
    // Then lets replace the current ID with the old ID
    // We also check that we are linked to the same past ID value
    if (idll != -1 && idrr != -1 && ids_last[cam_id_left][idll] == ids_last[cam_id_right][idrr]) {
      good_left.push_back(pts_left_new[i]);
      good_right.push_back(pts_right_new[i]);
      good_desc_left.push_back(desc_left_new.row((int)i));
      good_desc_right.push_back(desc_right_new.row((int)i));
      good_ids_left.push_back(ids_last[cam_id_left][idll]);
      good_ids_right.push_back(ids_last[cam_id_right][idrr]);
      num_tracklast++;
    } else {
      // Else just append the current feature and its unique ID
      good_left.push_back(pts_left_new[i]);
      good_right.push_back(pts_right_new[i]);
      good_desc_left.push_back(desc_left_new.row((int)i));
      good_desc_right.push_back(desc_right_new.row((int)i));
      good_ids_left.push_back(ids_left_new[i]);
      good_ids_right.push_back(ids_left_new[i]);
    }
  }
  rT4 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    // Assert that our IDs are the same
    assert(good_ids_left.at(i) == good_ids_right.at(i));
    // Try to undistort the point
    cv::Point2f npt_l = camera_calib.at(cam_id_left)->undistort_cv(good_left.at(i).pt);
    cv::Point2f npt_r = camera_calib.at(cam_id_right)->undistort_cv(good_right.at(i).pt);
    // Append to the database
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id_left, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x,
                             npt_l.y);
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id_right, good_right.at(i).pt.x, good_right.at(i).pt.y, npt_r.x,
                             npt_r.y);
  }

  // Debug info
  // PRINT_DEBUG("LtoL = %d | RtoR = %d | LtoR = %d | good = %d | fromlast = %d\n", (int)matches_ll.size(),
  //       (int)matches_rr.size(),(int)ids_left_new.size(),(int)good_left.size(),num_tracklast);

  // Move forward in time
  img_last[cam_id_left] = img_left;
  img_last[cam_id_right] = img_right;
  img_mask_last[cam_id_left] = mask_left;
  img_mask_last[cam_id_right] = mask_right;
  pts_last[cam_id_left] = good_left;
  pts_last[cam_id_right] = good_right;
  ids_last[cam_id_left] = good_ids_left;
  ids_last[cam_id_right] = good_ids_right;
  desc_last[cam_id_left] = good_desc_left;
  desc_last[cam_id_right] = good_desc_right;
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Our timing information
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for detection\n",(rT2-rT1).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for matching\n",(rT3-rT2).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for merging\n",(rT4-rT3).total_microseconds() * 1e-6);
  // PRINT_DEBUG("[TIME-DESC]: %.4f seconds for feature DB update (%d features)\n",(rT5-rT4).total_microseconds() * 1e-6,
  // (int)good_left.size()); PRINT_DEBUG("[TIME-DESC]: %.4f seconds for total\n",(rT5-rT1).total_microseconds() * 1e-6);
}

void TrackDescriptor::perform_detection_monocular(const cv::Mat &img0, const cv::Mat &mask0, std::vector<cv::KeyPoint> &pts0,
                                                  cv::Mat &desc0, std::vector<size_t> &ids0) {

  // Assert that we need features
  assert(pts0.empty());

  // Extract our features (use FAST with griding)
  std::vector<cv::KeyPoint> pts0_ext;
  Grider_FAST::perform_griding(img0, mask0, pts0_ext, num_features, grid_x, grid_y, threshold, true);

  // For all new points, extract their descriptors
  cv::Mat desc0_ext;
  this->orb0->compute(img0, pts0_ext, desc0_ext);

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less then grid_px_size points away then existing features
  cv::Size size((int)((float)img0.cols / (float)min_px_dist), (int)((float)img0.rows / (float)min_px_dist));
  cv::Mat grid_2d = cv::Mat::zeros(size, CV_8UC1);

  // For all good matches, lets append to our returned vectors
  // NOTE: if we multi-thread this atomic can cause some randomness due to multiple thread detecting features
  // NOTE: this is due to the fact that we select update features based on feat id
  // NOTE: thus the order will matter since we try to select oldest (smallest id) to update with
  // NOTE: not sure how to remove... maybe a better way?
  for (size_t i = 0; i < pts0_ext.size(); i++) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = pts0_ext.at(i);
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int x_grid = (int)(kpt.pt.x / (float)min_px_dist);
    int y_grid = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_grid < 0 || x_grid >= size.width || y_grid < 0 || y_grid >= size.height || x < 0 || x >= img0.cols || y < 0 || y >= img0.rows) {
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d.at<uint8_t>(y_grid, x_grid) > 127)
      continue;
    // Else we are good, append our keypoints and descriptors
    pts0.push_back(pts0_ext.at(i));
    desc0.push_back(desc0_ext.row((int)i));
    // Set our IDs to be unique IDs here, will later replace with corrected ones, after temporal matching
    size_t temp = ++currid;
    ids0.push_back(temp);
    grid_2d.at<uint8_t>(y_grid, x_grid) = 255;
  }
}

void TrackDescriptor::perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, const cv::Mat &mask0, const cv::Mat &mask1,
                                               std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1, cv::Mat &desc0,
                                               cv::Mat &desc1, size_t cam_id0, size_t cam_id1, std::vector<size_t> &ids0,
                                               std::vector<size_t> &ids1) {

  // Assert that we need features
  assert(pts0.empty());
  assert(pts1.empty());

  // Extract our features (use FAST with griding), and their descriptors
  std::vector<cv::KeyPoint> pts0_ext, pts1_ext;
  cv::Mat desc0_ext, desc1_ext;
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    Grider_FAST::perform_griding(is_left ? img0 : img1, is_left ? mask0 : mask1, is_left ? pts0_ext : pts1_ext,
                                                 num_features, grid_x, grid_y, threshold, true);
                    (is_left ? orb0 : orb1)->compute(is_left ? img0 : img1, is_left ? pts0_ext : pts1_ext, is_left ? desc0_ext : desc1_ext);
                  }
                }));

  // Do matching from the left to the right image
  std::vector<cv::DMatch> matches;
  robust_match(pts0_ext, pts1_ext, desc0_ext, desc1_ext, cam_id0, cam_id1, matches);

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less then grid_px_size points away then existing features
  cv::Size size0((int)((float)img0.cols / (float)min_px_dist), (int)((float)img0.rows / (float)min_px_dist));
  cv::Mat grid_2d_0 = cv::Mat::zeros(size0, CV_8UC1);
  cv::Size size1((int)((float)img1.cols / (float)min_px_dist), (int)((float)img1.rows / (float)min_px_dist));
  cv::Mat grid_2d_1 = cv::Mat::zeros(size1, CV_8UC1);

  // For all good matches, lets append to our returned vectors
  for (size_t i = 0; i < matches.size(); i++) {

    // Get our ids
    int index_pt0 = matches.at(i).queryIdx;
    int index_pt1 = matches.at(i).trainIdx;

    // Get current left/right keypoint, check that it is in bounds
    cv::KeyPoint kpt0 = pts0_ext.at(index_pt0);
    cv::KeyPoint kpt1 = pts1_ext.at(index_pt1);
    int x0 = (int)kpt0.pt.x;
    int y0 = (int)kpt0.pt.y;
    int x0_grid = (int)(kpt0.pt.x / (float)min_px_dist);
    int y0_grid = (int)(kpt0.pt.y / (float)min_px_dist);
    if (x0_grid < 0 || x0_grid >= size0.width || y0_grid < 0 || y0_grid >= size0.height || x0 < 0 || x0 >= img0.cols || y0 < 0 ||
        y0 >= img0.rows) {
      continue;
    }
    int x1 = (int)kpt1.pt.x;
    int y1 = (int)kpt1.pt.y;
    int x1_grid = (int)(kpt1.pt.x / (float)min_px_dist);
    int y1_grid = (int)(kpt1.pt.y / (float)min_px_dist);
    if (x1_grid < 0 || x1_grid >= size1.width || y1_grid < 0 || y1_grid >= size1.height || x1 < 0 || x1 >= img0.cols || y1 < 0 ||
        y1 >= img0.rows) {
      continue;
    }

    // Check if this keypoint is near another point
    if (grid_2d_0.at<uint8_t>(y0_grid, x0_grid) > 127 || grid_2d_1.at<uint8_t>(y1_grid, x1_grid) > 127)
      continue;

    // Append our keypoints and descriptors
    pts0.push_back(pts0_ext[index_pt0]);
    pts1.push_back(pts1_ext[index_pt1]);
    desc0.push_back(desc0_ext.row(index_pt0));
    desc1.push_back(desc1_ext.row(index_pt1));

    // Set our IDs to be unique IDs here, will later replace with corrected ones, after temporal matching
    size_t temp = ++currid;
    ids0.push_back(temp);
    ids1.push_back(temp);
  }
}

void TrackDescriptor::robust_match(std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> pts1, cv::Mat &desc0, cv::Mat &desc1,
                                   size_t id0, size_t id1, std::vector<cv::DMatch> &matches) {

  // Our 1to2 and 2to1 match vectors
  std::vector<std::vector<cv::DMatch>> matches0to1, matches1to0;

  // Match descriptors (return 2 nearest neighbours)
  matcher->knnMatch(desc0, desc1, matches0to1, 2);
  matcher->knnMatch(desc1, desc0, matches1to0, 2);

  // Do a ratio test for both matches
  robust_ratio_test(matches0to1);
  robust_ratio_test(matches1to0);

  // Finally do a symmetry test
  std::vector<cv::DMatch> matches_good;
  robust_symmetry_test(matches0to1, matches1to0, matches_good);

  // Convert points into points for RANSAC
  std::vector<cv::Point2f> pts0_rsc, pts1_rsc;
  for (size_t i = 0; i < matches_good.size(); i++) {
    // Get our ids
    int index_pt0 = matches_good.at(i).queryIdx;
    int index_pt1 = matches_good.at(i).trainIdx;
    // Push back just the 2d point
    pts0_rsc.push_back(pts0[index_pt0].pt);
    pts1_rsc.push_back(pts1[index_pt1].pt);
  }

  // If we don't have enough points for ransac just return empty
  if (pts0_rsc.size() < 10)
    return;

  // Normalize these points, so we can then do ransac
  // We don't want to do ransac on distorted image uvs since the mapping is nonlinear
  std::vector<cv::Point2f> pts0_n, pts1_n;
  for (size_t i = 0; i < pts0_rsc.size(); i++) {
    pts0_n.push_back(camera_calib.at(id0)->undistort_cv(pts0_rsc.at(i)));
    pts1_n.push_back(camera_calib.at(id1)->undistort_cv(pts1_rsc.at(i)));
  }

  // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
  std::vector<uchar> mask_rsc;
  double max_focallength_img0 = std::max(camera_calib.at(id0)->get_K()(0, 0), camera_calib.at(id0)->get_K()(1, 1));
  double max_focallength_img1 = std::max(camera_calib.at(id1)->get_K()(0, 0), camera_calib.at(id1)->get_K()(1, 1));
  double max_focallength = std::max(max_focallength_img0, max_focallength_img1);
  cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 1 / max_focallength, 0.999, mask_rsc);

  // Loop through all good matches, and only append ones that have passed RANSAC
  for (size_t i = 0; i < matches_good.size(); i++) {
    // Skip if bad ransac id
    if (mask_rsc[i] != 1)
      continue;
    // Else, lets append this match to the return array!
    matches.push_back(matches_good.at(i));
  }
}

void TrackDescriptor::robust_ratio_test(std::vector<std::vector<cv::DMatch>> &matches) {
  // Loop through all matches
  for (auto &match : matches) {
    // If 2 NN has been identified, else remove this feature
    if (match.size() > 1) {
      // check distance ratio, remove it if the ratio is larger
      if (match[0].distance / match[1].distance > knn_ratio) {
        match.clear();
      }
    } else {
      // does not have 2 neighbours, so remove it
      match.clear();
    }
  }
}

void TrackDescriptor::robust_symmetry_test(std::vector<std::vector<cv::DMatch>> &matches1, std::vector<std::vector<cv::DMatch>> &matches2,
                                           std::vector<cv::DMatch> &good_matches) {
  // for all matches image 1 -> image 2
  for (auto &match1 : matches1) {
    // ignore deleted matches
    if (match1.empty() || match1.size() < 2)
      continue;
    // for all matches image 2 -> image 1
    for (auto &match2 : matches2) {
      // ignore deleted matches
      if (match2.empty() || match2.size() < 2)
        continue;
      // Match symmetry test
      if (match1[0].queryIdx == match2[0].trainIdx && match2[0].queryIdx == match1[0].trainIdx) {
        // add symmetrical match
        good_matches.emplace_back(cv::DMatch(match1[0].queryIdx, match1[0].trainIdx, match1[0].distance));
        // next match in image 1 -> image 2
        break;
      }
    }
  }
}
