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


#include "TrackKLT.h"

using namespace ov_core;

void TrackKLT::feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  std::unique_lock<std::mutex> lck(mtx_feeds.at(cam_id));

  // Histogram equalize
  cv::equalizeHist(img, img);
  // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
  // clahe->apply(img, img);

  // Extract the new image pyramid
  std::vector<cv::Mat> imgpyr;
  cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // If we didn't have any successful tracks last time, just extract this time
  // This also handles, the tracking initalization on the first call to this extractor
  if (pts_last[cam_id].empty()) {
    // Detect new features
    perform_detection_monocular(imgpyr, pts_last[cam_id], ids_last[cam_id]);
    // Save the current image and pyramid
    img_last[cam_id] = img.clone();
    img_pyramid_last[cam_id] = imgpyr;
    return;
  }

  // First we should make that the last images have enough features so we can do KLT
  // This will "top-off" our number of tracks so always have a constant number
  perform_detection_monocular(img_pyramid_last[cam_id], pts_last[cam_id], ids_last[cam_id]);
  rT3 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // Debug
  // printf("current points = %d,%d\n",(int)pts_left_last.size(),(int)pts_right_last.size());

  // Our return success masks, and predicted new features
  std::vector<uchar> mask_ll;
  std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id];

  // Lets track temporally
  perform_matching(img_pyramid_last[cam_id], imgpyr, pts_last[cam_id], pts_left_new, cam_id, cam_id, mask_ll);
  assert(pts_left_new.size()==ids_last[cam_id].size());
  rT4 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // If any of our mask is empty, that means we didn't have enough to do ransac, so just return
  if (mask_ll.empty()) {
    img_last[cam_id] = img.clone();
    img_pyramid_last[cam_id] = imgpyr;
    pts_last[cam_id].clear();
    ids_last[cam_id].clear();
    printf(RED "[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....\n" RESET);
    return;
  }

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left;
  std::vector<size_t> good_ids_left;

  // Loop through all left points
  for (size_t i = 0; i < pts_left_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_left_new.at(i).pt.x < 0 || pts_left_new.at(i).pt.y < 0 || (int)pts_left_new.at(i).pt.x > img.cols ||
        (int)pts_left_new.at(i).pt.y > img.rows)
      continue;
    // If it is a good track, and also tracked from left to right
    if (mask_ll[i]) {
      good_left.push_back(pts_left_new[i]);
      good_ids_left.push_back(ids_last[cam_id][i]);
    }
  }

  //===================================================================================
  //===================================================================================

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id);
    database->update_feature(good_ids_left.at(i), timestamp, cam_id, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x, npt_l.y);
  }

  // Move forward in time
  img_last[cam_id] = img.clone();
  img_pyramid_last[cam_id] = imgpyr;
  pts_last[cam_id] = good_left;
  ids_last[cam_id] = good_ids_left;
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  // printf("[TIME-KLT]: %.4f seconds for pyramid\n",(rT2-rT1).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for detection\n",(rT3-rT2).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for temporal klt\n",(rT4-rT3).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for feature DB update (%d features)\n",(rT5-rT4).total_microseconds() * 1e-6, (int)good_left.size());
  // printf("[TIME-KLT]: %.4f seconds for total\n",(rT5-rT1).total_microseconds() * 1e-6);
}

void TrackKLT::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Lock this data feed for this camera
  std::unique_lock<std::mutex> lck1(mtx_feeds.at(cam_id_left));
  std::unique_lock<std::mutex> lck2(mtx_feeds.at(cam_id_right));

  // Histogram equalize and then
  // Extract image pyramids (boost seems to require us to put all the arguments even if there are defaults....)
  cv::Mat img_left, img_right;
  std::vector<cv::Mat> imgpyr_left, imgpyr_right;
  // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    // Histogram equalize
                    cv::equalizeHist(is_left ? img_leftin : img_rightin, is_left ? img_left : img_right);
                    // clahe->apply((is_left ? img_leftin : img_rightin), (is_left ? img_left : img_right));
                    // Extract image pyramids (boost seems to require us to put all the arguments even if there are defaults....)
                    cv::buildOpticalFlowPyramid(is_left ? img_left : img_right, is_left ? imgpyr_left : imgpyr_right, win_size, pyr_levels);
                  }
                }));
  rT2 = boost::posix_time::microsec_clock::local_time();

  // If we didn't have any successful tracks last time, just extract this time
  // This also handles, the tracking initalization on the first call to this extractor
  if (pts_last[cam_id_left].empty() || pts_last[cam_id_right].empty()) {
    // Track into the new image
    perform_detection_stereo(imgpyr_left, imgpyr_right, pts_last[cam_id_left], pts_last[cam_id_right], ids_last[cam_id_left],
                             ids_last[cam_id_right]);
    // Save the current image and pyramid
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = imgpyr_left;
    img_pyramid_last[cam_id_right] = imgpyr_right;
    return;
  }

  // First we should make that the last images have enough features so we can do KLT
  // This will "top-off" our number of tracks so always have a constant number
  perform_detection_stereo(img_pyramid_last[cam_id_left], img_pyramid_last[cam_id_right], pts_last[cam_id_left], pts_last[cam_id_right],
                           ids_last[cam_id_left], ids_last[cam_id_right]);
  rT3 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // Our return success masks, and predicted new features
  std::vector<uchar> mask_ll, mask_rr;
  std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id_left];
  std::vector<cv::KeyPoint> pts_right_new = pts_last[cam_id_right];

  // Lets track temporally
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    perform_matching(img_pyramid_last[is_left ? cam_id_left : cam_id_right], is_left ? imgpyr_left : imgpyr_right,
                                     pts_last[is_left ? cam_id_left : cam_id_right], is_left ? pts_left_new : pts_right_new,
                                     is_left ? cam_id_left : cam_id_right, is_left ? cam_id_left : cam_id_right,
                                     is_left ? mask_ll : mask_rr);
                  }
                }));
  rT4 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // left to right matching
  // TODO: we should probably still do this to reject outliers
  // TODO: maybe we should collect all tracks that are in both frames and make they pass this?
  // std::vector<uchar> mask_lr;
  // perform_matching(imgpyr_left, imgpyr_right, pts_left_new, pts_right_new, cam_id_left, cam_id_right, mask_lr);
  rT5 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // If any of our masks are empty, that means we didn't have enough to do ransac, so just return
  if (mask_ll.empty() || mask_rr.empty()) {
    img_last[cam_id_left] = std::move(img_left);
    img_last[cam_id_right] = std::move(img_right);
    img_pyramid_last[cam_id_left] = std::move(imgpyr_left);
    img_pyramid_last[cam_id_right] = std::move(imgpyr_right);
    pts_last[cam_id_left].clear();
    pts_last[cam_id_right].clear();
    ids_last[cam_id_left].clear();
    ids_last[cam_id_right].clear();
    printf(RED "[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting....." RESET);
    return;
  }

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left, good_right;
  std::vector<size_t> good_ids_left, good_ids_right;

  // Loop through all left points
  for (size_t i = 0; i < pts_left_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_left_new.at(i).pt.x < 0 || pts_left_new.at(i).pt.y < 0 || (int)pts_left_new.at(i).pt.x > img_left.cols ||
        (int)pts_left_new.at(i).pt.y > img_left.rows)
      continue;
    // See if we have the same feature in the right
    bool found_right = false;
    size_t index_right = 0;
    for (size_t n = 0; n < ids_last[cam_id_right].size(); n++) {
      if (ids_last[cam_id_left].at(i) == ids_last[cam_id_right].at(n)) {
        found_right = true;
        index_right = n;
        break;
      }
    }
    // If it is a good track, and also tracked from left to right
    // Else track it as a mono feature in just the left image
    if (mask_ll[i] && found_right && mask_rr[index_right]) {
      // Ensure we do not have any bad KLT tracks (i.e., points are negative)
      if (pts_right_new.at(index_right).pt.x < 0 || pts_right_new.at(index_right).pt.y < 0 ||
          (int)pts_right_new.at(index_right).pt.x > img_right.cols || (int)pts_right_new.at(index_right).pt.y > img_right.rows)
        continue;
      good_left.push_back(pts_left_new.at(i));
      good_right.push_back(pts_right_new.at(index_right));
      good_ids_left.push_back(ids_last[cam_id_left].at(i));
      good_ids_right.push_back(ids_last[cam_id_right].at(index_right));
      // std::cout << "adding to stereo - " << ids_last[cam_id_left].at(i) << " , " << ids_last[cam_id_right].at(index_right) << std::endl;
    } else if (mask_ll[i]) {
      good_left.push_back(pts_left_new.at(i));
      good_ids_left.push_back(ids_last[cam_id_left].at(i));
      // std::cout << "adding to left - " << ids_last[cam_id_left].at(i) << std::endl;
    }
  }

  // Loop through all right points
  for (size_t i = 0; i < pts_right_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_right_new.at(i).pt.x < 0 || pts_right_new.at(i).pt.y < 0 || (int)pts_right_new.at(i).pt.x > img_right.cols ||
        (int)pts_right_new.at(i).pt.y > img_right.rows)
      continue;
    // See if we have the same feature in the right
    bool added_already = (std::find(good_ids_right.begin(), good_ids_right.end(), ids_last[cam_id_right].at(i)) != good_ids_right.end());
    // If it has not already been added as a good feature, add it as a mono track
    if (mask_rr[i] && !added_already) {
      good_right.push_back(pts_right_new.at(i));
      good_ids_right.push_back(ids_last[cam_id_right].at(i));
      // std::cout << "adding to right - " << ids_last[cam_id_right].at(i) << std::endl;
    }
  }

  //===================================================================================
  //===================================================================================

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id_left);
    database->update_feature(good_ids_left.at(i), timestamp, cam_id_left, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x, npt_l.y);
  }
  for (size_t i = 0; i < good_right.size(); i++) {
    cv::Point2f npt_r = undistort_point(good_right.at(i).pt, cam_id_right);
    database->update_feature(good_ids_right.at(i), timestamp, cam_id_right, good_right.at(i).pt.x, good_right.at(i).pt.y, npt_r.x, npt_r.y);
  }

  // Move forward in time
  img_last[cam_id_left] = std::move(img_left);
  img_last[cam_id_right] = std::move(img_right);
  img_pyramid_last[cam_id_left] = std::move(imgpyr_left);
  img_pyramid_last[cam_id_right] = std::move(imgpyr_right);
  pts_last[cam_id_left] = std::move(good_left);
  pts_last[cam_id_right] = std::move(good_right);
  ids_last[cam_id_left] = std::move(good_ids_left);
  ids_last[cam_id_right] = std::move(good_ids_right);
  rT6 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  // printf("[TIME-KLT]: %.4f seconds for pyramid\n",(rT2-rT1).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for detection\n",(rT3-rT2).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for temporal klt\n",(rT4-rT3).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for stereo klt\n",(rT5-rT4).total_microseconds() * 1e-6);
  // printf("[TIME-KLT]: %.4f seconds for feature DB update (%d features)\n",(rT6-rT5).total_microseconds() * 1e-6, (int)good_left.size());
  // printf("[TIME-KLT]: %.4f seconds for total\n",(rT6-rT1).total_microseconds() * 1e-6);
}

void TrackKLT::perform_detection_monocular(const std::vector<cv::Mat> &img0pyr, std::vector<cv::KeyPoint> &pts0,
                                           std::vector<size_t> &ids0) {

  // First compute how many more features we need to extract from this image
  int num_featsneeded = num_features - (int)pts0.size();

  // If we don't need any features, just return
  if (num_featsneeded < 0.2 * num_features)
    return;

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less then grid_px_size points away then existing features
  // TODO: figure out why I need to add the windowsize of the klt to handle features that are outside the image bound
  // TODO: I assume this is because klt of features at the corners is not really well defined, thus if it doesn't get a match it will be
  // out-of-bounds
  Eigen::MatrixXi grid_2d =
      Eigen::MatrixXi::Zero((int)(img0pyr.at(0).rows / min_px_dist) + 15, (int)(img0pyr.at(0).cols / min_px_dist) + 15);
  auto it0 = pts0.begin();
  auto it2 = ids0.begin();
  while (it0 != pts0.end()) {
    // Get current left keypoint
    cv::KeyPoint kpt = *it0;
    // Check if this keypoint is near another point
    if (grid_2d((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1) {
      it0 = pts0.erase(it0);
      it2 = ids0.erase(it2);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
    it0++;
    it2++;
  }

  // Extract our features (use fast with griding)
  std::vector<cv::KeyPoint> pts0_ext;
  Grider_FAST::perform_griding(img0pyr.at(0), pts0_ext, num_featsneeded, grid_x, grid_y, threshold, true);

  // Now, reject features that are close a current feature
  std::vector<cv::KeyPoint> kpts0_new;
  std::vector<cv::Point2f> pts0_new;
  for (auto &kpt : pts0_ext) {
    // Check that it is in bounds
    bool outof_bounds_0 = (kpt.pt.x < 0 || kpt.pt.y < 0 || kpt.pt.x >= img0pyr.at(0).cols || kpt.pt.y >= img0pyr.at(0).rows);
    if (outof_bounds_0)
      continue;
    // See if there is a point at this location
    if (grid_2d((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1)
      continue;
    // Else lets add it!
    kpts0_new.push_back(kpt);
    pts0_new.push_back(kpt.pt);
    grid_2d((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
  }

  // Loop through and record only ones that are valid
  // NOTE: if we multi-thread this atomic can cause some randomness due to multiple thread detecting features
  // NOTE: this is due to the fact that we select update features based on feat id
  // NOTE: thus the order will matter since we try to select oldest (smallest id) to update with
  // NOTE: not sure how to remove... maybe a better way?
  for (size_t i = 0; i < pts0_new.size(); i++) {
    // update the uv coordinates
    kpts0_new.at(i).pt = pts0_new.at(i);
    // append the new uv coordinate
    pts0.push_back(kpts0_new.at(i));
    // move id foward and append this new point
    size_t temp = ++currid;
    ids0.push_back(temp);
  }
}

void TrackKLT::perform_detection_stereo(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr,
                                        std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1,
                                        std::vector<size_t> &ids0, std::vector<size_t> &ids1) {

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less then grid_px_size points away then existing features
  // TODO: figure out why I need to add the windowsize of the klt to handle features that are outside the image bound
  // TODO: I assume this is because klt of features at the corners is not really well defined, thus if it doesn't get a match it will be
  // out-of-bounds
  Eigen::MatrixXi grid_2d_0 =
      Eigen::MatrixXi::Zero((int)(img0pyr.at(0).rows / min_px_dist) + 15, (int)(img0pyr.at(0).cols / min_px_dist) + 15);
  Eigen::MatrixXi grid_2d_1 =
      Eigen::MatrixXi::Zero((int)(img1pyr.at(0).rows / min_px_dist) + 15, (int)(img1pyr.at(0).cols / min_px_dist) + 15);
  auto it0 = pts0.begin();
  auto it1 = ids0.begin();
  while (it0 != pts0.end()) {
    // Get current left keypoint
    cv::KeyPoint kpt = *it0;
    // Check if this keypoint is near another point
    if (grid_2d_0((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d_0((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
    it0++;
    it1++;
  }
  it0 = pts1.begin();
  it1 = ids1.begin();
  while (it0 != pts1.end()) {
    // Get current right keypoint
    cv::KeyPoint kpt = *it0;
    // Check if this keypoint is near another point
    if (grid_2d_1((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d_1((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
    it0++;
    it1++;
  }

  // First compute how many more features we need to extract from this image
  int num_featsneeded_0 = num_features - (int)pts0.size();

  // LEFT: if we need features we should extract them in the current frame
  // LEFT: we will also try to track them from this frame over to the right frame
  // LEFT: in the case that we have two features that are the same, then we should merge them
  if (num_featsneeded_0 > 0.2 * num_features) {

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_FAST::perform_griding(img0pyr.at(0), pts0_ext, num_featsneeded_0, grid_x, grid_y, threshold, true);

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for (auto &kpt : pts0_ext) {
      // See if there is a point at this location
      if (grid_2d_0((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1)
        continue;
      // Else lets add it!
      kpts0_new.push_back(kpt);
      pts0_new.push_back(kpt.pt);
      grid_2d_0((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
    }

    // TODO: Project points from the left frame into the right frame
    // TODO: This will not work for large baseline systems.....
    std::vector<cv::KeyPoint> kpts1_new;
    std::vector<cv::Point2f> pts1_new;
    kpts1_new = kpts0_new;
    pts1_new = pts0_new;

    // If we have points, do KLT tracking to get the valid projections
    if (!pts0_new.empty()) {

      // Do our KLT tracking from the left to the right frame of reference
      // Note: we have a pretty big window size here since our projection might be bad
      // Note: but this might cause failure in cases of repeated textures (eg. checkerboard)
      std::vector<uchar> mask;
      std::vector<float> error;
      cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.01);
      cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0_new, pts1_new, mask, error, win_size, pyr_levels, term_crit,
                               cv::OPTFLOW_USE_INITIAL_FLOW);

      // Loop through and record only ones that are valid
      for (size_t i = 0; i < pts0_new.size(); i++) {

        // Check that our tracks are in bounds
        bool outof_bounds_0 = (pts0_new.at(i).x < 0 || pts0_new.at(i).y < 0 || pts0_new.at(i).x >= img0pyr.at(0).cols ||
                               pts0_new.at(i).y >= img0pyr.at(0).rows);
        bool outof_bounds_1 = (pts1_new.at(i).x < 0 || pts1_new.at(i).y < 0 || pts1_new.at(i).x >= img1pyr.at(0).cols ||
                               pts1_new.at(i).y >= img1pyr.at(0).rows);

        // If this is not already in the right image, then we should treat it as a stereo
        // Otherwise we will treat this as just a monocular track of the feature
        // TODO: we should check to see if we can combine this new feature and the one in the right
        if (!outof_bounds_0 && mask[i] == 1 && !outof_bounds_1 &&
            !(grid_2d_1((int)(pts1_new.at(i).y / min_px_dist), (int)(pts1_new.at(i).x / min_px_dist)) == 1)) {
          // update the uv coordinates
          kpts0_new.at(i).pt = pts0_new.at(i);
          kpts1_new.at(i).pt = pts1_new.at(i);
          // append the new uv coordinate
          pts0.push_back(kpts0_new.at(i));
          pts1.push_back(kpts1_new.at(i));
          // move id forward and append this new point
          size_t temp = ++currid;
          ids0.push_back(temp);
          ids1.push_back(temp);
        } else if (!outof_bounds_0) {
          // update the uv coordinates
          kpts0_new.at(i).pt = pts0_new.at(i);
          // append the new uv coordinate
          pts0.push_back(kpts0_new.at(i));
          // move id forward and append this new point
          size_t temp = ++currid;
          ids0.push_back(temp);
        }
      }
    }
  }

  // RIGHT: if we need features we should extract them in the current frame
  // RIGHT: note that we don't track them to the left as we already did left->right tracking above
  int num_featsneeded_1 = num_features - (int)pts1.size();
  if (num_featsneeded_1 > 0.2 * num_features) {

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts1_ext;
    Grider_FAST::perform_griding(img1pyr.at(0), pts1_ext, num_featsneeded_1, grid_x, grid_y, threshold, true);

    // Now, reject features that are close a current feature
    for (auto &kpt : pts1_ext) {
      // See if there is a point at this location
      if (grid_2d_1((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) == 1)
        continue;
      // Else lets add it!
      pts1.push_back(kpt);
      size_t temp = ++currid;
      ids1.push_back(temp);
      grid_2d_1((int)(kpt.pt.y / min_px_dist), (int)(kpt.pt.x / min_px_dist)) = 1;
    }
  }
}

void TrackKLT::perform_matching(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr, std::vector<cv::KeyPoint> &kpts0,
                                std::vector<cv::KeyPoint> &kpts1, size_t id0, size_t id1, std::vector<uchar> &mask_out) {

  // We must have equal vectors
  assert(kpts0.size() == kpts1.size());

  // Return if we don't have any points
  if (kpts0.empty() || kpts1.empty())
    return;

  // Convert keypoints into points (stupid opencv stuff)
  std::vector<cv::Point2f> pts0, pts1;
  for (size_t i = 0; i < kpts0.size(); i++) {
    pts0.push_back(kpts0.at(i).pt);
    pts1.push_back(kpts1.at(i).pt);
  }

  // If we don't have enough points for ransac just return empty
  // We set the mask to be all zeros since all points failed RANSAC
  if (pts0.size() < 10) {
    for (size_t i = 0; i < pts0.size(); i++)
      mask_out.push_back((uchar)0);
    return;
  }

  // Now do KLT tracking to get the valid new points
  std::vector<uchar> mask_klt;
  std::vector<float> error;
  cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.01);
  cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

  // Normalize these points, so we can then do ransac
  // We don't want to do ransac on distorted image uvs since the mapping is nonlinear
  std::vector<cv::Point2f> pts0_n, pts1_n;
  for (size_t i = 0; i < pts0.size(); i++) {
    pts0_n.push_back(undistort_point(pts0.at(i), id0));
    pts1_n.push_back(undistort_point(pts1.at(i), id1));
  }

  // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
  std::vector<uchar> mask_rsc;
  double max_focallength_img0 = std::max(camera_k_OPENCV.at(id0)(0, 0), camera_k_OPENCV.at(id0)(1, 1));
  double max_focallength_img1 = std::max(camera_k_OPENCV.at(id1)(0, 0), camera_k_OPENCV.at(id1)(1, 1));
  double max_focallength = std::max(max_focallength_img0, max_focallength_img1);
  cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 1.0 / max_focallength, 0.999, mask_rsc);

  // Loop through and record only ones that are valid
  for (size_t i = 0; i < mask_klt.size(); i++) {
    auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i]) ? 1 : 0);
    mask_out.push_back(mask);
  }

  // Copy back the updated positions
  for (size_t i = 0; i < pts0.size(); i++) {
    kpts0.at(i).pt = pts0.at(i);
    kpts1.at(i).pt = pts1.at(i);
  }
}
