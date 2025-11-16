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

#ifndef OV_CORE_TRACK_DESC_H
#define OV_CORE_TRACK_DESC_H

#include "TrackBase.h"

namespace ov_core {

/**
 * @brief Descriptor-based visual tracking
 *
 * Here we use descriptor matching to track features from one frame to the next.
 * We track both temporally, and across stereo pairs to get stereo constraints.
 * Right now we use ORB descriptors as we have found it is the fastest when computing descriptors.
 * Tracks are then rejected based on a ratio test and ransac.
 */
class TrackDescriptor : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   * @param fast_threshold FAST detection threshold
   * @param gridx size of grid in the x-direction / u-direction
   * @param gridy size of grid in the y-direction / v-direction
   * @param minpxdist features need to be at least this number pixels away from each other
   * @param knnratio matching ratio needed (smaller value forces top two descriptors during match to be more different)
   */
  explicit TrackDescriptor(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numfeats, int numaruco, bool stereo,
                           HistogramMethod histmethod, int fast_threshold, int gridx, int gridy, int minpxdist, double knnratio)
      : TrackBase(cameras, numfeats, numaruco, stereo, histmethod), threshold(fast_threshold), grid_x(gridx), grid_y(gridy),
        min_px_dist(minpxdist), knn_ratio(knnratio) {}

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override;

protected:
  /**
   * @brief Process a new monocular image
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id the camera index in message data vector
   */
  void feed_monocular(const CameraData &message, size_t msg_id);

  /**
   * @brief Process new stereo pair of images
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id_left first image index in message data vector
   * @param msg_id_right second image index in message data vector
   */
  void feed_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right);

  /**
   * @brief Detects new features in the current image
   * @param img0 image we will detect features on
   * @param mask0 mask which has what ROI we do not want features in
   * @param pts0 vector of extracted keypoints
   * @param desc0 vector of the extracted descriptors
   * @param ids0 vector of all new IDs
   *
   * Given a set of images, and their currently extracted features, this will try to add new features.
   * We return all extracted descriptors here since we DO NOT need to do stereo tracking left to right.
   * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
   * See robust_match() for the matching.
   */
  void perform_detection_monocular(const cv::Mat &img0, const cv::Mat &mask0, std::vector<cv::KeyPoint> &pts0, cv::Mat &desc0,
                                   std::vector<size_t> &ids0);

  /**
   * @brief Detects new features in the current stereo pair
   * @param img0 left image we will detect features on
   * @param img1 right image we will detect features on
   * @param mask0 mask which has what ROI we do not want features in
   * @param mask1 mask which has what ROI we do not want features in
   * @param pts0 left vector of new keypoints
   * @param pts1 right vector of new keypoints
   * @param desc0 left vector of extracted descriptors
   * @param desc1 left vector of extracted descriptors
   * @param cam_id0 id of the first camera
   * @param cam_id1 id of the second camera
   * @param ids0 left vector of all new IDs
   * @param ids1 right vector of all new IDs
   *
   * This does the same logic as the perform_detection_monocular() function, but we also enforce stereo contraints.
   * We also do STEREO matching from the left to right, and only return good matches that are found in both the left and right.
   * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
   * See robust_match() for the matching.
   */
  void perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, const cv::Mat &mask0, const cv::Mat &mask1,
                                std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1, cv::Mat &desc0, cv::Mat &desc1,
                                size_t cam_id0, size_t cam_id1, std::vector<size_t> &ids0, std::vector<size_t> &ids1);

  /**
   * @brief Find matches between two keypoint+descriptor sets.
   * @param pts0 first vector of keypoints
   * @param pts1 second vector of keypoints
   * @param desc0 first vector of descriptors
   * @param desc1 second vector of decriptors
   * @param id0 id of the first camera
   * @param id1 id of the second camera
   * @param matches vector of matches that we have found
   *
   * This will perform a "robust match" between the two sets of points (slow but has great results).
   * First we do a simple KNN match from 1to2 and 2to1, which is followed by a ratio check and symmetry check.
   * Original code is from the "RobustMatcher" in the opencv examples, and seems to give very good results in the matches.
   * https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
   */
  void robust_match(const std::vector<cv::KeyPoint> &pts0, const std::vector<cv::KeyPoint> &pts1, const cv::Mat &desc0,
                    const cv::Mat &desc1, size_t id0, size_t id1, std::vector<cv::DMatch> &matches);

  // Helper functions for the robust_match function
  // Original code is from the "RobustMatcher" in the opencv examples
  // https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
  void robust_ratio_test(std::vector<std::vector<cv::DMatch>> &matches);
  void robust_symmetry_test(std::vector<std::vector<cv::DMatch>> &matches1, std::vector<std::vector<cv::DMatch>> &matches2,
                            std::vector<cv::DMatch> &good_matches);

  // Timing variables
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // Our orb extractor
  cv::Ptr<cv::ORB> orb0 = cv::ORB::create();
  cv::Ptr<cv::ORB> orb1 = cv::ORB::create();

  // Our descriptor matcher
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Parameters for our FAST grid detector
  int threshold;
  int grid_x;
  int grid_y;

  // Minimum pixel distance to be "far away enough" to be a different extracted feature
  int min_px_dist;

  // The ratio between two kNN matches, if that ratio is larger then this threshold
  // then the two features are too close, so should be considered ambiguous/bad match
  double knn_ratio;

  // Descriptor matrices
  std::unordered_map<size_t, cv::Mat> desc_last;
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_DESC_H */
