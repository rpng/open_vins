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

#ifndef OV_CORE_TRACK_ARUCO_H
#define OV_CORE_TRACK_ARUCO_H

#if ENABLE_ARUCO_TAGS
#include <opencv2/aruco.hpp>
#endif

#include "TrackBase.h"

namespace ov_core {

/**
 * @brief Tracking of OpenCV Aruoc tags.
 *
 * This class handles the tracking of [OpenCV Aruco tags](https://github.com/opencv/opencv_contrib/tree/master/modules/aruco).
 * We track the corners of the tag as compared to the pose of the tag or any other corners.
 * Right now we hardcode the dictionary to be `cv::aruco::DICT_6X6_1000`, so please generate tags in this family of tags.
 * You can generate these tags using an online utility: https://chev.me/arucogen/
 * The actual size of the tags do not matter since we do not recover the pose and instead just use this for re-detection and tracking of the
 * four corners of the tag.
 */
class TrackAruco : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numaruco the max id of the arucotags, we don't use any tags greater than this value even if we extract them
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   * @param downsize we can scale the image by 1/2 to increase Aruco tag extraction speed
   */
  explicit TrackAruco(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numaruco, bool stereo, HistogramMethod histmethod,
                      bool downsize)
      : TrackBase(cameras, 0, numaruco, stereo, histmethod), max_tag_id(numaruco), do_downsizing(downsize) {
#if ENABLE_ARUCO_TAGS
#if CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    aruco_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    aruco_detector = cv::aruco::ArucoDetector(aruco_dict, aruco_params);
#else
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    aruco_params = cv::aruco::DetectorParameters::create();
    // NOTE: people with newer opencv might fail here
    // aruco_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
#endif
#else
    PRINT_ERROR(RED "[ERROR]: you have not compiled with aruco tag support!!!\n" RESET);
    std::exit(EXIT_FAILURE);
#endif
  }

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override;

#if ENABLE_ARUCO_TAGS
  /**
   * @brief We override the display equation so we can show the tags we extract.
   * @param img_out image to which we will overlayed features on
   * @param r1,g1,b1 first color to draw in
   * @param r2,g2,b2 second color to draw in
   * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
   */
  void display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay = "") override;
#endif

protected:
#if ENABLE_ARUCO_TAGS
  /**
   * @brief Process a new monocular image
   * @param timestamp timestamp the new image occurred at
   * @param imgin new cv:Mat grayscale image
   * @param cam_id the camera id that this new image corresponds too
   * @param maskin tracking mask for the given input image
   */
  void perform_tracking(double timestamp, const cv::Mat &imgin, size_t cam_id, const cv::Mat &maskin);
#endif

  // Max tag ID we should extract from (i.e., number of aruco tags starting from zero)
  int max_tag_id;

  // If we should downsize the image
  bool do_downsizing;

#if ENABLE_ARUCO_TAGS
#if CV_MAJOR_VERSION > 4 || ( CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 7)
  // Our dictionary that we will extract aruco tags with
  cv::aruco::Dictionary aruco_dict;
  // Parameters the opencv extractor uses
  cv::aruco::DetectorParameters aruco_params;
  // Actual detector class
  cv::aruco::ArucoDetector aruco_detector;
#else
  // Our dictionary that we will extract aruco tags with
  cv::Ptr<cv::aruco::Dictionary> aruco_dict;
  // Parameters the opencv extractor uses
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
#endif


  // Our tag IDs and corner we will get from the extractor
  std::unordered_map<size_t, std::vector<int>> ids_aruco;
  std::unordered_map<size_t, std::vector<std::vector<cv::Point2f>>> corners, rejects;
#endif
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_ARUCO_H */
