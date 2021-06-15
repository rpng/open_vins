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


#ifndef OV_CORE_TRACK_ARUCO_H
#define OV_CORE_TRACK_ARUCO_H

#include <opencv2/aruco.hpp>

#include "TrackBase.h"

namespace ov_core {

/**
 * @brief Tracking of OpenCV Aruoc tags.
 *
 * This class handles the tracking of [OpenCV Aruco tags](https://github.com/opencv/opencv_contrib/tree/master/modules/aruco).
 * We track the top left corner of the tag as compared to the pose of the tag or any other corners.
 * Right now we hardcode the dictionary to be `cv::aruco::DICT_6X6_25`, so please generate tags in this family of tags.
 */
class TrackAruco : public TrackBase {

public:
  /**
   * @brief Public default constructor
   */
  TrackAruco() : TrackBase(), max_tag_id(1024), do_downsizing(false) {
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    aruco_params = cv::aruco::DetectorParameters::create();
    // aruco_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX; // people with newer opencv might fail
    // here
  }

  /**
   * @brief Public constructor with configuration variables
   * @param numaruco the max id of the arucotags, we don't use any tags greater than this value even if we extract them
   * @param do_downsizing we can scale the image by 1/2 to increase Aruco tag extraction speed
   */
  explicit TrackAruco(int numaruco, bool do_downsizing) : TrackBase(0, numaruco), max_tag_id(numaruco), do_downsizing(do_downsizing) {
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    aruco_params = cv::aruco::DetectorParameters::create();
    // aruco_params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX; // people with newer opencv might fail
    // here
  }

  /**
   * @brief Process a new monocular image
   * @param timestamp timestamp the new image occurred at
   * @param img new cv:Mat grayscale image
   * @param cam_id the camera id that this new image corresponds too
   */
  void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) override;

  /**
   * @brief Process new stereo pair of images
   * @param timestamp timestamp this pair occured at (stereo is synchronised)
   * @param img_left first grayscaled image
   * @param img_right second grayscaled image
   * @param cam_id_left first image camera id
   * @param cam_id_right second image camera id
   */
  void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) override;

  /**
   * @brief We override the display equation so we can show the tags we extract.
   * @param img_out image to which we will overlayed features on
   * @param r1,g1,b1 first color to draw in
   * @param r2,g2,b2 second color to draw in
   */
  void display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) override;

protected:
  // Timing variables
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // Max tag ID we should extract from (i.e., number of aruco tags starting from zero)
  int max_tag_id;

  // If we should downsize the image
  bool do_downsizing;

  // Our dictionary that we will extract aruco tags with
  cv::Ptr<cv::aruco::Dictionary> aruco_dict;

  // Parameters the opencv extractor uses
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

  // Mutex for our ids_aruco, corners, rejects which we use for drawing
  std::mutex mtx_aruco;

  // Our tag IDs and corner we will get from the extractor
  std::unordered_map<size_t, std::vector<int>> ids_aruco;
  std::unordered_map<size_t, std::vector<std::vector<cv::Point2f>>> corners, rejects;
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_ARUCO_H */
