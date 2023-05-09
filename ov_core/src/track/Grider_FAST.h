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

#ifndef OV_CORE_GRIDER_FAST_H
#define OV_CORE_GRIDER_FAST_H

#include <Eigen/Eigen>
#include <functional>
#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "utils/opencv_lambda_body.h"

namespace ov_core {

/**
 * @brief Extracts FAST features in a grid pattern.
 *
 * As compared to just extracting fast features over the entire image,
 * we want to have as uniform of extractions as possible over the image plane.
 * Thus we split the image into a bunch of small grids, and extract points in each.
 * We then pick enough top points in each grid so that we have the total number of desired points.
 */
class Grider_FAST {

public:
  /**
   * @brief Compare keypoints based on their response value.
   * @param first First keypoint
   * @param second Second keypoint
   *
   * We want to have the keypoints with the highest values!
   * See: https://stackoverflow.com/a/10910921
   */
  static bool compare_response(cv::KeyPoint first, cv::KeyPoint second) { return first.response > second.response; }

  /**
   * @brief This function will perform grid extraction using FAST.
   * @param img Image we will do FAST extraction on
   * @param mask Region of the image we do not want to extract features in (255 = do not detect features)
   * @param pts vector of extracted points we will return
   * @param num_features max number of features we want to extract
   * @param grid_x size of grid in the x-direction / u-direction
   * @param grid_y size of grid in the y-direction / v-direction
   * @param threshold FAST threshold paramter (10 is a good value normally)
   * @param nonmaxSuppression if FAST should perform non-max suppression (true normally)
   *
   * Given a specified grid size, this will try to extract fast features from each grid.
   * It will then return the best from each grid in the return vector.
   */
  static void perform_griding(const cv::Mat &img, const cv::Mat &mask, std::vector<cv::KeyPoint> &pts, int num_features, int grid_x,
                              int grid_y, int threshold, bool nonmaxSuppression) {

    // We want to have equally distributed features
    // NOTE: If we have more grids than number of total points, we calc the biggest grid we can do
    // NOTE: Thus if we extract 1 point per grid we have
    // NOTE:    -> 1 = num_features / (grid_x * grid_y))
    // NOTE:    -> grid_x = ratio * grid_y (keep the original grid ratio)
    // NOTE:    -> grid_y = sqrt(num_features / ratio)
    if (num_features < grid_x * grid_y) {
      double ratio = (double)grid_x / (double)grid_y;
      grid_y = std::ceil(std::sqrt(num_features / ratio));
      grid_x = std::ceil(grid_y * ratio);
    }
    int num_features_grid = (int)((double)num_features / (double)(grid_x * grid_y)) + 1;
    assert(grid_x > 0);
    assert(grid_y > 0);
    assert(num_features_grid > 0);

    // Calculate the size our extraction boxes should be
    int size_x = img.cols / grid_x;
    int size_y = img.rows / grid_y;

    // Make sure our sizes are not zero
    assert(size_x > 0);
    assert(size_y > 0);

    // Parallelize our 2d grid extraction!!
    int ct_cols = std::floor(img.cols / size_x);
    int ct_rows = std::floor(img.rows / size_y);
    std::vector<std::vector<cv::KeyPoint>> collection(ct_cols * ct_rows);
    parallel_for_(cv::Range(0, ct_cols * ct_rows), LambdaBody([&](const cv::Range &range) {
                    for (int r = range.start; r < range.end; r++) {
                      // Calculate what cell xy value we are in
                      int x = r % ct_cols * size_x;
                      int y = r / ct_cols * size_y;

                      // Skip if we are out of bounds
                      if (x + size_x > img.cols || y + size_y > img.rows)
                        continue;

                      // Calculate where we should be extracting from
                      cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                      // Extract FAST features for this part of the image
                      std::vector<cv::KeyPoint> pts_new;
                      cv::FAST(img(img_roi), pts_new, threshold, nonmaxSuppression);

                      // Now lets get the top number from this
                      std::sort(pts_new.begin(), pts_new.end(), Grider_FAST::compare_response);

                      // Append the "best" ones to our vector
                      // Note that we need to "correct" the point u,v since we extracted it in a ROI
                      // So we should append the location of that ROI in the image
                      for (size_t i = 0; i < (size_t)num_features_grid && i < pts_new.size(); i++) {

                        // Create keypoint
                        cv::KeyPoint pt_cor = pts_new.at(i);
                        pt_cor.pt.x += (float)x;
                        pt_cor.pt.y += (float)y;

                        // Reject if out of bounds (shouldn't be possible...)
                        if ((int)pt_cor.pt.x < 0 || (int)pt_cor.pt.x > img.cols || (int)pt_cor.pt.y < 0 || (int)pt_cor.pt.y > img.rows)
                          continue;

                        // Check if it is in the mask region
                        // NOTE: mask has max value of 255 (white) if it should be removed
                        if (mask.at<uint8_t>((int)pt_cor.pt.y, (int)pt_cor.pt.x) > 127)
                          continue;
                        collection.at(r).push_back(pt_cor);
                      }
                    }
                  }));

    // Combine all the collections into our single vector
    for (size_t r = 0; r < collection.size(); r++) {
      pts.insert(pts.end(), collection.at(r).begin(), collection.at(r).end());
    }

    // Return if no points
    if (pts.empty())
      return;

    // Sub-pixel refinement parameters
    cv::Size win_size = cv::Size(5, 5);
    cv::Size zero_zone = cv::Size(-1, -1);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.001);

    // Get vector of points
    std::vector<cv::Point2f> pts_refined;
    for (size_t i = 0; i < pts.size(); i++) {
      pts_refined.push_back(pts.at(i).pt);
    }

    // Finally get sub-pixel for all extracted features
    cv::cornerSubPix(img, pts_refined, win_size, zero_zone, term_crit);

    // Save the refined points!
    for (size_t i = 0; i < pts.size(); i++) {
      pts.at(i).pt = pts_refined.at(i);
    }
  }
};

} // namespace ov_core

#endif /* OV_CORE_GRIDER_FAST_H */
