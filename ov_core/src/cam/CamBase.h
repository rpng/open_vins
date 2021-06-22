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


#ifndef OV_CORE_CAM_BASE_H
#define OV_CORE_CAM_BASE_H

#include <Eigen/Eigen>
#include <unordered_map>

#include <opencv2/opencv.hpp>

namespace ov_core {

/**
 * @brief Base pinhole camera model class
 *
 * This is the base class for all our camera models.
 * All these models are pinhole cameras, thus just have standard reprojection logic.
 *
 * See each base class for detailed examples on each model:
 *  - @ref ov_core::CamEqui
 *  - @ref ov_core::CamRadtan
 */
class CamBase {

public:

  /**
   * @brief This will set and update the camera calibration values.
   * This should be called on startup for each camera and after update!
   * @param cam_id Which camera id we use intrinsics from
   * @param calib Camera calibration information (f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 & k_4)
   */
  virtual void set_values(int cam_id, const Eigen::MatrixXd &calib) {

    // Assert we are of size eight
    assert(calib.rows() == 8);
    camera_values[cam_id] = calib;

    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = calib(0);
    tempK(0, 1) = 0;
    tempK(0, 2) = calib(2);
    tempK(1, 0) = 0;
    tempK(1, 1) = calib(1);
    tempK(1, 2) = calib(3);
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV[cam_id] = tempK;

    // Distortion parameters
    cv::Vec4d tempD;
    tempD(0) = calib(4);
    tempD(1) = calib(5);
    tempD(2) = calib(6);
    tempD(3) = calib(7);
    camera_d_OPENCV[cam_id] = tempD;
  }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param cam_id Which camera id we use intrinsics from
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  virtual Eigen::Vector2f undistort(int cam_id, Eigen::Vector2f uv_dist) = 0;

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param cam_id Which camera id we use intrinsics from
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  cv::Point2f undistort(int cam_id, cv::Point2f uv_dist) {
    Eigen::Vector2f ept1, ept2;
    ept1 << uv_dist.x, uv_dist.y;
    ept2 = undistort(cam_id, ept1);
    cv::Point2f pt_out;
    pt_out.x = ept2(0);
    pt_out.y = ept2(1);
    return pt_out;
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param cam_id Which camera id we use intrinsics from
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  virtual Eigen::Vector2f distort(int cam_id, Eigen::Vector2f uv_norm) = 0;

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param cam_id Which camera id we use intrinsics from
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  cv::Point2f distort(int cam_id, cv::Point2f uv_norm) {
    Eigen::Vector2f ept1, ept2;
    ept1 << uv_norm.x, uv_norm.y;
    ept2 = distort(cam_id, ept1);
    cv::Point2f pt_out;
    pt_out.x = ept2(0);
    pt_out.y = ept2(1);
    return pt_out;
  }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param cam_id Which camera id we use intrinsics from
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic parameters
   */
  virtual void compute_distort_jacobian(int cam_id, Eigen::Vector2f uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) = 0;

  /**
   * @brief Gets how many unique cameras we have
   * @return num cameras
   */
  size_t get_num_cameras() {
    return camera_k_OPENCV.size();
  }

protected:

  // Cannot construct the base camera class, needs a distortion model
  CamBase() = default;
  
  /// Raw set of camera intrinic values (f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 & k_4)
  std::map<size_t, Eigen::MatrixXd> camera_values;

  /// Camera intrinsics in OpenCV format
  std::map<size_t, cv::Matx33d> camera_k_OPENCV;

  /// Camera distortion in OpenCV format
  std::map<size_t, cv::Vec4d> camera_d_OPENCV;

};

} // namespace ov_core

#endif /* OV_CORE_CAM_BASE_H */