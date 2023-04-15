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

#ifndef OV_CORE_CAM_RADTAN_H
#define OV_CORE_CAM_RADTAN_H

#include "CamBase.h"

namespace ov_core {

/**
 * @brief Radial-tangential / Brown–Conrady model pinhole camera model class
 *
 * To calibrate camera intrinsics, we need to know how to map our normalized coordinates
 * into the raw pixel coordinates on the image plane. We first employ the radial distortion
 * as in [OpenCV model](https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#details):
 *
 * \f{align*}{
 * \begin{bmatrix} u \\ v \end{bmatrix}:= \mathbf{z}_k &= \mathbf h_d(\mathbf{z}_{n,k}, ~\boldsymbol\zeta)
 * = \begin{bmatrix}  f_x * x + c_x \\
 * f_y * y + c_y \end{bmatrix}\\[1em]
 * \empty
 * {\rm where}~~
 * x &= x_n (1 + k_1 r^2 + k_2 r^4) + 2 p_1 x_n y_n + p_2(r^2 + 2 x_n^2) \\\
 * y &= y_n (1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n \\[1em]
 * r^2 &= x_n^2 + y_n^2
 * \f}
 *
 * where \f$ \mathbf{z}_{n,k} = [ x_n ~ y_n ]^\top\f$  are the normalized coordinates of the 3D feature and u and v are the distorted image
 * coordinates on the image plane. The following distortion and camera intrinsic (focal length and image center) parameters are involved in
 * the above distortion model, which can be estimated online:
 *
 * \f{align*}{
 * \boldsymbol\zeta = \begin{bmatrix} f_x & f_y & c_x & c_y & k_1 & k_2 & p_1 & p_2 \end{bmatrix}^\top
 * \f}
 *
 * Note that we do not estimate the higher order (i.e., higher than fourth order) terms as in most offline calibration
 * methods such as [Kalibr](https://github.com/ethz-asl/kalibr). To estimate these intrinsic parameters (including the
 * distortation parameters), the following Jacobian for these parameters is needed:
 *
 * \f{align*}{
 * \frac{\partial \mathbf h_d(\cdot)}{\partial \boldsymbol\zeta} =
 * \begin{bmatrix}
 * x & 0  & 1 & 0 & f_x*(x_nr^2) & f_x*(x_nr^4) & f_x*(2x_ny_n) & f_x*(r^2+2x_n^2)  \\[5pt]
 * 0  & y & 0 & 1 & f_y*(y_nr^2) & f_y*(y_nr^4) & f_y*(r^2+2y_n^2) & f_y*(2x_ny_n)
 * \end{bmatrix}
 * \f}
 *
 * Similarly, the Jacobian with respect to the normalized coordinates can be obtained as follows:
 *
 * \f{align*}{
 * \frac{\partial \mathbf h_d (\cdot)}{\partial \mathbf{z}_{n,k}} =
 * \begin{bmatrix}
 * f_x*((1+k_1r^2+k_2r^4)+(2k_1x_n^2+4k_2x_n^2(x_n^2+y_n^2))+2p_1y_n+(2p_2x_n+4p_2x_n))  &
 * f_x*(2k_1x_ny_n+4k_2x_ny_n(x_n^2+y_n^2)+2p_1x_n+2p_2y_n)    \\
 * f_y*(2k_1x_ny_n+4k_2x_ny_n(x_n^2+y_n^2)+2p_1x_n+2p_2y_n)  &
 * f_y*((1+k_1r^2+k_2r^4)+(2k_1y_n^2+4k_2y_n^2(x_n^2+y_n^2))+(2p_1y_n+4p_1y_n)+2p_2x_n)
 * \end{bmatrix}
 * \f}
 *
 * To equate this camera class to Kalibr's models, this is what you would use for `pinhole-radtan`.
 *
 */
class CamRadtan : public CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamRadtan(int width, int height) : CamBase(width, height) {}

  ~CamRadtan() {}

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {

    // Determine what camera parameters we should use
    cv::Matx33d camK = camera_k_OPENCV;
    cv::Vec4d camD = camera_d_OPENCV;

    // Convert to opencv format
    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv_dist(0);
    mat.at<float>(0, 1) = uv_dist(1);
    mat = mat.reshape(2); // Nx1, 2-channel

    // Undistort it!
    cv::undistortPoints(mat, mat, camK, camD);

    // Construct our return vector
    Eigen::Vector2f pt_out;
    mat = mat.reshape(1); // Nx2, 1-channel
    pt_out(0) = mat.at<float>(0, 0);
    pt_out(1) = mat.at<float>(0, 1);
    return pt_out;
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {

    // Get our camera parameters
    Eigen::MatrixXd cam_d = camera_values;

    // Calculate distorted coordinates for radial
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double r_2 = r * r;
    double r_4 = r_2 * r_2;
    double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
                cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
    double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
                2 * cam_d(7) * uv_norm(0) * uv_norm(1);

    // Return the distorted point
    Eigen::Vector2f uv_dist;
    uv_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
    uv_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));
    return uv_dist;
  }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic parameters
   */
  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override {

    // Get our camera parameters
    Eigen::MatrixXd cam_d = camera_values;

    // Calculate distorted coordinates for radial
    double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    double r_2 = r * r;
    double r_4 = r_2 * r_2;

    // Jacobian of distorted pixel to normalized pixel
    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    double x = uv_norm(0);
    double y = uv_norm(1);
    double x_2 = uv_norm(0) * uv_norm(0);
    double y_2 = uv_norm(1) * uv_norm(1);
    double x_y = uv_norm(0) * uv_norm(1);
    H_dz_dzn(0, 0) = cam_d(0) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * x_2 + 4 * cam_d(5) * x_2 * r_2) +
                                 2 * cam_d(6) * y + (2 * cam_d(7) * x + 4 * cam_d(7) * x));
    H_dz_dzn(0, 1) = cam_d(0) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
    H_dz_dzn(1, 0) = cam_d(1) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
    H_dz_dzn(1, 1) = cam_d(1) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * y_2 + 4 * cam_d(5) * y_2 * r_2) +
                                 2 * cam_d(7) * x + (2 * cam_d(6) * y + 4 * cam_d(6) * y));

    // Calculate distorted coordinates for radtan
    double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
                cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
    double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
                2 * cam_d(7) * uv_norm(0) * uv_norm(1);

    // Compute the Jacobian in respect to the intrinsics
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
    H_dz_dzeta(0, 0) = x1;
    H_dz_dzeta(0, 2) = 1;
    H_dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * r_2;
    H_dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * r_4;
    H_dz_dzeta(0, 6) = 2 * cam_d(0) * uv_norm(0) * uv_norm(1);
    H_dz_dzeta(0, 7) = cam_d(0) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
    H_dz_dzeta(1, 1) = y1;
    H_dz_dzeta(1, 3) = 1;
    H_dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * r_2;
    H_dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * r_4;
    H_dz_dzeta(1, 6) = cam_d(1) * (r_2 + 2 * uv_norm(1) * uv_norm(1));
    H_dz_dzeta(1, 7) = 2 * cam_d(1) * uv_norm(0) * uv_norm(1);
  }
};

} // namespace ov_core

#endif /* OV_CORE_CAM_RADTAN_H */