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

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "core/VioManagerOptions.h"

using namespace ov_msckf;

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: test_mask_downsampling <config-directory>" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string config_dir = argv[1];
  const std::string test_config_path = config_dir + "/test_mask_downsampling.yaml";
  const std::string test_mask_path = config_dir + "/test_mask_downsampling.png";
  const cv::Mat mask(480, 752, CV_8UC1, cv::Scalar(255));

  if (!cv::imwrite(test_mask_path, mask)) {
    std::cerr << "Unable to create test mask." << std::endl;
    return EXIT_FAILURE;
  }

  {
    std::ofstream config(test_config_path);
    config << "%YAML:1.0\nmax_cameras: 2\ngravity_mag: 9.81\ndownsample_cameras: true\nuse_mask: true\n"
           << "mask0: \"test_mask_downsampling.png\"\nmask1: \"test_mask_downsampling.png\"\n"
           << "relative_config_imu: \"kalibr_imu_chain.yaml\"\nrelative_config_imucam: \"kalibr_imucam_chain.yaml\"\n";
  }

  auto parser = std::make_shared<ov_core::YamlParser>(test_config_path);
  VioManagerOptions options;
  options.state_options.num_cameras = 2;
  options.downsample_cameras = true;
  options.print_and_load_state(parser);

  std::remove(test_config_path.c_str());
  std::remove(test_mask_path.c_str());

  if (options.masks.at(0).size() != mask.size() || options.camera_intrinsics.at(0)->w() != 376 ||
      options.camera_intrinsics.at(0)->h() != 240) {
    std::cerr << "Downsampled camera did not retain a mask in the input image coordinate system." << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
