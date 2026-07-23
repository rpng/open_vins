/* ROS-free EuRoC/ASL runner for conformal Stage 1. */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DiagnosticsLogger.hpp"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace {

struct ImageRecord {
  double timestamp = 0.0;
  std::string path;
};

std::vector<std::string> split_csv(const std::string &line) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
    fields.emplace_back(field);
  return fields;
}

std::vector<ov_core::ImuData> load_imu_csv(const std::string &path) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("unable to open IMU CSV: " + path);
  std::vector<ov_core::ImuData> measurements;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    const auto fields = split_csv(line);
    if (fields.size() < 7)
      throw std::runtime_error("invalid IMU row in " + path);
    ov_core::ImuData measurement;
    measurement.timestamp = std::stod(fields[0]) * 1e-9;
    measurement.wm << std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3]);
    measurement.am << std::stod(fields[4]), std::stod(fields[5]), std::stod(fields[6]);
    measurements.emplace_back(measurement);
  }
  if (measurements.empty())
    throw std::runtime_error("IMU CSV contains no measurements: " + path);
  if (!std::is_sorted(measurements.begin(), measurements.end()))
    throw std::runtime_error("IMU CSV timestamps are not ordered: " + path);
  return measurements;
}

std::vector<ImageRecord> load_camera_csv(const std::string &path, const std::string &image_directory) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("unable to open camera CSV: " + path);
  std::vector<ImageRecord> images;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    const auto fields = split_csv(line);
    if (fields.size() < 2)
      throw std::runtime_error("invalid camera row in " + path);
    ImageRecord image;
    image.timestamp = std::stod(fields[0]) * 1e-9;
    image.path = image_directory + "/" + fields[1];
    while (!image.path.empty() && (image.path.back() == '\r' || image.path.back() == '\n'))
      image.path.pop_back();
    images.emplace_back(std::move(image));
  }
  if (images.empty())
    throw std::runtime_error("camera CSV contains no images: " + path);
  return images;
}

void validate_gt_sensor_transform(const std::string &sequence_directory) {
  const std::string yaml = sequence_directory + "/mav0/state_groundtruth_estimate0/sensor.yaml";
  std::ifstream stream(yaml);
  if (!stream)
    throw std::runtime_error("unable to open GT sensor YAML: " + yaml);
  const std::string contents((std::istreambuf_iterator<char>(stream)),
                             std::istreambuf_iterator<char>());
  const size_t transform_key = contents.find("T_BS:");
  const size_t data_key = transform_key == std::string::npos
                              ? std::string::npos
                              : contents.find("data:", transform_key);
  const size_t begin = data_key == std::string::npos ? std::string::npos
                                                      : contents.find('[', data_key);
  const size_t end = begin == std::string::npos ? std::string::npos
                                                 : contents.find(']', begin);
  if (begin == std::string::npos || end == std::string::npos)
    throw std::runtime_error("missing T_BS.data in " + yaml);
  std::string values_text = contents.substr(begin + 1, end - begin - 1);
  std::replace(values_text.begin(), values_text.end(), ',', ' ');
  std::istringstream values_stream(values_text);
  std::vector<double> values;
  double value = 0.0;
  while (values_stream >> value)
    values.emplace_back(value);
  if (values.size() != 16)
    throw std::runtime_error("T_BS.data must contain 16 values in " + yaml);
  double error = 0.0;
  for (size_t row = 0; row < 4; ++row) {
    for (size_t column = 0; column < 4; ++column) {
      const double expected = row == column ? 1.0 : 0.0;
      error = std::max(error, std::abs(values[row * 4 + column] - expected));
    }
  }
  if (error > 1e-9) {
    throw std::runtime_error(
        "non-identity ground-truth T_BS is not supported by the Stage-1 runner; Gate 1 must transform GT first");
  }
}

void handle_signal(int signal) { std::exit(signal); }

} // namespace

int main(int argc, char **argv) {
  if (argc < 5 || argc > 6) {
    std::cerr << "usage: run_asl_msckf <estimator_config.yaml> <seq_dir> <sequence_name> <out.h5> [start_offset_s]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string config_path = argv[1];
    const std::string sequence_directory = argv[2];
    const std::string sequence_name = argv[3];
    const std::string output_h5 = argv[4];
    const double start_offset = argc == 6 ? std::stod(argv[5]) : 0.0;
    if (start_offset < 0.0)
      throw std::runtime_error("start_offset_s must be non-negative");
    std::signal(SIGINT, handle_signal);

    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    ov_msckf::VioManagerOptions parameters;
    parameters.print_and_load(parser);
    parameters.num_opencv_threads = 1;
    parameters.use_multi_threading_pubs = false;
    parameters.use_multi_threading_subs = false;
    if (!parser->successful())
      throw std::runtime_error("unable to parse all OpenVINS parameters");

    validate_gt_sensor_transform(sequence_directory);
    auto imu = load_imu_csv(sequence_directory + "/mav0/imu0/data.csv");
    auto camera0 = load_camera_csv(sequence_directory + "/mav0/cam0/data.csv",
                                   sequence_directory + "/mav0/cam0/data");
    auto camera1 = load_camera_csv(sequence_directory + "/mav0/cam1/data.csv",
                                   sequence_directory + "/mav0/cam1/data");
    constexpr double stereo_tolerance = 1e-6;
    std::vector<std::pair<size_t, size_t>> stereo_pairs;
    size_t left_index = 0;
    size_t right_index = 0;
    size_t unmatched_left = 0;
    size_t unmatched_right = 0;
    while (left_index < camera0.size() && right_index < camera1.size()) {
      const double difference = camera0[left_index].timestamp - camera1[right_index].timestamp;
      if (std::abs(difference) <= stereo_tolerance) {
        stereo_pairs.emplace_back(left_index++, right_index++);
      } else if (difference < 0.0) {
        ++unmatched_left;
        ++left_index;
      } else {
        ++unmatched_right;
        ++right_index;
      }
    }
    unmatched_left += camera0.size() - left_index;
    unmatched_right += camera1.size() - right_index;
    if (stereo_pairs.empty())
      throw std::runtime_error("camera CSV files contain no synchronized stereo timestamps");

    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
    ov_core::DatasetReader::load_gt_file(
        sequence_directory + "/mav0/state_groundtruth_estimate0/data.csv", gt_states);

    auto system = std::make_shared<ov_msckf::VioManager>(parameters);
    conformal::DiagnosticsLogger logger(output_h5, sequence_name, config_path);
    system->set_msckf_diagnostic_callback(
        [&logger, &system, &gt_states](const ov_msckf::MsckfFeatureDiagnostic &diagnostic) {
          logger.log_msckf_diagnostic(system, diagnostic, gt_states);
        });

    const double dataset_start = std::min(imu.front().timestamp, camera0.front().timestamp);
    const double first_timestamp = dataset_start + start_offset;
    size_t imu_index = static_cast<size_t>(
        std::lower_bound(imu.begin(), imu.end(), first_timestamp,
                         [](const ov_core::ImuData &measurement, double timestamp) {
                           return measurement.timestamp < timestamp;
                         }) -
        imu.begin());
    size_t frames_logged = 0;
    for (size_t frame = 0; frame < stereo_pairs.size(); ++frame) {
      const auto [left_frame, right_frame] = stereo_pairs[frame];
      const double timestamp = camera0[left_frame].timestamp;
      if (timestamp < first_timestamp)
        continue;

      for (; imu_index < imu.size() && imu[imu_index].timestamp <= timestamp; ++imu_index) {
        system->feed_measurement_imu(imu[imu_index]);
        logger.push_imu(imu[imu_index]);
      }

      const cv::Mat left = cv::imread(camera0[left_frame].path, cv::IMREAD_GRAYSCALE);
      const cv::Mat right = cv::imread(camera1[right_frame].path, cv::IMREAD_GRAYSCALE);
      if (left.empty() || right.empty())
        throw std::runtime_error("unable to load stereo images at frame " + std::to_string(frame));
      ov_core::CameraData camera;
      camera.timestamp = timestamp;
      camera.sensor_ids = {0, 1};
      camera.images = {left, right};
      camera.masks = {cv::Mat::zeros(left.size(), CV_8UC1), cv::Mat::zeros(right.size(), CV_8UC1)};
      system->feed_measurement_camera(camera);

      if (!system->initialized())
        continue;
      Eigen::Matrix<double, 17, 1> groundtruth;
      if (ov_core::DatasetReader::get_gt_state(timestamp, groundtruth, gt_states)) {
        logger.log_frame(system, camera, groundtruth);
        ++frames_logged;
      }
    }

    logger.close();
    if (frames_logged == 0)
      throw std::runtime_error("OpenVINS initialized but no frames with ground truth were logged");
    std::cout << "[conformal] Stage-1 dump complete: " << output_h5
              << " frames=" << frames_logged
              << " stereo_pairs=" << stereo_pairs.size()
              << " unmatched_left=" << unmatched_left
              << " unmatched_right=" << unmatched_right << "\n";
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << "[conformal] ERROR: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
}
