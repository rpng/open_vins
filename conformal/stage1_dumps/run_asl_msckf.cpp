/* ROS-free EuRoC/ASL runner for conformal Stage 1. */

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <H5Cpp.h>
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

std::vector<hsize_t> dataset_shape(H5::H5File &file, const std::string &name) {
  H5::DataSet dataset = file.openDataSet(name);
  H5::DataSpace space = dataset.getSpace();
  std::vector<hsize_t> shape(static_cast<size_t>(space.getSimpleExtentNdims()));
  space.getSimpleExtentDims(shape.data());
  return shape;
}

size_t element_count(const std::vector<hsize_t> &shape) {
  size_t count = 1;
  for (hsize_t extent : shape)
    count *= static_cast<size_t>(extent);
  return count;
}

std::vector<double> read_double_vector(H5::H5File &file, const std::string &name) {
  H5::DataSet dataset = file.openDataSet(name);
  std::vector<double> values(element_count(dataset_shape(file, name)));
  dataset.read(values.data(), H5::PredType::NATIVE_DOUBLE);
  return values;
}

Eigen::MatrixXd read_matrix(H5::H5File &file, const std::string &name,
                            size_t expected_rows, size_t expected_columns) {
  const auto shape = dataset_shape(file, name);
  if (shape.size() != 2 || shape[0] != expected_rows || shape[1] != expected_columns)
    throw std::runtime_error(name + " has an unexpected matrix shape");
  const auto values = read_double_vector(file, name);
  Eigen::MatrixXd matrix(expected_rows, expected_columns);
  for (size_t row = 0; row < expected_rows; ++row)
    for (size_t column = 0; column < expected_columns; ++column)
      matrix(row, column) = values[row * expected_columns + column];
  return matrix;
}

Eigen::VectorXd read_vector(H5::H5File &file, const std::string &name, size_t expected_size) {
  const auto shape = dataset_shape(file, name);
  if (shape.size() != 1 || shape[0] != expected_size)
    throw std::runtime_error(name + " has an unexpected vector shape");
  const auto values = read_double_vector(file, name);
  return Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

class ImuSidecar {
public:
  explicit ImuSidecar(const std::string &path) {
    H5::H5File file(path, H5F_ACC_RDONLY);
    frame_timestamps_ = read_double_vector(file, "/frames/timestamp");
    const auto imu_shape = dataset_shape(file, "/frames/imu_sigmas");
    if (imu_shape.size() != 2 || imu_shape[0] != frame_timestamps_.size() || imu_shape[1] != 4)
      throw std::runtime_error("sidecar /frames/imu_sigmas must have shape [N,4]");
    imu_sigmas_ = read_double_vector(file, "/frames/imu_sigmas");
    if (frame_timestamps_.empty())
      throw std::runtime_error("sidecar contains no frame sigmas");
    for (double sigma : imu_sigmas_) {
      if (!std::isfinite(sigma) || sigma <= 0.0)
        throw std::runtime_error("sidecar contains invalid IMU sigma");
    }
  }

  ov_msckf::NoiseManager imu_noises(double timestamp) const {
    auto next = std::lower_bound(frame_timestamps_.begin(), frame_timestamps_.end(), timestamp);
    size_t index = 0;
    if (next == frame_timestamps_.end()) {
      index = frame_timestamps_.size() - 1;
    } else {
      index = static_cast<size_t>(next - frame_timestamps_.begin());
      if (index > 0 && std::abs(frame_timestamps_[index - 1] - timestamp) <
                           std::abs(frame_timestamps_[index] - timestamp))
        --index;
    }
    const size_t offset = index * 4;
    ov_msckf::NoiseManager noises;
    noises.sigma_w = imu_sigmas_[offset + 0];
    noises.sigma_a = imu_sigmas_[offset + 1];
    noises.sigma_wb = imu_sigmas_[offset + 2];
    noises.sigma_ab = imu_sigmas_[offset + 3];
    return noises;
  }

private:
  std::vector<double> frame_timestamps_;
  std::vector<double> imu_sigmas_;
};

class LiveNetA {
public:
  LiveNetA(const std::string &path, double sigma_scale)
      : sigma_scale_(sigma_scale) {
    if (!std::isfinite(sigma_scale_) || sigma_scale_ <= 0.0)
      throw std::runtime_error("Net-A visual scale must be finite and positive");
    H5::H5File file(path, H5F_ACC_RDONLY);
    encoder0_weight_ = read_matrix(file, "/weights/encoder0_weight", 64, 8);
    encoder0_bias_ = read_vector(file, "/weights/encoder0_bias", 64);
    encoder2_weight_ = read_matrix(file, "/weights/encoder2_weight", 64, 64);
    encoder2_bias_ = read_vector(file, "/weights/encoder2_bias", 64);
    pool_weight_ = read_matrix(file, "/weights/pool_weight", 64, 192);
    pool_bias_ = read_vector(file, "/weights/pool_bias", 64);
    head0_weight_ = read_matrix(file, "/weights/head0_weight", 64, 134);
    head0_bias_ = read_vector(file, "/weights/head0_bias", 64);
    head2_weight_ = read_matrix(file, "/weights/head2_weight", 1, 64);
    head2_bias_ = read_vector(file, "/weights/head2_bias", 1);

    const auto verification_shape = dataset_shape(file, "/verification/features");
    if (verification_shape.size() != 2 || verification_shape[1] != 8)
      throw std::runtime_error("Net-A verification features have an invalid shape");
    const size_t count = static_cast<size_t>(verification_shape[0]);
    const Eigen::MatrixXd features =
        read_matrix(file, "/verification/features", count, 8);
    const Eigen::VectorXd context =
        read_vector(file, "/verification/frame_context", 6);
    const Eigen::VectorXd expected =
        read_vector(file, "/verification/log_sigma", count);
    const Eigen::VectorXd actual = infer_log_sigma(features, context);
    const double max_error = (actual - expected).cwiseAbs().maxCoeff();
    if (!std::isfinite(max_error) || max_error > 1e-5)
      throw std::runtime_error("C++ Net-A parity check failed with max error " +
                               std::to_string(max_error));
    std::cout << "[conformal] live Net-A parity PASS max_abs_error="
              << max_error << "\n";
  }

  std::vector<double> predict(
      const std::vector<ov_msckf::MsckfVisualInput> &features,
      const ov_msckf::MsckfFrameContext &context) const {
    if (features.empty())
      return {};
    Eigen::MatrixXd feature_matrix(features.size(), 8);
    for (size_t row = 0; row < features.size(); ++row)
      for (size_t column = 0; column < 8; ++column)
        feature_matrix(row, column) = features[row][column];
    Eigen::VectorXd context_vector(6);
    for (size_t index = 0; index < 6; ++index)
      context_vector(index) = context[index];
    const Eigen::VectorXd log_sigma = infer_log_sigma(feature_matrix, context_vector);
    std::vector<double> sigma(features.size());
    for (size_t index = 0; index < features.size(); ++index) {
      const double clamped = std::max(-7.0, std::min(7.0, log_sigma(index)));
      sigma[index] = std::exp(clamped) * sigma_scale_;
    }
    ++batches_;
    predictions_ += features.size();
    return sigma;
  }

  size_t batches() const { return batches_.load(); }
  size_t predictions() const { return predictions_.load(); }

private:
  static Eigen::MatrixXd relu(const Eigen::MatrixXd &values) {
    return values.cwiseMax(0.0);
  }

  Eigen::VectorXd infer_log_sigma(
      const Eigen::MatrixXd &features,
      const Eigen::VectorXd &context) const {
    if (features.rows() < 1 || features.cols() != 8 || context.size() != 6)
      throw std::runtime_error("invalid live Net-A inference dimensions");
    Eigen::MatrixXd encoded = features * encoder0_weight_.transpose();
    encoded.rowwise() += encoder0_bias_.transpose();
    encoded = relu(encoded);
    encoded = encoded * encoder2_weight_.transpose();
    encoded.rowwise() += encoder2_bias_.transpose();
    encoded = relu(encoded);

    const Eigen::VectorXd mean = encoded.colwise().mean().transpose();
    const Eigen::VectorXd maximum = encoded.colwise().maxCoeff().transpose();
    const Eigen::MatrixXd centered = encoded.rowwise() - mean.transpose();
    const Eigen::VectorXd variance =
        centered.array().square().colwise().mean().matrix().transpose();
    const Eigen::VectorXd standard_deviation =
        variance.cwiseMax(1e-12).cwiseSqrt();
    Eigen::VectorXd pooled(192);
    pooled.segment(0, 64) = mean;
    pooled.segment(64, 64) = maximum;
    pooled.segment(128, 64) = standard_deviation;
    const Eigen::VectorXd summary = pool_weight_ * pooled + pool_bias_;

    Eigen::MatrixXd head_input(features.rows(), 134);
    head_input.block(0, 0, features.rows(), 64) = encoded;
    head_input.block(0, 64, features.rows(), 64).rowwise() = summary.transpose();
    head_input.block(0, 128, features.rows(), 6).rowwise() = context.transpose();
    Eigen::MatrixXd hidden = head_input * head0_weight_.transpose();
    hidden.rowwise() += head0_bias_.transpose();
    hidden = relu(hidden);
    Eigen::MatrixXd output = hidden * head2_weight_.transpose();
    output.rowwise() += head2_bias_.transpose();
    return output.col(0);
  }

  double sigma_scale_ = 1.0;
  Eigen::MatrixXd encoder0_weight_, encoder2_weight_, pool_weight_;
  Eigen::MatrixXd head0_weight_, head2_weight_;
  Eigen::VectorXd encoder0_bias_, encoder2_bias_, pool_bias_;
  Eigen::VectorXd head0_bias_, head2_bias_;
  mutable std::atomic<size_t> batches_{0};
  mutable std::atomic<size_t> predictions_{0};
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
  if (argc < 5 || argc > 9) {
    std::cerr << "usage: run_asl_msckf <estimator_config.yaml> <seq_dir> <sequence_name> <out.h5> "
                 "[start_offset_s] [imu_sidecar.h5] [net_a.h5] [visual_scale]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string config_path = argv[1];
    const std::string sequence_directory = argv[2];
    const std::string sequence_name = argv[3];
    const std::string output_h5 = argv[4];
    const double start_offset = argc >= 6 ? std::stod(argv[5]) : 0.0;
    const std::string imu_sidecar_path = argc >= 7 ? argv[6] : "";
    const std::string net_a_path = argc >= 8 ? argv[7] : "";
    const double visual_scale = argc >= 9 ? std::stod(argv[8]) : 1.0;
    if (start_offset < 0.0)
      throw std::runtime_error("start_offset_s must be non-negative");
    if (!net_a_path.empty() && imu_sidecar_path.empty())
      throw std::runtime_error("live Net-A runs require an IMU sidecar");
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
    std::shared_ptr<ImuSidecar> imu_sidecar;
    if (!imu_sidecar_path.empty()) {
      imu_sidecar = std::make_shared<ImuSidecar>(imu_sidecar_path);
      std::cout << "[conformal] loaded IMU sidecar: " << imu_sidecar_path << "\n";
    }
    std::shared_ptr<LiveNetA> live_net_a;
    if (!net_a_path.empty()) {
      live_net_a = std::make_shared<LiveNetA>(net_a_path, visual_scale);
      system->set_msckf_sigma_provider(
          [live_net_a](
              const std::vector<ov_msckf::MsckfVisualInput> &features,
              const ov_msckf::MsckfFrameContext &context) {
            return live_net_a->predict(features, context);
          });
      std::cout << "[conformal] loaded causal live Net-A: " << net_a_path
                << " visual_scale=" << visual_scale << "\n";
    }
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
      const size_t left_frame = stereo_pairs[frame].first;
      const size_t right_frame = stereo_pairs[frame].second;
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
      if (imu_sidecar)
        system->set_imu_noises(imu_sidecar->imu_noises(timestamp));
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
    if (live_net_a) {
      std::cout << "[conformal] live Net-A batches=" << live_net_a->batches()
                << " predictions=" << live_net_a->predictions() << "\n";
    }
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
