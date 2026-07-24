#include "DiagnosticsLogger.hpp"

#include <H5Cpp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "cam/CamBase.h"
#include "core/VioManager.h"
#include "feat/Feature.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "track/TrackBase.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "utils/dataset_reader.h"
#include "utils/quat_ops.h"

namespace {

constexpr hsize_t kUnlimited = H5S_UNLIMITED;

H5::DataSet create_extendible(H5::H5File &file, const std::string &name,
                              const std::vector<hsize_t> &row_shape,
                              const std::vector<hsize_t> &chunk_shape) {
  std::vector<hsize_t> initial(1 + row_shape.size(), 0);
  std::vector<hsize_t> maximum(1 + row_shape.size(), 0);
  std::vector<hsize_t> chunk(1 + chunk_shape.size(), 0);
  maximum[0] = kUnlimited;
  chunk[0] = 128;
  for (size_t i = 0; i < row_shape.size(); ++i) {
    initial[i + 1] = row_shape[i];
    maximum[i + 1] = row_shape[i];
    chunk[i + 1] = chunk_shape[i];
  }
  H5::DataSpace space(static_cast<int>(initial.size()), initial.data(), maximum.data());
  H5::DSetCreatPropList properties;
  properties.setChunk(static_cast<int>(chunk.size()), chunk.data());
  properties.setDeflate(4);
  return file.createDataSet(name, H5::PredType::NATIVE_DOUBLE, space, properties);
}

void append_row(H5::DataSet &dataset, hsize_t &rows, const double *data,
                const std::vector<hsize_t> &row_shape) {
  std::vector<hsize_t> dimensions(1 + row_shape.size());
  dimensions[0] = rows + 1;
  std::copy(row_shape.begin(), row_shape.end(), dimensions.begin() + 1);
  dataset.extend(dimensions.data());

  H5::DataSpace file_space = dataset.getSpace();
  std::vector<hsize_t> offset(dimensions.size(), 0);
  offset[0] = rows;
  std::vector<hsize_t> count = dimensions;
  count[0] = 1;
  file_space.selectHyperslab(H5S_SELECT_SET, count.data(), offset.data());
  H5::DataSpace memory_space(static_cast<int>(count.size()), count.data());
  dataset.write(data, H5::PredType::NATIVE_DOUBLE, memory_space, file_space);
  ++rows;
}

void write_string_attribute(H5::H5Object &object, const std::string &name,
                            const std::string &value) {
  H5::StrType type(H5::PredType::C_S1, H5T_VARIABLE);
  H5::DataSpace scalar(H5S_SCALAR);
  H5::Attribute attribute = object.createAttribute(name, type, scalar);
  const char *raw = value.c_str();
  attribute.write(type, &raw);
}

bool lookup_gt(double timestamp, Eigen::Matrix<double, 17, 1> &imustate,
               const std::map<double, Eigen::Matrix<double, 17, 1>> &states) {
  if (states.empty())
    return false;
  auto upper = states.lower_bound(timestamp);
  auto nearest = upper;
  if (upper == states.end()) {
    nearest = std::prev(states.end());
  } else if (upper != states.begin()) {
    const auto lower = std::prev(upper);
    if (std::abs(lower->first - timestamp) < std::abs(upper->first - timestamp))
      nearest = lower;
  }
  if (nearest == states.end() || std::abs(nearest->first - timestamp) >= 0.10)
    return false;
  const auto &raw = nearest->second;
  imustate(0) = nearest->first;
  imustate.block<4, 1>(1, 0) << raw(5), raw(6), raw(7), raw(4);
  imustate.block<3, 1>(5, 0) = raw.block<3, 1>(1, 0);
  imustate.block<3, 1>(8, 0) = raw.block<3, 1>(8, 0);
  imustate.block<3, 1>(11, 0) = raw.block<3, 1>(11, 0);
  imustate.block<3, 1>(14, 0) = raw.block<3, 1>(14, 0);
  return true;
}

double gt_reprojection_error(
    const std::shared_ptr<ov_msckf::VioManager> &vio,
    const std::shared_ptr<const ov_core::Feature> &feature,
    std::map<double, Eigen::Matrix<double, 17, 1>> &gt_states) {
  if (!feature)
    return std::numeric_limits<double>::quiet_NaN();
  const auto state = vio->get_state();
  Eigen::Matrix<double, 17, 1> gt_at_update;
  if (!lookup_gt(state->_timestamp, gt_at_update, gt_states))
    return std::numeric_limits<double>::quiet_NaN();

  // Landmarks use OpenVINS' local global frame E, while EuRoC poses use the
  // mocap global frame G. Align E to G at the update timestamp before using a
  // landmark with any GT camera pose.
  const Eigen::Matrix3d R_GtoI_update = ov_core::quat_2_Rot(gt_at_update.block<4, 1>(1, 0));
  const Eigen::Matrix3d R_EtoI_update = state->_imu->Rot();
  const Eigen::Matrix3d R_EtoG = R_GtoI_update.transpose() * R_EtoI_update;
  const Eigen::Vector3d p_IinG_update = gt_at_update.block<3, 1>(5, 0);
  const Eigen::Vector3d p_IinE_update = state->_imu->pos();
  const Eigen::Vector3d p_EinG = p_IinG_update - R_EtoG * p_IinE_update;
  const Eigen::Vector3d p_FinG = R_EtoG * feature->p_FinG + p_EinG;

  std::vector<double> errors;
  for (const auto &camera_entry : feature->timestamps) {
    const size_t camera_id = camera_entry.first;
    if (feature->uvs.find(camera_id) == feature->uvs.end() ||
        state->_calib_IMUtoCAM.find(camera_id) == state->_calib_IMUtoCAM.end() ||
        state->_cam_intrinsics_cameras.find(camera_id) == state->_cam_intrinsics_cameras.end())
      continue;
    const auto &timestamps = camera_entry.second;
    const auto &measurements = feature->uvs.at(camera_id);
    const size_t count = std::min(timestamps.size(), measurements.size());
    for (size_t index = 0; index < count; ++index) {
      Eigen::Matrix<double, 17, 1> gt;
      if (!lookup_gt(timestamps[index], gt, gt_states))
        continue;
      const Eigen::Matrix3d R_GtoI = ov_core::quat_2_Rot(gt.block<4, 1>(1, 0));
      const Eigen::Vector3d p_IinG = gt.block<3, 1>(5, 0);
      const auto calibration = state->_calib_IMUtoCAM.at(camera_id);
      const Eigen::Matrix3d R_GtoC = calibration->Rot() * R_GtoI;
      const Eigen::Vector3d p_CinG = p_IinG - R_GtoC.transpose() * calibration->pos();
      const Eigen::Vector3d p_FinC = R_GtoC * (p_FinG - p_CinG);
      if (p_FinC.z() <= 1e-6)
        continue;
      const Eigen::Vector2d normalized(p_FinC.x() / p_FinC.z(), p_FinC.y() / p_FinC.z());
      const Eigen::Vector2d predicted = state->_cam_intrinsics_cameras.at(camera_id)->distort_d(normalized);
      errors.emplace_back((measurements[index].cast<double>() - predicted).norm());
    }
  }
  if (errors.empty())
    return std::numeric_limits<double>::quiet_NaN();
  return std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
}

} // namespace

namespace conformal {

struct DiagnosticsLogger::Impl {
  explicit Impl(const std::string &path) : file(path, H5F_ACC_TRUNC) {
    file.createGroup("/meta");
    file.createGroup("/frames");
    file.createGroup("/features");
    timestamp = create_extendible(file, "/frames/timestamp", {}, {});
    state = create_extendible(file, "/frames/state", {16}, {16});
    groundtruth = create_extendible(file, "/frames/groundtruth", {17}, {17});
    covariance = create_extendible(file, "/frames/covariance_imu15", {15, 15}, {15, 15});
    imu_window = create_extendible(file, "/frames/imu_window", {20, 6}, {20, 6});
    frame_diagnostics = create_extendible(file, "/frames/diagnostics", {5}, {5});
    feature_diagnostics = create_extendible(file, "/features/diagnostics", {12}, {12});
    write_string_attribute(frame_diagnostics, "columns",
                           "timestamp,num_tracked,num_lost,mean_brightness,state_dim");
    write_string_attribute(feature_diagnostics, "columns",
                           "timestamp,feature_id,track_measurements,last_camera,u,v,filter_residual_norm,"
                           "gt_reprojection_residual_norm,chi2,chi2_threshold,sigma_pix,passed_chi2_gate");
  }

  H5::H5File file;
  H5::DataSet timestamp, state, groundtruth, covariance, imu_window;
  H5::DataSet frame_diagnostics, feature_diagnostics;
  hsize_t frame_rows = 0;
  hsize_t feature_rows = 0;
};

DiagnosticsLogger::DiagnosticsLogger(const std::string &output_h5_path,
                                     const std::string &sequence,
                                     const std::string &config_path)
    : impl_(new Impl(output_h5_path)) {
  H5::Group meta = impl_->file.openGroup("/meta");
  write_string_attribute(meta, "sequence", sequence);
  write_string_attribute(meta, "config_path", config_path);
  write_string_attribute(meta, "schema_version", "1");
  write_string_attribute(meta, "state_order", "q_GtoI_xyzw,p_IinG,v_IinG,b_g,b_a");
  write_string_attribute(meta, "covariance_order", "dtheta,p,v,b_g,b_a");
}

DiagnosticsLogger::~DiagnosticsLogger() {
  try {
    close();
  } catch (...) {
  }
}

void DiagnosticsLogger::push_imu(const ov_core::ImuData &imu) {
  imu_buffer_.push_back(imu);
  while (imu_buffer_.size() > 20)
    imu_buffer_.pop_front();
}

void DiagnosticsLogger::log_msckf_diagnostic(
    const std::shared_ptr<ov_msckf::VioManager> &vio,
    const ov_msckf::MsckfFeatureDiagnostic &diagnostic,
    std::map<double, Eigen::Matrix<double, 17, 1>> &gt_states) {
  if (closed_)
    throw std::runtime_error("cannot append to a closed diagnostics logger");

  double last_camera = -1.0;
  double last_timestamp = -std::numeric_limits<double>::infinity();
  double u = std::numeric_limits<double>::quiet_NaN();
  double v = std::numeric_limits<double>::quiet_NaN();
  if (diagnostic.feature) {
    for (const auto &camera_entry : diagnostic.feature->timestamps) {
      const auto uv_it = diagnostic.feature->uvs.find(camera_entry.first);
      if (uv_it == diagnostic.feature->uvs.end())
        continue;
      const size_t count = std::min(camera_entry.second.size(), uv_it->second.size());
      for (size_t index = 0; index < count; ++index) {
        if (camera_entry.second[index] > last_timestamp) {
          last_timestamp = camera_entry.second[index];
          last_camera = static_cast<double>(camera_entry.first);
          u = uv_it->second[index](0);
          v = uv_it->second[index](1);
        }
      }
    }
  }

  const std::array<double, 12> row = {
      diagnostic.timestamp,
      static_cast<double>(diagnostic.feature_id),
      static_cast<double>(diagnostic.track_measurements),
      last_camera,
      u,
      v,
      diagnostic.filter_residual_norm,
      gt_reprojection_error(vio, diagnostic.feature, gt_states),
      diagnostic.chi2,
      diagnostic.chi2_threshold,
      diagnostic.sigma_pix,
      diagnostic.passed_chi2_gate ? 1.0 : 0.0,
  };
  append_row(impl_->feature_diagnostics, impl_->feature_rows, row.data(), {12});
}

void DiagnosticsLogger::log_frame(
    const std::shared_ptr<ov_msckf::VioManager> &vio,
    const ov_core::CameraData &cam,
    const Eigen::Matrix<double, 17, 1> &gt_imustate) {
  if (closed_)
    throw std::runtime_error("cannot append to a closed diagnostics logger");
  if (imu_buffer_.size() < 20)
    return;

  const auto state = vio->get_state();
  const Eigen::VectorXd state_value = state->_imu->value();
  const Eigen::MatrixXd covariance = ov_msckf::StateHelper::get_marginal_covariance(state, {state->_imu});
  if (state_value.rows() != 16 || covariance.rows() != 15 || covariance.cols() != 15)
    throw std::runtime_error("unexpected OpenVINS IMU state/covariance dimensions");

  std::array<double, 120> window{};
  for (size_t index = 0; index < 20; ++index) {
    for (size_t axis = 0; axis < 3; ++axis) {
      window[index * 6 + axis] = imu_buffer_[index].wm(axis);
      window[index * 6 + 3 + axis] = imu_buffer_[index].am(axis);
    }
  }

  int num_tracked = 0;
  const auto ids = vio->get_feature_tracker()->get_last_ids();
  for (const auto &camera_ids : ids)
    num_tracked += static_cast<int>(camera_ids.second.size());
  const int num_lost = std::max(0, previous_num_tracked_ - num_tracked);
  previous_num_tracked_ = num_tracked;
  const double mean_brightness = cam.images.empty() ? std::numeric_limits<double>::quiet_NaN()
                                                     : cv::mean(cam.images.front())[0];
  const std::array<double, 5> frame_row = {
      cam.timestamp,
      static_cast<double>(num_tracked),
      static_cast<double>(num_lost),
      mean_brightness,
      static_cast<double>(state->max_covariance_size()),
  };

  append_row(impl_->timestamp, impl_->frame_rows, &cam.timestamp, {});
  // All frame datasets share frame_rows; append_row increments, so use a local
  // row counter initialized to the just-written row for the sibling datasets.
  hsize_t sibling_row = impl_->frame_rows - 1;
  append_row(impl_->state, sibling_row, state_value.data(), {16});
  sibling_row = impl_->frame_rows - 1;
  append_row(impl_->groundtruth, sibling_row, gt_imustate.data(), {17});
  sibling_row = impl_->frame_rows - 1;
  append_row(impl_->covariance, sibling_row, covariance.data(), {15, 15});
  sibling_row = impl_->frame_rows - 1;
  append_row(impl_->imu_window, sibling_row, window.data(), {20, 6});
  sibling_row = impl_->frame_rows - 1;
  append_row(impl_->frame_diagnostics, sibling_row, frame_row.data(), {5});
}

void DiagnosticsLogger::close() {
  if (closed_)
    return;
  impl_->file.flush(H5F_SCOPE_GLOBAL);
  closed_ = true;
  // Destroy datasets before H5File (Impl declaration order guarantees this),
  // allowing HDF5 to write the final object headers and end-of-file marker.
  impl_.reset();
}

} // namespace conformal
