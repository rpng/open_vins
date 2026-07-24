#ifndef CONFORMAL_DIAGNOSTICS_LOGGER_HPP
#define CONFORMAL_DIAGNOSTICS_LOGGER_HPP

#include <Eigen/Eigen>
#include <deque>
#include <map>
#include <memory>
#include <string>

#include "update/ConformalHooks.h"
#include "utils/sensor_data.h"

namespace ov_msckf {
class VioManager;
}

namespace conformal {

/** HDF5 writer used by the ASL Stage-1 runner.
 *
 * The schema is flat and append-only so interrupted sequence runs remain easy
 * to inspect with h5py/h5dump. Feature candidates are logged from the MSCKF
 * callback before chi-squared rejection.
 */
class DiagnosticsLogger {
public:
  DiagnosticsLogger(const std::string &output_h5_path, const std::string &sequence,
                    const std::string &config_path);
  ~DiagnosticsLogger();

  DiagnosticsLogger(const DiagnosticsLogger &) = delete;
  DiagnosticsLogger &operator=(const DiagnosticsLogger &) = delete;

  void push_imu(const ov_core::ImuData &imu);

  void log_msckf_diagnostic(
      const std::shared_ptr<ov_msckf::VioManager> &vio,
      const ov_msckf::MsckfFeatureDiagnostic &diagnostic,
      std::map<double, Eigen::Matrix<double, 17, 1>> &gt_states);

  void log_frame(const std::shared_ptr<ov_msckf::VioManager> &vio,
                 const ov_core::CameraData &cam,
                 const Eigen::Matrix<double, 17, 1> &gt_imustate);

  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::deque<ov_core::ImuData> imu_buffer_;
  int previous_num_tracked_ = 0;
  bool closed_ = false;
};

} // namespace conformal

#endif // CONFORMAL_DIAGNOSTICS_LOGGER_HPP
