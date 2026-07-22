/*
 * DiagnosticsLogger.hpp  --  Stage-1 training-data extractor for conformal.
 *
 * Companion to: Part IV, Section 18.3 of conformal_explainer.pdf ("DiagnosticsLogger.hpp
 *               -- the only real patch"), and the Net input tables on p.14.
 *
 * WHAT THIS FILE IS FOR
 * ---------------------
 * conformal trains two tiny networks (Net A: per-feature sigma_pix; Net B: 4 IMU noise
 * densities) OFFLINE from data dumped by an otherwise-unmodified OpenVINS run
 * (see Section 8.3, "Training: fully offline"). This header is the single mechanism
 * that reaches into OpenVINS and writes, per frame and per feature, everything the
 * networks read PLUS the residuals and ground-truth needed to build training targets.
 *
 * DESIGN CONSTRAINT (the paper's central defensibility claim, Section 18.3 / 4.4):
 *   ov_msckf/ and ov_core/ must stay byte-for-byte unmodified. This logger therefore
 *   only READS already-public OpenVINS getters:
 *     - ov_msckf::VioManager::get_state()                -> state x_hat and covariance P
 *     - ov_msckf::VioManager::get_propagator()
 *     - ov_core::TrackBase::get_last_obs() / get_last_ids()
 *     - ov_core::TrackBase::get_feature_database()       -> per-feature tracks
 *     - ov_msckf::StateHelper::get_full_covariance()     -> for NEES / P dumps
 *   It is included and driven by conformal/stage1_dumps/run_asl_msckf.cpp. Nothing here
 *   is compiled into the core libraries.
 *
 * TWO IMPLEMENTATION TRAPS (Section 20) -- both belong to Stage 1, honour them here:
 *   TRAP 2 (residual dumping order): dump residuals BEFORE outlier rejection. The
 *   rejected features are precisely the ones carrying the degradation signal; gating
 *   them away before logging trains the networks only on measurements that already
 *   behaved. See log_visual_residuals() below.
 *   (TRAP 1, IMU ordering, is enforced by the runner -- see run_asl_msckf.cpp.)
 *
 * OUTPUT: one HDF5 file per sequence, with (at least) these groups:
 *   /meta                     sequence name, config hash, git SHA of ov_* (must be clean)
 *   /imu_windows              [N_frames, 20, 6]  raw gyro+accel windows        -> Net B input
 *   /imu_preint_error         [N_frames]         preintegration error          -> Net B target
 *   /features/frame_XXXXX     per-feature diagnostics table (see PerFeatureRow) -> Net A input
 *   /features/.../residual    ||z_i - h(x_gt, p_i)|| per feature               -> Net A target
 *   /frame_diag               per-frame table (see PerFrameRow)                -> Net A frame ctx
 *   /state/x_hat, /state/P    estimate and covariance                          -> NEES (Section 7)
 *   /gt/x                     interpolated ground truth (after Gate-1 T_BS fix) -> all targets
 *
 * TODO(intern): pick an HDF5 binding. HighFive (header-only, wraps libhdf5) is the
 * least-friction choice and matches the "no ROS, only OpenCV/Eigen/Ceres + HDF5"
 * build in Section 18.1. The method bodies below are intentionally stubs.
 */

#ifndef conformal_DIAGNOSTICS_LOGGER_HPP
#define conformal_DIAGNOSTICS_LOGGER_HPP

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// Forward declarations only -- we never modify these types, we only read them.
namespace ov_msckf {
class VioManager;
} // namespace ov_msckf
namespace ov_core {
struct ImuData;
struct CameraData;
} // namespace ov_core

namespace conformal {

/// Per-feature diagnostics -- exactly the Net A inputs from the p.14 table.
struct PerFeatureRow {
  size_t feature_id = 0;
  double u = 0.0, v = 0.0;             ///< pixel location (raw, distorted)
  double klt_fwd_bwd_error = 0.0;      ///< KLT forward-backward round-trip error (Section 8.1)
  double shi_tomasi_response = 0.0;    ///< Shi-Tomasi / corner response
  double track_age = 0.0;             ///< number of frames this feature has survived
  double parallax = 0.0;              ///< triangulation parallax angle
  double stereo_lr_consistency = 0.0; ///< left-right consistency (stereo rigs only)
  double residual_norm = 0.0;         ///< ||z_i - h(x_gt, p_i)|| -- Net A TRAINING TARGET
  bool passed_chi2_gate = false;      ///< did it survive the Mahalanobis gate? (log it, don't filter on it)
};

/// Per-frame diagnostics -- the frame-level Net A context from the p.14 table.
struct PerFrameRow {
  double timestamp = 0.0;
  int num_tracked = 0;                ///< features tracked into this frame
  int num_lost = 0;                   ///< features lost at this frame
  double ransac_inlier_ratio = 0.0;   ///< RANSAC inlier ratio
  double mean_brightness = 0.0;       ///< frame mean brightness (low-light proxy)
  int state_dim_n = 0;                ///< n = 15 + 6*N_clones -- the NEES reference (Section 4.1)
};

/**
 * @brief Accumulates Stage-1 training data from a live OpenVINS run and dumps HDF5.
 *
 * Usage (from run_asl_msckf.cpp):
 *   conformal::DiagnosticsLogger logger("MH_01_easy.h5");
 *   ... per IMU sample:   logger.push_imu(msg);
 *   ... per camera frame: logger.log_frame(vio, cam_msg, gt_interp);
 *   logger.close();
 */
class DiagnosticsLogger {
public:
  explicit DiagnosticsLogger(const std::string &output_h5_path);
  ~DiagnosticsLogger();

  /// Feed every raw IMU sample here so we can cut the 20x6 Net-B windows aligned to frames.
  /// TODO(intern): keep a rolling buffer; on each frame, snapshot the last 20 samples.
  void push_imu(const ov_core::ImuData &imu);

  /**
   * @brief Log one camera frame: features, residuals, IMU window, state, covariance, GT.
   * @param vio        the (unmodified) OpenVINS manager -- read-only access to state/track
   * @param cam        the camera message just fed to the filter
   * @param gt_imustate interpolated ground truth [t,q_GtoI,p,v,bg,ba] AFTER the Gate-1 T_BS fix
   *
   * TRAP 2 lives here: collect per-feature rows for ALL current tracks (including those the
   * MSCKF will reject), compute residuals against gt, and record passed_chi2_gate as a FLAG.
   * Never drop a row because it failed the gate.
   */
  void log_frame(const std::shared_ptr<ov_msckf::VioManager> &vio, const ov_core::CameraData &cam,
                 const Eigen::Matrix<double, 17, 1> &gt_imustate);

  /// Flush and close the HDF5 file.
  void close();

private:
  // TODO(intern): implement against your chosen HDF5 binding (HighFive recommended).
  std::vector<PerFeatureRow> collect_feature_rows(const std::shared_ptr<ov_msckf::VioManager> &vio,
                                                  const Eigen::Matrix<double, 17, 1> &gt_imustate);
  PerFrameRow collect_frame_row(const std::shared_ptr<ov_msckf::VioManager> &vio, double timestamp);
  Eigen::MatrixXd snapshot_imu_window() const; ///< last 20 samples x 6 channels, Net B input

  std::string output_path_;
  std::deque<ov_core::ImuData> imu_buffer_; ///< rolling buffer for 20x6 windows (TRAP 1: keep in order)
  // ... HDF5 handle(s) go here.
};

} // namespace conformal

#endif // conformal_DIAGNOSTICS_LOGGER_HPP
