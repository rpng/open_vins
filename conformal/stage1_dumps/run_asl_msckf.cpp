/*
 * run_asl_msckf.cpp  --  ROS-free ASL/EuRoC dataset runner for conformal Stage 1.
 *
 * Companion to: Part IV, Section 18.2 of conformal_explainer.pdf
 *               ("run_asl_msckf.cpp -- the custom dataset runner (~150 lines)").
 *
 * WHAT THIS FILE IS FOR
 * ---------------------
 * OpenVINS ships an ASL *ground-truth* reader (ov_core::DatasetReader::load_gt_file)
 * but NO ASL image/IMU reader for the main pipeline -- run_subscribe_msckf.cpp expects
 * ROS topics. This small program reads the raw ASL layout directly:
 *     <seq>/mav0/cam0/data/*.png , <seq>/mav0/cam0/data.csv   (timestamp,filename)
 *     <seq>/mav0/cam1/data/*.png , <seq>/mav0/cam1/data.csv   (stereo)
 *     <seq>/mav0/imu0/data.csv                                 (t, wx,wy,wz, ax,ay,az)
 *     <seq>/mav0/state_groundtruth_estimate0/data.csv          (Vicon/Leica GT)
 * and feeds ov_msckf::VioManager directly, so the whole training path is ROS-free
 * (Section 18.1: build with `cmake -DENABLE_ROS=OFF`, needs only OpenCV/Eigen/Ceres[/HDF5]).
 *
 * It also drives conformal::DiagnosticsLogger to dump the Stage-1 HDF5 (Section 18.3).
 *
 * TRAP 1 (Section 20, IMU ordering): feed ALL IMU samples with timestamp < t_frame
 * BEFORE feeding the camera frame at t_frame. Out-of-order feeding makes propagation
 * silently wrong and the error masquerades as sensor noise -- the networks would then
 * learn to model a bug as if it were physics. The merge loop below enforces this.
 *
 * USAGE:
 *   run_asl_msckf <estimator_config.yaml> <seq_dir> <out.h5>
 * e.g.
 *   run_asl_msckf config/euroc_mav/estimator_config.yaml /data/EuRoC/MH_01_easy MH_01_easy.h5
 *
 * TODO(intern): this is a scaffold with the control flow and OpenVINS API calls spelled
 * out; the CSV/PNG parsing bodies are marked TODO. Target ~150 lines when filled in.
 */

#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/VioManager.h"          // ov_msckf::VioManager, VioManagerOptions
#include "utils/dataset_reader.h"     // ov_core::DatasetReader::load_gt_file / get_gt_state
#include "utils/sensor_data.h"        // ov_core::ImuData, ov_core::CameraData
#include "utils/print.h"
#include "cam/CamBase.h"

#include "DiagnosticsLogger.hpp"      // conformal::DiagnosticsLogger (this project)

using namespace ov_msckf;

// A single ASL image record (timestamp + path on disk), loaded lazily to save RAM.
struct ImageRecord {
  double timestamp = 0.0;
  std::string path;
};

// ---- ASL parsing helpers ---------------------------------------------------------------
// TODO(intern): fill these three in. ASL CSV timestamps are in NANOSECONDS -> divide by 1e9.
static std::vector<ov_core::ImuData> load_imu_csv(const std::string &imu_csv);
static std::vector<ImageRecord> load_cam_csv(const std::string &cam_csv, const std::string &img_dir);
static void load_gt(const std::string &gt_csv, std::map<double, Eigen::Matrix<double, 17, 1>> &gt);

void signal_callback_handler(int signum) { std::exit(signum); }

int main(int argc, char **argv) {
  if (argc < 4) {
    PRINT_ERROR(RED "usage: run_asl_msckf <config.yaml> <seq_dir> <out.h5>\n" RESET);
    return EXIT_FAILURE;
  }
  const std::string config_path = argv[1];
  const std::string seq_dir = argv[2];
  const std::string out_h5 = argv[3];
  signal(SIGINT, signal_callback_handler);

  // --- Load config exactly like run_simulation.cpp, but with no ROS node handle. -------
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  params.num_opencv_threads = 0;          // repeatability -- Stage 1 must be deterministic
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix the config\n" RESET);
    return EXIT_FAILURE;
  }

  auto sys = std::make_shared<VioManager>(params);
  conformal::DiagnosticsLogger logger(out_h5);

  // --- Load the raw ASL streams for this sequence. -------------------------------------
  std::vector<ov_core::ImuData> imu = load_imu_csv(seq_dir + "/mav0/imu0/data.csv");
  std::vector<ImageRecord> cam0 = load_cam_csv(seq_dir + "/mav0/cam0/data.csv", seq_dir + "/mav0/cam0/data");
  std::vector<ImageRecord> cam1 = load_cam_csv(seq_dir + "/mav0/cam1/data.csv", seq_dir + "/mav0/cam1/data");
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  // Gate 1 (Section 19.1): ground truth is in the Vicon *body* frame; the T_BS transform
  // from the sensor YAML must be applied to bring it to the IMU frame before use as a
  // training target. TODO(intern): apply T_BS inside load_gt (or right after).
  load_gt(seq_dir + "/mav0/state_groundtruth_estimate0/data.csv", gt_states);

  // --- Main merge loop: strictly time-ordered IMU-then-camera feeding (TRAP 1). --------
  size_t imu_idx = 0;
  for (size_t c = 0; c < cam0.size(); ++c) {
    const double t_frame = cam0[c].timestamp;

    // Feed every IMU sample up to (and including) this frame's timestamp FIRST.
    for (; imu_idx < imu.size() && imu[imu_idx].timestamp <= t_frame; ++imu_idx) {
      sys->feed_measurement_imu(imu[imu_idx]);
      logger.push_imu(imu[imu_idx]); // rolling 20x6 window for Net B
    }

    // Build the (stereo) camera message and feed it.
    ov_core::CameraData cam;
    cam.timestamp = t_frame;
    cam.sensor_ids = {0, 1};
    cam.images = {cv::imread(cam0[c].path, cv::IMREAD_GRAYSCALE),
                  cv::imread(cam1[c].path, cv::IMREAD_GRAYSCALE)};
    cam.masks = {cv::Mat::zeros(cam.images[0].size(), CV_8UC1),
                 cv::Mat::zeros(cam.images[1].size(), CV_8UC1)};
    sys->feed_measurement_camera(cam);

    if (!sys->initialized())
      continue;

    // Interpolate ground truth to this frame time and dump diagnostics (Section 18.3).
    Eigen::Matrix<double, 17, 1> gt_imustate;
    if (ov_core::DatasetReader::get_gt_state(t_frame, gt_imustate, gt_states)) {
      logger.log_frame(sys, cam, gt_imustate);
    }
  }

  logger.close();
  PRINT_INFO(GREEN "[conformal] Stage-1 dump complete -> %s\n" RESET, out_h5.c_str());
  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------------------
// TODO(intern): implement the three ASL loaders below.
//   * ASL CSV files have a one-line "#..." header; skip it.
//   * Timestamps are int64 nanoseconds -> seconds = t_ns * 1e-9.
//   * imu0/data.csv columns:  t, w_x,w_y,w_z, a_x,a_y,a_z
//   * camX/data.csv columns:  t, filename   (image is <img_dir>/<filename>)
//   * state_groundtruth_estimate0/data.csv:  t, p(3), q_wxyz(4), v(3), bw(3), ba(3)
//     -> pack into [t, q_GtoI(4), p(3), v(3), bg(3), ba(3)] to match load_gt_file's layout,
//        applying the Gate-1 T_BS body->IMU transform.
// Keep the whole file ROS-free: no <ros/ros.h>, no message_filters, no rosbag.
// ----------------------------------------------------------------------------------------
static std::vector<ov_core::ImuData> load_imu_csv(const std::string &imu_csv) {
  (void)imu_csv;
  return {}; // TODO(intern)
}
static std::vector<ImageRecord> load_cam_csv(const std::string &cam_csv, const std::string &img_dir) {
  (void)cam_csv;
  (void)img_dir;
  return {}; // TODO(intern)
}
static void load_gt(const std::string &gt_csv, std::map<double, Eigen::Matrix<double, 17, 1>> &gt) {
  // The bundled reader parses the ASL GT layout; reuse it, then apply T_BS (Gate 1).
  ov_core::DatasetReader::load_gt_file(gt_csv, gt);
  // TODO(intern): apply T_BS from the sensor YAML here.
}
