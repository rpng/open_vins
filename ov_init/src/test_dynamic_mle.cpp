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

#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <ceres/ceres.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#if ROS_AVAILABLE == 1
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#endif

#include "ceres/Factor_GenericPrior.h"
#include "ceres/Factor_ImageReprojCalib.h"
#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "init/InertialInitializerOptions.h"
#include "sim/SimulatorInit.h"
#include "utils/helper.h"

#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "utils/colors.h"
#include "utils/sensor_data.h"

using namespace ov_init;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Launch our ros node
  ros::init(argc, argv, "test_dynamic_mle");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);

  // Topics to publish
  auto pub_pathimu = nh->advertise<nav_msgs::Path>("/ov_msckf/pathimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathimu.getTopic().c_str());
  auto pub_pathgt = nh->advertise<nav_msgs::Path>("/ov_msckf/pathgt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathgt.getTopic().c_str());
  auto pub_loop_point = nh->advertise<sensor_msgs::PointCloud>("/ov_msckf/loop_feats", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_loop_point.getTopic().c_str());
  auto pub_points_sim = nh->advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_sim", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_sim.getTopic().c_str());
#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create the simulator
  InertialInitializerOptions params;
  params.print_and_load(parser);
  params.print_and_load_simulation(parser);
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  SimulatorInit sim(params);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Ceres problem stuff
  // NOTE: By default the problem takes ownership of the memory
  ceres::Problem problem;

  // Our system states (map from time to index)
  std::map<double, int> map_states;
  std::vector<double *> ceres_vars_ori;
  std::vector<double *> ceres_vars_pos;
  std::vector<double *> ceres_vars_vel;
  std::vector<double *> ceres_vars_bias_g;
  std::vector<double *> ceres_vars_bias_a;

  // Feature states (3dof p_FinG)
  std::map<size_t, int> map_features;
  std::vector<double *> ceres_vars_feat;
  typedef std::vector<std::pair<Factor_ImageReprojCalib *, std::vector<double *>>> factpair;
  std::map<size_t, factpair> map_features_delayed; // only valid as long as problem is

  // Setup extrinsic calibration q_ItoC, p_IinC (map from camera id to index)
  std::map<size_t, int> map_calib_cam2imu;
  std::vector<double *> ceres_vars_calib_cam2imu_ori;
  std::vector<double *> ceres_vars_calib_cam2imu_pos;

  // Setup intrinsic calibration focal, center, distortion (map from camera id to index)
  std::map<size_t, int> map_calib_cam;
  std::vector<double *> ceres_vars_calib_cam_intrinsics;

  // Set the optimization settings
  // NOTE: We use dense schur since after eliminating features we have a dense problem
  // NOTE: http://ceres-solver.org/solving_faqs.html#solving
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.num_threads = 6;
  options.max_solver_time_in_seconds = 9999;
  options.max_num_iterations = 20;
  options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;
  // options.function_tolerance = 1e-5;
  // options.gradient_tolerance = 1e-4 * options.function_tolerance;

  // DEBUG: check analytical jacobians using ceres finite differences
  // options.check_gradients = true;
  // options.gradient_check_relative_precision = 1e-4;
  // options.gradient_check_numeric_derivative_relative_step_size = 1e-8;

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Random number generator used to perturb the groundtruth features
  // NOTE: It seems that if this too large it can really prevent good optimization
  // NOTE: Values greater 5cm seems to fail, not sure if this is reasonable or not...
  std::mt19937 gen_state_perturb(params.sim_seed_preturb);
  double feat_noise = 0.05; // meters

  // Buffer our camera image
  double buffer_timecam = -1;
  double buffer_timecam_last = -1;
  std::vector<int> buffer_camids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> buffer_feats;
  auto imu_readings = std::make_shared<std::vector<ov_core::ImuData>>();

  // Simulate!
  signal(SIGINT, signal_callback_handler);
  while (sim.ok()) {

    // IMU: get the next simulated IMU measurement if we have it
    ov_core::ImuData message_imu;
    bool hasimu = sim.get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
    if (hasimu) {
      imu_readings->push_back(message_imu);
    }

    // CAM: get the next simulated camera uv measurements if we have them
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim.get_next_cam(time_cam, camids, feats);
    if (hascam) {
      if (buffer_timecam != -1) {

        // Get our predicted state at the requested camera timestep
        double timestamp_k = buffer_timecam_last;
        double timestamp_k1 = buffer_timecam;
        Eigen::Matrix<double, 16, 1> state_k1;
        std::shared_ptr<ov_core::CpiV1> cpi = nullptr;
        if (map_states.empty()) {

          // Add the first ever pose to the problem
          // TODO: do not initialize from the groundtruth pose
          double time1 = timestamp_k1 + sim.get_true_parameters().calib_camimu_dt;
          Eigen::Matrix<double, 17, 1> gt_imustate;
          bool success = sim.get_state(time1, gt_imustate);
          assert(success);
          state_k1 = gt_imustate.block(1, 0, 16, 1);

        } else {

          // Get our previous state timestamp (newest time) and biases to integrate with
          assert(timestamp_k != -1);
          Eigen::Vector4d quat_k;
          for (int i = 0; i < 4; i++) {
            quat_k(i) = ceres_vars_ori.at(map_states.at(timestamp_k))[i];
          }
          Eigen::Vector3d pos_k, vel_k, bias_g_k, bias_a_k;
          for (int i = 0; i < 3; i++) {
            pos_k(i) = ceres_vars_pos.at(map_states.at(timestamp_k))[i];
            vel_k(i) = ceres_vars_vel.at(map_states.at(timestamp_k))[i];
            bias_g_k(i) = ceres_vars_bias_g.at(map_states.at(timestamp_k))[i];
            bias_a_k(i) = ceres_vars_bias_a.at(map_states.at(timestamp_k))[i];
          }
          Eigen::Vector3d gravity;
          gravity << 0.0, 0.0, params.gravity_mag;

          // Preintegrate from previous state
          // Then append a new state with preintegration factor
          // TODO: estimate the timeoffset? don't use the true?
          double time0 = timestamp_k + sim.get_true_parameters().calib_camimu_dt;
          double time1 = timestamp_k1 + sim.get_true_parameters().calib_camimu_dt;
          auto readings = InitializerHelper::select_imu_readings(*imu_readings, time0, time1);
          cpi = std::make_shared<ov_core::CpiV1>(params.sigma_w, params.sigma_wb, params.sigma_a, params.sigma_ab, false);
          cpi->setLinearizationPoints(bias_g_k, bias_a_k, quat_k, gravity);
          for (size_t k = 0; k < readings.size() - 1; k++) {
            auto imu0 = readings.at(k);
            auto imu1 = readings.at(k + 1);
            cpi->feed_IMU(imu0.timestamp, imu1.timestamp, imu0.wm, imu0.am, imu1.wm, imu1.am);
          }
          assert(timestamp_k1 > timestamp_k);

          // Compute the predicted state
          state_k1.block(0, 0, 4, 1) = ov_core::quat_multiply(cpi->q_k2tau, quat_k);
          state_k1.block(4, 0, 3, 1) =
              pos_k + vel_k * cpi->DT - 0.5 * cpi->grav * std::pow(cpi->DT, 2) + ov_core::quat_2_Rot(quat_k).transpose() * cpi->alpha_tau;
          state_k1.block(7, 0, 3, 1) = vel_k - cpi->grav * cpi->DT + ov_core::quat_2_Rot(quat_k).transpose() * cpi->beta_tau;
          state_k1.block(10, 0, 3, 1) = bias_g_k;
          state_k1.block(13, 0, 3, 1) = bias_a_k;
        }

        // ================================================================
        //  ADDING GRAPH STATE / ESTIMATES!
        // ================================================================

        // Load our state variables into our allocated state pointers
        auto *var_ori = new double[4];
        for (int i = 0; i < 4; i++) {
          var_ori[i] = state_k1(0 + i, 0);
        }
        auto *var_pos = new double[3];
        auto *var_vel = new double[3];
        auto *var_bias_g = new double[3];
        auto *var_bias_a = new double[3];
        for (int i = 0; i < 3; i++) {
          var_pos[i] = state_k1(4 + i, 0);
          var_vel[i] = state_k1(7 + i, 0);
          var_bias_g[i] = state_k1(10 + i, 0);
          var_bias_a[i] = state_k1(13 + i, 0);
        }

        // Now actually create the parameter block in the ceres problem
        auto ceres_jplquat = new State_JPLQuatLocal();
        problem.AddParameterBlock(var_ori, 4, ceres_jplquat);
        problem.AddParameterBlock(var_pos, 3);
        problem.AddParameterBlock(var_vel, 3);
        problem.AddParameterBlock(var_bias_g, 3);
        problem.AddParameterBlock(var_bias_a, 3);

        // Fix this first ever pose to constrain the problem
        // On the Comparison of Gauge Freedom Handling in Optimization-Based Visual-Inertial State Estimation
        // Zichao Zhang; Guillermo Gallego; Davide Scaramuzza
        // https://ieeexplore.ieee.org/document/8354808
        if (map_states.empty()) {

          // Construct state and prior
          Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(7, 1);
          for (int i = 0; i < 4; i++) {
            x_lin(0 + i) = var_ori[i];
          }
          for (int i = 0; i < 3; i++) {
            x_lin(4 + i) = var_pos[i];
            // x_lin(7 + i) = var_bias_g[i];
            // x_lin(10 + i) = var_bias_a[i];
          }
          Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(4, 1);
          Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(4, 4);
          prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1e-8, 2); // 4dof unobservable yaw and position
          // prior_Info.block(4, 4, 3, 3) *= 1.0 / std::pow(1e-1, 2); // bias_g prior
          // prior_Info.block(7, 7, 3, 3) *= 1.0 / std::pow(1e-1, 2); // bias_a prior

          // Construct state type and ceres parameter pointers
          std::vector<std::string> x_types;
          std::vector<double *> factor_params;
          factor_params.push_back(var_ori);
          x_types.emplace_back("quat_yaw");
          factor_params.push_back(var_pos);
          x_types.emplace_back("vec3");
          // factor_params.push_back(var_bias_g);
          // x_types.emplace_back("vec3");
          // factor_params.push_back(var_bias_a);
          // x_types.emplace_back("vec3");

          // Append it to the problem
          auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
          problem.AddResidualBlock(factor_prior, nullptr, factor_params);
          // problem.SetParameterBlockConstant(var_ori);
          // problem.SetParameterBlockConstant(var_pos);
        }

        // Append to our historical vector of states
        map_states.insert({timestamp_k1, (int)ceres_vars_ori.size()});
        ceres_vars_ori.push_back(var_ori);
        ceres_vars_pos.push_back(var_pos);
        ceres_vars_vel.push_back(var_vel);
        ceres_vars_bias_g.push_back(var_bias_g);
        ceres_vars_bias_a.push_back(var_bias_a);
        buffer_timecam_last = timestamp_k1;

        // ================================================================
        //  ADDING GRAPH FACTORS!
        // ================================================================

        // Append the new IMU factor
        if (cpi != nullptr) {
          assert(timestamp_k != -1);
          std::vector<double *> factor_params;
          factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k)));
          factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k)));
          factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k)));
          factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k)));
          factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k)));
          factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k1)));
          factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k1)));
          factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k1)));
          factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k1)));
          factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k1)));
          auto *factor_imu = new Factor_ImuCPIv1(cpi->DT, cpi->grav, cpi->alpha_tau, cpi->beta_tau, cpi->q_k2tau, cpi->b_a_lin,
                                                 cpi->b_w_lin, cpi->J_q, cpi->J_b, cpi->J_a, cpi->H_b, cpi->H_a, cpi->P_meas);
          problem.AddResidualBlock(factor_imu, nullptr, factor_params);
        }

        // Then, append new feature observations factors seen from this frame (initialize features as needed)
        // We first loop through each camera and for each we append measurements as needed
        assert(buffer_camids.size() == buffer_feats.size());
        for (size_t n = 0; n < buffer_camids.size(); n++) {

          // First make sure we have calibration states added
          int cam_id = buffer_camids.at(n);
          if (map_calib_cam2imu.find(cam_id) == map_calib_cam2imu.end()) {
            auto *var_calib_ori = new double[4];
            for (int i = 0; i < 4; i++) {
              var_calib_ori[i] = params.camera_extrinsics.at(cam_id)(0 + i, 0);
            }
            auto *var_calib_pos = new double[3];
            for (int i = 0; i < 3; i++) {
              var_calib_pos[i] = params.camera_extrinsics.at(cam_id)(4 + i, 0);
            }
            auto ceres_calib_jplquat = new State_JPLQuatLocal();
            problem.AddParameterBlock(var_calib_ori, 4, ceres_calib_jplquat);
            problem.AddParameterBlock(var_calib_pos, 3);
            map_calib_cam2imu.insert({cam_id, (int)ceres_vars_calib_cam2imu_ori.size()});
            ceres_vars_calib_cam2imu_ori.push_back(var_calib_ori);
            ceres_vars_calib_cam2imu_pos.push_back(var_calib_pos);

            // Construct state and prior
            Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(7, 1);
            for (int i = 0; i < 4; i++) {
              x_lin(0 + i) = var_calib_ori[i];
            }
            for (int i = 0; i < 3; i++) {
              x_lin(4 + i) = var_calib_pos[i];
            }
            Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(6, 1);
            Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(6, 6);
            prior_Info.block(0, 0, 3, 3) *= 1.0 / std::pow(0.001, 2);
            prior_Info.block(3, 3, 3, 3) *= 1.0 / std::pow(0.01, 2);

            // Construct state type and ceres parameter pointers
            std::vector<std::string> x_types;
            std::vector<double *> factor_params;
            factor_params.push_back(var_calib_ori);
            x_types.emplace_back("quat");
            factor_params.push_back(var_calib_pos);
            x_types.emplace_back("vec3");
            if (!params.init_dyn_mle_opt_calib) {
              problem.SetParameterBlockConstant(var_calib_ori);
              problem.SetParameterBlockConstant(var_calib_pos);
            } else {
              auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
              problem.AddResidualBlock(factor_prior, nullptr, factor_params);
            }
          }
          bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(params.camera_intrinsics.at(cam_id)) != nullptr);
          if (map_calib_cam.find(cam_id) == map_calib_cam.end()) {
            auto *var_calib_cam = new double[8];
            for (int i = 0; i < 8; i++) {
              var_calib_cam[i] = params.camera_intrinsics.at(cam_id)->get_value()(i, 0);
            }
            problem.AddParameterBlock(var_calib_cam, 8);
            map_calib_cam.insert({cam_id, (int)ceres_vars_calib_cam_intrinsics.size()});
            ceres_vars_calib_cam_intrinsics.push_back(var_calib_cam);

            // Construct state and prior
            Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(8, 1);
            for (int i = 0; i < 8; i++) {
              x_lin(0 + i) = var_calib_cam[i];
            }
            Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(8, 1);
            Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(8, 8);
            prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1.0, 2);
            prior_Info.block(4, 4, 4, 4) *= 1.0 / std::pow(0.005, 2);

            // Construct state type and ceres parameter pointers
            std::vector<std::string> x_types;
            std::vector<double *> factor_params;
            factor_params.push_back(var_calib_cam);
            x_types.emplace_back("vec8");
            if (!params.init_dyn_mle_opt_calib) {
              problem.SetParameterBlockConstant(var_calib_cam);
            } else {
              auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
              problem.AddResidualBlock(factor_prior, nullptr, factor_params);
            }
          }

          // Now loop through each feature observed at this k+1 timestamp
          for (auto const &feat : buffer_feats.at(n)) {
            size_t featid = feat.first;
            Eigen::Vector2d uv_raw;
            uv_raw << feat.second(0), feat.second(1);

            // Next make sure that we have the feature state
            // Normally, we should need to triangulate this once we have enough measurements
            // TODO: do not initialize features from the groundtruth map
            if (map_features.find(featid) == map_features.end()) {
              assert(params.use_stereo);
              Eigen::Vector3d p_FinG = sim.get_map().at(featid);
              auto *var_feat = new double[3];
              for (int i = 0; i < 3; i++) {
                std::normal_distribution<double> w(0, 1);
                var_feat[i] = p_FinG(i) + feat_noise * w(gen_state_perturb);
              }
              map_features.insert({featid, (int)ceres_vars_feat.size()});
              map_features_delayed.insert({featid, factpair()});
              ceres_vars_feat.push_back(var_feat);
            }

            // Factor parameters it is a function of
            std::vector<double *> factor_params;
            factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k1)));
            factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k1)));
            factor_params.push_back(ceres_vars_feat.at(map_features.at(featid)));
            factor_params.push_back(ceres_vars_calib_cam2imu_ori.at(map_calib_cam2imu.at(cam_id)));
            factor_params.push_back(ceres_vars_calib_cam2imu_pos.at(map_calib_cam2imu.at(cam_id)));
            factor_params.push_back(ceres_vars_calib_cam_intrinsics.at(map_calib_cam.at(cam_id)));
            auto *factor_pinhole = new Factor_ImageReprojCalib(uv_raw, params.sigma_pix, is_fisheye);
            if (map_features_delayed.find(featid) != map_features_delayed.end()) {
              map_features_delayed.at(featid).push_back({factor_pinhole, factor_params});
            } else {
              ceres::LossFunction *loss_function = nullptr;
              // ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
              problem.AddResidualBlock(factor_pinhole, loss_function, factor_params);
            }
          }
        }
      }
      buffer_timecam = time_cam;
      buffer_camids = camids;
      buffer_feats = feats;

      // If a delayed feature has enough measurements we can add it to the problem!
      // We will wait for enough observations before adding the parameter
      // This should make it much more stable since the parameter / optimization is more constrained
      size_t min_num_meas_to_optimize = 10;
      auto it0 = map_features_delayed.begin();
      while (it0 != map_features_delayed.end()) {
        size_t featid = (*it0).first;
        auto factors = (*it0).second;
        if (factors.size() >= min_num_meas_to_optimize) {
          problem.AddParameterBlock(ceres_vars_feat.at(map_features.at(featid)), 3);
          for (auto const &factorpair : factors) {
            auto factor_pinhole = factorpair.first;
            auto factor_params = factorpair.second;
            ceres::LossFunction *loss_function = nullptr;
            // ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(factor_pinhole, loss_function, factor_params);
          }
          it0 = map_features_delayed.erase(it0);
        } else {
          it0++;
        }
      }

      // ================================================================
      //  PERFORM ACTUAL OPTIMIZATION!
      // ================================================================

      // We can try to optimize every few frames, but this can cause the IMU to drift
      // Thus this can't be too large, nor too small to reduce the computation
      if (map_states.size() % 10 == 0 && map_states.size() > min_num_meas_to_optimize) {

        // COMPUTE: error before optimization
        double error_before = 0.0;
        if (!map_features.empty()) {
          for (auto &feat : map_features) {
            Eigen::Vector3d pos1, pos2;
            pos1 << ceres_vars_feat.at(feat.second)[0], ceres_vars_feat.at(feat.second)[1], ceres_vars_feat.at(feat.second)[2];
            pos2 = sim.get_map().at(feat.first);
            error_before += (pos2 - pos1).norm();
          }
          error_before /= (double)map_features.size();
        }

        // Optimize the ceres graph
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        PRINT_INFO("[CERES]: %s\n", summary.message.c_str());
        PRINT_INFO("[CERES]: %d iterations | %zu states, %zu features | %d parameters and %d residuals | cost %.4e => %.4e\n",
                   (int)summary.iterations.size(), map_states.size(), map_features.size(), summary.num_parameters, summary.num_residuals,
                   summary.initial_cost, summary.final_cost);

        // COMPUTE: error after optimization
        double error_after = 0.0;
        if (!map_features.empty()) {
          for (auto &feat : map_features) {
            Eigen::Vector3d pos1, pos2;
            pos1 << ceres_vars_feat.at(feat.second)[0], ceres_vars_feat.at(feat.second)[1], ceres_vars_feat.at(feat.second)[2];
            pos2 = sim.get_map().at(feat.first);
            error_after += (pos2 - pos1).norm();
          }
          error_after /= (double)map_features.size();
        }
        PRINT_INFO("[CERES]: map error %.4f => %.4f (meters) for %zu features\n", error_before, error_after, map_features.size());

        // Print feature positions to console
        // https://gist.github.com/goldbattle/177a6b2cccb4b4208e687b0abae4bc9f
        //      for (auto &feat : map_features) {
        //        size_t featid = feat.first;
        //        std::cout << featid << ",";
        //        std::cout << ceres_vars_feat[map_features[featid]][0] << "," << ceres_vars_feat[map_features[featid]][1] << ","
        //                  << ceres_vars_feat[map_features[featid]][2] << ",";
        //        std::cout << sim.get_map().at(featid)[0] << "," << sim.get_map().at(featid)[1] << "," << sim.get_map().at(featid)[2] <<
        //        ","; std::cout << map_features_num_meas[feat.first] << ";" << std::endl;
        //      }
      }

#if ROS_AVAILABLE == 1
      // Visualize in RVIZ our current estimates and
      if (map_states.size() % 1 == 0) {

        // Pose states
        nav_msgs::Path arrEST, arrGT;
        arrEST.header.stamp = ros::Time::now();
        arrEST.header.frame_id = "global";
        arrGT.header.stamp = ros::Time::now();
        arrGT.header.frame_id = "global";
        for (auto &statepair : map_states) {
          geometry_msgs::PoseStamped poseEST, poseGT;
          poseEST.header.stamp = ros::Time(statepair.first);
          poseEST.header.frame_id = "global";
          poseEST.pose.orientation.x = ceres_vars_ori[statepair.second][0];
          poseEST.pose.orientation.y = ceres_vars_ori[statepair.second][1];
          poseEST.pose.orientation.z = ceres_vars_ori[statepair.second][2];
          poseEST.pose.orientation.w = ceres_vars_ori[statepair.second][3];
          poseEST.pose.position.x = ceres_vars_pos[statepair.second][0];
          poseEST.pose.position.y = ceres_vars_pos[statepair.second][1];
          poseEST.pose.position.z = ceres_vars_pos[statepair.second][2];
          Eigen::Matrix<double, 17, 1> gt_imustate;
          bool success = sim.get_state(statepair.first + sim.get_true_parameters().calib_camimu_dt, gt_imustate);
          assert(success);
          poseGT.header.stamp = ros::Time(statepair.first);
          poseGT.header.frame_id = "global";
          poseGT.pose.orientation.x = gt_imustate(1);
          poseGT.pose.orientation.y = gt_imustate(2);
          poseGT.pose.orientation.z = gt_imustate(3);
          poseGT.pose.orientation.w = gt_imustate(4);
          poseGT.pose.position.x = gt_imustate(5);
          poseGT.pose.position.y = gt_imustate(6);
          poseGT.pose.position.z = gt_imustate(7);
          arrEST.poses.push_back(poseEST);
          arrGT.poses.push_back(poseGT);
        }
        pub_pathimu.publish(arrEST);
        pub_pathgt.publish(arrGT);

        // Features ESTIMATES pointcloud
        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.frame_id = "global";
        point_cloud.header.stamp = ros::Time::now();
        for (auto &featpair : map_features) {
          if (map_features_delayed.find(featpair.first) != map_features_delayed.end())
            continue;
          geometry_msgs::Point32 p;
          p.x = (float)ceres_vars_feat[map_features[featpair.first]][0];
          p.y = (float)ceres_vars_feat[map_features[featpair.first]][1];
          p.z = (float)ceres_vars_feat[map_features[featpair.first]][2];
          point_cloud.points.push_back(p);
        }
        pub_loop_point.publish(point_cloud);

        // Features GROUNDTRUTH pointcloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "global";
        cloud.header.stamp = ros::Time::now();
        cloud.width = map_features.size();
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = false; // there may be invalid points
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(map_features.size());
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        for (auto &featpair : map_features) {
          if (map_features_delayed.find(featpair.first) != map_features_delayed.end())
            continue;
          *out_x = (float)sim.get_map().at(featpair.first)(0); // no change in id since no tracker is used
          ++out_x;
          *out_y = (float)sim.get_map().at(featpair.first)(1); // no change in id since no tracker is used
          ++out_y;
          *out_z = (float)sim.get_map().at(featpair.first)(2); // no change in id since no tracker is used
          ++out_z;
        }
        pub_points_sim.publish(cloud);
      }
#endif
    }
  }

  // Done!
  return EXIT_SUCCESS;
};