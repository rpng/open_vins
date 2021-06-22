#include <sstream>
#include <algorithm>
#include <numeric>

#include <opencv2/core.hpp>

#include "core/VioManagerOptions.h"

template<typename T>
__inline
void ReadParamCVFs(T& param, cv::FileStorage fs, std::string paramname)
{
    cv::FileNode node = fs[paramname];
    if(!node.empty()) {
        if(typeid (bool) == typeid (T)) {
            int tmp;
            fs[paramname] >> tmp;
            param = tmp;
        } else {
            fs[paramname] >> param;
        }
    }
}

void StringToVector(std::vector<double>& data_vec, const std::string& str) {
    std::string data_str = str;
    size_t n = 0;
    for(auto& d : data_str) {
        if(d == ',') {
            d = ' ';
            n++;
        }
    }
    n = n ? n + 1 : n;
    data_vec.resize(n);

    std::stringstream ss;
    ss << data_str;
    for(size_t i = 0; i < n; i++) {
        ss >> data_vec[i];
    }
}

void ReadConfig(ov_msckf::VioManagerOptions& params, const std::string& config_file_path) {
    cv::FileStorage fs;
    fs.open(config_file_path, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "Error! cannot open config file: " << config_file_path << std::endl;
        return;
    }

    std::string data_str;
    std::vector<double> data_vec;

    // key parameters
    ReadParamCVFs(params.state_options.num_cameras, fs, "max_cameras");
    ReadParamCVFs(params.state_options.do_calib_camera_pose, fs, "calib_cam_extrinsics");
    ReadParamCVFs(params.state_options.do_calib_camera_intrinsics, fs, "calib_cam_intrinsics");
    ReadParamCVFs(params.state_options.do_calib_camera_timeoffset, fs, "calib_cam_timeoffset");
    ReadParamCVFs(params.calib_camimu_dt, fs, "calib_camimu_dt");

    // world/filter parameters
    ReadParamCVFs(params.state_options.do_fej, fs, "use_fej");
    ReadParamCVFs(params.state_options.imu_avg, fs, "use_imuavg");
    ReadParamCVFs(params.state_options.use_rk4_integration, fs, "use_rk4int");
    ReadParamCVFs(params.use_stereo, fs, "use_stereo");

    ReadParamCVFs(params.state_options.max_clone_size, fs, "max_clones");
    ReadParamCVFs(params.state_options.max_slam_features, fs, "max_slam");
    ReadParamCVFs(params.state_options.max_slam_in_update, fs, "max_slam_in_update");
    ReadParamCVFs(params.state_options.max_msckf_in_update, fs, "max_msckf_in_update");

    ReadParamCVFs(params.dt_slam_delay, fs, "dt_slam_delay");
    ReadParamCVFs(params.init_window_time, fs, "init_window_time");
    ReadParamCVFs(params.init_imu_thresh, fs, "init_imu_thresh");
    ReadParamCVFs(data_str, fs, "gravity");
    StringToVector(data_vec, data_str);
    params.gravity << data_vec[0], data_vec[1], data_vec[2];

    ReadParamCVFs(data_str, fs, "feat_rep_msckf");
    params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string(data_str);
    ReadParamCVFs(data_str, fs, "feat_rep_slam");
    params.state_options.feat_rep_slam = LandmarkRepresentation::from_string(data_str);
    ReadParamCVFs(data_str, fs, "feat_rep_aruco");
    params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string(data_str);

    // zero velocity update parameters
    ReadParamCVFs(params.try_zupt, fs, "try_zupt");
    ReadParamCVFs(params.zupt_options.chi2_multipler, fs, "zupt_chi2_multipler");
    ReadParamCVFs(params.zupt_max_velocity, fs, "zupt_max_velocity");
    ReadParamCVFs(params.zupt_noise_multiplier, fs, "zupt_noise_multiplier");

    // tracker/extractor parameters
    ReadParamCVFs(params.use_klt, fs, "use_klt");
    ReadParamCVFs(params.num_pts, fs, "num_pts");
    ReadParamCVFs(params.fast_threshold, fs, "fast_threshold");
    ReadParamCVFs(params.grid_x, fs, "grid_x");
    ReadParamCVFs(params.grid_y, fs, "grid_y");
    ReadParamCVFs(params.min_px_dist, fs, "min_px_dist");
    ReadParamCVFs(params.knn_ratio, fs, "knn_ratio");
    ReadParamCVFs(params.downsample_cameras, fs, "downsample_cameras");
    ReadParamCVFs(params.use_multi_threading, fs, "multi_threading");

    // aruco tag/mapping properties
    ReadParamCVFs(params.use_aruco, fs, "use_aruco");
    ReadParamCVFs(params.state_options.max_aruco_features, fs, "num_aruco");
    ReadParamCVFs(params.downsize_aruco, fs, "downsize_aruco");

    // sensor noise values / update
    ReadParamCVFs(params.msckf_options.sigma_pix, fs, "up_msckf_sigma_px");
    ReadParamCVFs(params.msckf_options.chi2_multipler, fs, "up_msckf_chi2_multipler");
    ReadParamCVFs(params.slam_options.sigma_pix, fs, "up_slam_sigma_px");
    ReadParamCVFs(params.slam_options.chi2_multipler, fs, "up_slam_chi2_multipler");
    ReadParamCVFs(params.aruco_options.sigma_pix, fs, "up_aruco_sigma_px");
    ReadParamCVFs(params.aruco_options.chi2_multipler, fs, "up_aruco_chi2_multipler");

    ReadParamCVFs(params.imu_noises.sigma_w, fs, "gyroscope_noise_density");
    ReadParamCVFs(params.imu_noises.sigma_wb, fs, "gyroscope_random_walk");
    ReadParamCVFs(params.imu_noises.sigma_a, fs, "accelerometer_noise_density");
    ReadParamCVFs(params.imu_noises.sigma_ab, fs, "accelerometer_random_walk");

    for(int idx = 0; idx < params.state_options.num_cameras; idx++) {
        std::string idx_str = "cam" + std::to_string(idx);

        // camera intrinsics
        ReadParamCVFs(data_str, fs, idx_str + "_wh");
        StringToVector(data_vec, data_str);
        params.camera_wh.insert({idx, {data_vec[0], data_vec[1]}});

        bool is_fisheye;
        ReadParamCVFs(is_fisheye, fs, idx_str + "_is_fisheye");
        params.camera_fisheye.insert({idx, is_fisheye});

        std::vector<double> K_vec, D_vec;
        ReadParamCVFs(data_str, fs, idx_str + "_k");
        StringToVector(K_vec, data_str);
        ReadParamCVFs(data_str, fs, idx_str + "_d");
        StringToVector(D_vec, data_str);
        Eigen::Matrix<double,8,1> cam_calib;
        cam_calib << K_vec[0], K_vec[1], K_vec[2], K_vec[3], D_vec[0], D_vec[1], D_vec[2], D_vec[3];
        params.camera_intrinsics.insert({idx, cam_calib});

        // camera extrinsics
        ReadParamCVFs(data_str, fs, idx_str + "_T_CtoI");
        StringToVector(data_vec, data_str);

        Eigen::Matrix4d T_CtoI;
        T_CtoI << data_vec[0],  data_vec[1],  data_vec[2],  data_vec[3],
                  data_vec[4],  data_vec[5],  data_vec[6],  data_vec[7],
                  data_vec[8],  data_vec[9],  data_vec[10], data_vec[11],
                  data_vec[12], data_vec[13], data_vec[14], data_vec[15];
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);
        params.camera_extrinsics.insert({idx, cam_eigen});


        // Halve if we are doing downsampling
        if(params.downsample_cameras) {
            // wh
            params.camera_wh.at(idx).first /= 2.0;
            params.camera_wh.at(idx).second /= 2.0;

            // K
            params.camera_intrinsics.at(idx)[0] /= 2.0;
            params.camera_intrinsics.at(idx)[1] /= 2.0;
            params.camera_intrinsics.at(idx)[2] /= 2.0;
            params.camera_intrinsics.at(idx)[3] /= 2.0;
        }
    }

    params.state_options.num_unique_cameras = (int)params.stereo_pairs.size() + params.state_options.num_cameras;
}


