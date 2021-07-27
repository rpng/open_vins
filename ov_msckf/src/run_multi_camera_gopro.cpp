/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
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

#include <memory>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "utils/sensor_data.h"
#include "./read_config.h"

using namespace ov_msckf;

std::shared_ptr<VioManager> sys;

void ReadImu(std::vector<std::pair<double, std::array<double, 6>>>& imu_data_vec, const std::string& imu_file_path) {
    ifstream ifs;
    ifs.open(imu_file_path.c_str());
    imu_data_vec.reserve(5000);

    while(!ifs.eof())
    {
        string line;
        getline(ifs, line);
        if (line[0] == '#')
            continue;

        if(!line.empty())
        {
            for(auto& a : line) {
                if(a == ',')
                    a = ' ';
            }

            std::pair<double, std::array<double, 6>> data;

            std::stringstream ss;
            ss << line;
            ss >> data.first >> data.second[0] >> data.second[1] >> data.second[2] >>
                    data.second[3] >> data.second[4] >> data.second[5];
            data.first /= 1e9;

            imu_data_vec.push_back(data);
        }
    }
}

void ReadGTFile(std::map<double, Eigen::Matrix<double, 17, 1>>& gt_states, const std::string& gt_file_path) {
    // Clear any old data
    gt_states.clear();

    ifstream ifs;
    ifs.open(gt_file_path.c_str());

    while(!ifs.eof())
    {
        string line;
        getline(ifs, line);
        if (line[0] == '#')
            continue;

        if(!line.empty())
        {
            for(auto& a : line) {
                if(a == ',')
                    a = ' ';
            }

            Eigen::Matrix<double, 17, 1> data = Eigen::Matrix<double, 17, 1>::Zero();

            std::stringstream ss;
            ss << line;
            for(size_t i = 0; i < 17; i++) {
                ss >> data[i];
            }
            gt_states.insert({1e-9 * data(0, 0), data});
        }
    }
}

bool get_gt_state(double timestep, Eigen::Matrix<double,17,1> &imustate, std::map<double, Eigen::Matrix<double,17,1>>& gt_states) {

    // Check that we even have groundtruth loaded
    if (gt_states.empty()) {
        printf(RED "Groundtruth data loaded is empty, make sure you call load before asking for a state.\n" RESET);
        return false;
    }

    // Loop through gt states and find the closest time stamp
    double closest_time = std::numeric_limits<double>::max();
    auto it0 = gt_states.begin();
    while(it0 != gt_states.end()) {
        if(std::abs(it0->first-timestep) < std::abs(closest_time-timestep)) {
            closest_time = it0->first;
        }
        it0++;
    }

    // If close to this timestamp, then use it
    if(std::abs(closest_time-timestep) < 0.005) {
        //printf("init DT = %.4f\n", std::abs(closest_time-timestep));
        //printf("timestamp = %.15f\n", closest_time);
        timestep = closest_time;
    }

    // Check that we have the timestamp in our GT file
    if(gt_states.find(timestep) == gt_states.end()) {
        printf(YELLOW "Unable to find %.6f timestamp in GT file, wrong GT file loaded???\n" RESET,timestep);
        return false;
    }

    // Get the GT state vector
    Eigen::Matrix<double, 17, 1> state = gt_states[timestep];

    // Our "fixed" state vector from the ETH GT format [q,p,v,bg,ba]
    imustate(0, 0) = timestep; //time
    imustate(1, 0) = state(5, 0); //quat
    imustate(2, 0) = state(6, 0);
    imustate(3, 0) = state(7, 0);
    imustate(4, 0) = state(4, 0);
    imustate(5, 0) = state(1, 0); //pos
    imustate(6, 0) = state(2, 0);
    imustate(7, 0) = state(3, 0);
    imustate(8, 0) = state(8, 0); //vel
    imustate(9, 0) = state(9, 0);
    imustate(10, 0) = state(10, 0);
    imustate(11, 0) = state(11, 0); //bg
    imustate(12, 0) = state(12, 0);
    imustate(13, 0) = state(13, 0);
    imustate(14, 0) = state(14, 0); //ba
    imustate(15, 0) = state(15, 0);
    imustate(16, 0) = state(16, 0);

    // Success!
    return true;
}

// Main function
int main(int argc, char** argv)
{
    if(argc != 2) {
        std::cout << "Usage: ./exe path_to_config_file" << std::endl;
        return -1;
    }

    std::string config_file_path(argv[1]);

    std::string video_folder = "";
    std::string imu_file_path = "";
    std::string gt_file_path = "";
    std::string output_pose_file_path = "pose.txt";
    double t_start = 0, t_end = -1;

    cv::FileStorage fs;
    fs.open(config_file_path, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "Error! cannot open config file: " << config_file_path << std::endl;
        return -1;
    }
    ReadParamCVFs(video_folder, fs, "video_folder");
    ReadParamCVFs(imu_file_path, fs, "imu_file_path");
    ReadParamCVFs(gt_file_path, fs, "gt_file_path");
    ReadParamCVFs(output_pose_file_path, fs, "output_pose_file_path");
    ReadParamCVFs(t_start, fs, "t_start");
    ReadParamCVFs(t_end, fs, "t_end");
    fs.release();

    // Create our VIO system
    VioManagerOptions params;
    ReadConfig(params, config_file_path);

    sys = std::make_shared<VioManager>(params);

    std::ofstream ofs_pose_cam0;
    ofs_pose_cam0.open(output_pose_file_path);
    if(!ofs_pose_cam0.is_open()) {
        std::cout << "Error! cannot open file " << output_pose_file_path << std::endl;
        return 1;
    }

    std::ofstream ofs_pose_imu;
    ofs_pose_imu.open(output_pose_file_path + ".imu");
    if(!ofs_pose_imu.is_open()) {
        std::cout << "Error! cannot open file " << output_pose_file_path + ".imu" << std::endl;
        return 1;
    }

    // --------------  Read IMU data --------------------
    std::vector<std::pair<double, std::array<double, 6>>> imu_data_vec;
    ReadImu(imu_data_vec, imu_file_path);

    // -------------- Read camera timestamps ------------
    size_t num_cameras = params.state_options.num_cameras;
    std::vector<cv::VideoCapture> caps(num_cameras);
    for(size_t idx_camera = 0; idx_camera < num_cameras; idx_camera++) {
        std::string video_path = video_folder + "/cam" + std::to_string(idx_camera) + ".mp4";
        caps[idx_camera].open(video_path);
        if(!caps[idx_camera].isOpened()) {
            std::cout << "Error! cannot open video " << video_path << std::endl;
            return 1;
        }
    }

    // -------------- Read ground truth -----------------
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
    if(!gt_file_path.empty()) {
        ReadGTFile(gt_states, gt_file_path);
    }

    size_t num_frames = caps[0].get(cv::CAP_PROP_FRAME_COUNT);
    double fps = caps[0].get(cv::CAP_PROP_FPS);
    size_t idx_start = 0, idx_end = num_frames - 1;
    if(t_start >= 0 && t_start < num_frames / fps) {
        for(size_t idx_camera = 0; idx_camera < num_cameras; idx_camera++) {
            caps[idx_camera].set(cv::CAP_PROP_POS_MSEC, t_start * 1e3);
        }
        idx_start = caps[0].get(cv::CAP_PROP_POS_FRAMES);
    }

    if(t_end > 0 && t_end < num_frames / fps) {
        idx_end = t_end * fps;
    }

    auto iter_imu_data = imu_data_vec.begin();
    for(size_t idx_frame = idx_start; idx_frame < idx_end; idx_frame++) {
        double t_img = caps[0].get(cv::CAP_PROP_POS_MSEC) * 1e-3;

        std::vector<ov_core::ImuData> measure_imu_vec;
        while (iter_imu_data != imu_data_vec.end() && iter_imu_data->first < t_img) {
            auto imu_data = iter_imu_data->second;
            ov_core::ImuData measure_imu;
            measure_imu.timestamp = iter_imu_data->first;
            measure_imu.am << imu_data[3], imu_data[4], imu_data[5];
            measure_imu.wm << imu_data[0], imu_data[1], imu_data[2];

            measure_imu_vec.push_back(measure_imu);
            iter_imu_data++;
        }

        for(const auto& measure_imu : measure_imu_vec) {
            sys->feed_measurement_imu(measure_imu);
        }

        std::vector<cv::Mat> imgs(num_cameras);
        for(size_t idx_camera = 0; idx_camera < num_cameras; idx_camera++) {
            cv::Mat img;
            caps[idx_camera].read(img);
            if(img.empty()) {
                std::cout << "Error! cannot get image from video " << idx_camera << std::endl;
                return 1;
            }

            if(img.channels() == 3) {
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            }
            imgs[idx_camera] = img;
        }

        Eigen::Matrix<double, 17, 1> imustate;
        if(!gt_states.empty() && !sys->initialized() && get_gt_state(t_img, imustate, gt_states)) {
            //biases are pretty bad normally, so zero them
            //imustate.block(11,0,6,1).setZero();
            sys->initialize_with_gt(imustate);
        } else if(gt_states.empty() || sys->initialized()) {
            for(size_t idx_camera = 0; idx_camera < num_cameras; idx_camera++) {
                ov_core::CameraData measure_img;
                measure_img.timestamp = t_img;
                measure_img.sensor_ids.push_back(idx_camera);
                measure_img.images.push_back(imgs[idx_camera]);
                sys->feed_measurement_camera(measure_img);
            }
        }

        // display tracking result
        cv::Mat tracked_img = sys->get_historical_viz_image();
        if(!tracked_img.empty()) {
            cv::imshow("tracked_img", tracked_img);
        }
        int key = cv::waitKey(5);
        if(key == 'q' || key == 'Q') {
            break;
        }

        if(sys->initialized()) {
            // Get current state
            std::shared_ptr<State> state = sys->get_state();

            // save pose of cam0
            {
                double t_cam = state->_timestamp;

                auto R_iw = state->_imu->Rot();
                auto t_wi = state->_imu->pos();

                auto R_ci = state->_calib_IMUtoCAM.at(0)->Rot();
                auto t_ci = state->_calib_IMUtoCAM.at(0)->pos();

                Eigen::Matrix3d R_ic = R_ci.transpose();
                Eigen::Vector3d t_ic = -R_ci.transpose() * t_ci;

                auto R_wi = R_iw.transpose();
                auto R_wc = R_wi * R_ic;
                auto t_wc = R_wi * t_ic + t_wi;

                Eigen::Quaterniond q_wc(R_wc);

                ofs_pose_cam0 << std::setprecision(18)
                        << t_cam << " "
                        << t_wc[0] << " " << t_wc[1] << " " << t_wc[2] << " "
                        << q_wc.x() << " " << q_wc.y() << " " << q_wc.z() << " " << q_wc.w()
                        << std::endl;
            }

            // save pose of imu
            {
                double timestamp = state->_timestamp;
                auto pos = state->_imu->pos();
                auto ori = state->_imu->quat(); // R_iw

                ofs_pose_imu << std::setprecision(18)
                        << timestamp << " "
                        << pos[0] << " " << pos[1] << " " << pos[2] << " "
                        << ori[0] << " " << ori[1] << " " << ori[2] << " " << ori[3]
                        << std::endl;
            }
        }
    }

    for(size_t i = 0; i < num_cameras; i++) {
        caps[i].release();
    }

    // Done!
    return EXIT_SUCCESS;
}


















