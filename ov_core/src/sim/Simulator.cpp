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
#include "Simulator.h"


using namespace ov_core;




Simulator::Simulator(ros::NodeHandle& nh) {


    //===============================================================
    //===============================================================

    // Nice startup message
    ROS_INFO("=======================================");
    ROS_INFO("VISUAL-INERTIAL SIMULATOR STARTING");
    ROS_INFO("=======================================");

    // Load the groundtruth trajectory and its spline
    std::string path_traj = "./src/open_vins/ov_data/sim/udel_gore.txt";
    nh.param<std::string>("sim_traj_path", path_traj, path_traj);
    load_data(path_traj);
    spline.feed_trajectory(traj_data);

    // Read in sensor simulation frequencies
    nh.param<double>("sim_freq_cam", freq_cam, 10);
    nh.param<double>("sim_freq_imu", freq_imu, 200);

    // Set all our timestamps as starting from the minimum spline time
    timestamp = spline.get_start_time();
    timestamp_last_imu = spline.get_start_time();
    timestamp_last_cam = spline.get_start_time();

    // Get the pose at the current timestep
    Eigen::Matrix3d R_GtoI_init;
    Eigen::Vector3d p_IinG_init;
    bool success_pose_init = spline.get_pose(timestamp, R_GtoI_init, p_IinG_init);
    if(!success_pose_init) {
        ROS_ERROR("[SIM]: unable to find the first pose in the spline...");
        std::exit(EXIT_FAILURE);
    }

    // Find the timestamp that we move enough to be considered "moved"
    double distance = 0.0;
    double distancethreshold = 0.0;
    nh.param<double>("sim_distance_threshold", distancethreshold, 0.5);
    while(true) {

        // Get the pose at the current timestep
        Eigen::Matrix3d R_GtoI;
        Eigen::Vector3d p_IinG;
        bool success_pose = spline.get_pose(timestamp, R_GtoI, p_IinG);

        // Check if it fails
        if(!success_pose) {
            ROS_ERROR("[SIM]: unable to find jolt in the groundtruth data to initialize at");
            std::exit(EXIT_FAILURE);
        }

        // Append to our scalar distance
        distance += (p_IinG-p_IinG_init).norm();
        p_IinG_init = p_IinG;

        // Now check if we have an acceleration, else move forward in time
        if(distance > distancethreshold) {
            break;
        } else {
            timestamp += 1.0/freq_cam;
            timestamp_last_imu += 1.0/freq_cam;
            timestamp_last_cam += 1.0/freq_cam;
        }

    }
    ROS_INFO("[SIM]: moved %.3f seconds into the dataset where it starts moving",timestamp-spline.get_start_time());

    // Our simulation is running
    is_running = true;

    //===============================================================
    //===============================================================

    // Load the seeds for the random number generators
    int seed_state_init, sim_seed_preturb, sim_seed_measurements;
    nh.param<int>("sim_seed_state_init", seed_state_init, 0);
    nh.param<int>("sim_seed_preturb", sim_seed_preturb, 0);
    nh.param<int>("sim_seed_measurements", sim_seed_measurements, 0);
    gen_state_init = std::mt19937(seed_state_init);
    gen_state_init.seed(seed_state_init);
    gen_state_perturb = std::mt19937(sim_seed_preturb);
    gen_state_perturb.seed(sim_seed_preturb);
    gen_meas_imu = std::mt19937(sim_seed_measurements);
    gen_meas_imu.seed(sim_seed_measurements);

    // Load number of cameras and number of points
    nh.param<int>("max_cameras", max_cameras, 1);
    nh.param<bool>("use_stereo", use_stereo, true);
    nh.param<int>("num_pts", num_pts, 200);

    // Create generator for our camera
    for(int i=0; i<max_cameras; i++) {
        gen_meas_cams.push_back(std::mt19937(sim_seed_measurements));
        gen_meas_cams.at(i).seed(sim_seed_measurements);
    }

    // Global gravity
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.81};
    nh.param<std::vector<double>>("gravity", vec_gravity, vec_gravity_default);
    gravity << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    // Timeoffset from camera to IMU
    nh.param<double>("calib_camimu_dt", calib_camimu_dt, 0.0);

    // Debug print
    ROS_INFO("SIMULATION PARAMETERS:");
    ROS_INFO("\t- \033[1;31mbold state init seed: %d \033[0m", seed_state_init);
    ROS_INFO("\t- \033[1;31mbold perturb seed: %d \033[0m", sim_seed_preturb);
    ROS_INFO("\t- \033[1;31mbold measurement seed: %d \033[0m", sim_seed_measurements);
    ROS_INFO("\t- cam feq: %.2f", freq_cam);
    ROS_INFO("\t- imu feq: %.2f", freq_imu);
    ROS_INFO("\t- max cameras: %d", max_cameras);
    ROS_INFO("\t- max features: %d", num_pts);
    ROS_INFO("\t- gravity: %.3f, %.3f, %.3f", vec_gravity.at(0), vec_gravity.at(1), vec_gravity.at(2));
    ROS_INFO("\t- cam+imu timeoff: %.3f", calib_camimu_dt);

    // Append the current true bias to our history
    hist_true_bias_time.push_back(timestamp_last_imu-1.0/freq_imu);
    hist_true_bias_accel.push_back(true_bias_accel);
    hist_true_bias_gyro.push_back(true_bias_gyro);
    hist_true_bias_time.push_back(timestamp_last_imu);
    hist_true_bias_accel.push_back(true_bias_accel);
    hist_true_bias_gyro.push_back(true_bias_gyro);

    // Temp set of variables that have the "true" values of the calibration
    std::vector<std::vector<double>> matrix_k_vec, matrix_d_vec, matrix_TCtoI_vec;

    // Loop through through, and load each of the cameras
    ROS_INFO("CAMERA PARAMETERS:");
    for(int i=0; i<max_cameras; i++) {

        // If our distortions are fisheye or not!
        bool is_fisheye;
        nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);

        // If the desired fov we should simulate
        std::vector<int> matrix_wh;
        std::vector<int> matrix_wd_default = {752,480};
        nh.param<std::vector<int>>("cam"+std::to_string(i)+"_wh", matrix_wh, matrix_wd_default);
        std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));

        // Camera intrinsic properties
        Eigen::Matrix<double,8,1> cam_calib;
        std::vector<double> matrix_k, matrix_d;
        std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
        std::vector<double> matrix_d_default = {0,0,0,0};
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
        cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
        matrix_k_vec.push_back(matrix_k);
        matrix_d_vec.push_back(matrix_d);

        // Our camera extrinsics transform
        Eigen::Matrix4d T_CtoI;
        std::vector<double> matrix_TCtoI;
        std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

        // Read in from ROS, and save into our eigen mat
        nh.param<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TCtoI, matrix_TtoI_default);
        T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);
        matrix_TCtoI_vec.push_back(matrix_TCtoI);

        // Load these into our state
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,is_fisheye});
        camera_intrinsics.insert({i,cam_calib});
        camera_extrinsics.insert({i,cam_eigen});
        camera_wh.insert({i,wh});

        // Debug printing
        cout << "cam_" << i << "wh:" << endl << wh.first << " x " << wh.second << endl;
        cout << "cam_" << i << "K:" << endl << cam_calib.block(0,0,4,1).transpose() << endl;
        cout << "cam_" << i << "d:" << endl << cam_calib.block(4,0,4,1).transpose() << endl;
        cout << "T_C" << i << "toI:" << endl << T_CtoI << endl << endl;

    }

    // Camera and imu noises
    nh.param<double>("gyroscope_noise_density", sigma_w, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", sigma_a, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", sigma_wb, 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", sigma_ab, 3.0000e-03);
    nh.param<double>("up_msckf_sigma_px", sigma_pix, 1);

    // Debug print out
    ROS_INFO("SENSOR NOISE VALUES:");
    ROS_INFO("\t- sigma_w: %.4f", sigma_w);
    ROS_INFO("\t- sigma_a: %.4f", sigma_a);
    ROS_INFO("\t- sigma_wb: %.4f", sigma_wb);
    ROS_INFO("\t- sigma_ab: %.4f", sigma_ab);
    ROS_INFO("\t- sigma_pxmsckf: %.2f", sigma_pix);


    //===============================================================
    //===============================================================


    // Get if we should perturb the initial state estimates for this system
    bool should_perturb;
    nh.param<bool>("sim_do_perturbation", should_perturb, false);

    // One std generator
    std::normal_distribution<double> w(0,1);

    // Perturb all calibration if we should
    if(should_perturb) {

        // cam imu offset
        double temp = calib_camimu_dt+0.01*w(gen_state_perturb);
        nh.setParam("calib_camimu_dt", temp);

        // camera intrinsics and extrinsics
        for(int i=0; i<max_cameras; i++) {

            // Camera intrinsic properties
            std::vector<double> matrix_k = matrix_k_vec.at(i);
            matrix_k.at(0) += 1.0*w(gen_state_perturb); // k1
            matrix_k.at(1) += 1.0*w(gen_state_perturb); // k2
            matrix_k.at(2) += 1.0*w(gen_state_perturb); // p1
            matrix_k.at(3) += 1.0*w(gen_state_perturb); // p2
            std::vector<double> matrix_d = matrix_d_vec.at(i);
            matrix_d.at(0) += 0.005*w(gen_state_perturb); // r1
            matrix_d.at(1) += 0.005*w(gen_state_perturb); // r2
            matrix_d.at(2) += 0.005*w(gen_state_perturb); // r3
            matrix_d.at(3) += 0.005*w(gen_state_perturb); // r4

            // Our camera extrinsics transform
            std::vector<double> matrix_TCtoI = matrix_TCtoI_vec.at(i);
            matrix_TCtoI.at(3) += 0.01*w(gen_state_perturb); // x
            matrix_TCtoI.at(7) += 0.01*w(gen_state_perturb); // y
            matrix_TCtoI.at(11) += 0.01*w(gen_state_perturb); // z

            // Perturb the orientation calibration
            Eigen::Matrix3d R_calib;
            R_calib << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),
                    matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),
                    matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10);
            Eigen::Vector3d w_vec;
            w_vec << 0.001*w(gen_state_perturb), 0.001*w(gen_state_perturb), 0.001*w(gen_state_perturb);
            R_calib = exp_so3(w_vec)*R_calib;

            matrix_TCtoI.at(0) = R_calib(0,0);
            matrix_TCtoI.at(1) = R_calib(0,1);
            matrix_TCtoI.at(2) = R_calib(0,2);
            matrix_TCtoI.at(4) = R_calib(1,0);
            matrix_TCtoI.at(5) = R_calib(1,1);
            matrix_TCtoI.at(6) = R_calib(1,2);
            matrix_TCtoI.at(8) = R_calib(2,0);
            matrix_TCtoI.at(9) = R_calib(2,1);
            matrix_TCtoI.at(10) = R_calib(2,2);

            // Overwrite their values
            nh.setParam("cam"+std::to_string(i)+"_k", matrix_k);
            nh.setParam("cam"+std::to_string(i)+"_d", matrix_d);
            nh.setParam("T_C"+std::to_string(i)+"toI", matrix_TCtoI);

        }

    }

    //===============================================================
    //===============================================================


    // We will create synthetic camera frames and ensure that each has enough features
    //double dt = 0.25/freq_cam;
    double dt = 0.25;
    size_t mapsize = featmap.size();
    ROS_INFO("[SIM]: Generating map features at %d rate",(int)(1.0/dt));

    // Loop through each camera
    // NOTE: we loop through cameras here so that the feature map for camera 1 will always be the same
    // NOTE: thus when we add more cameras the first camera should get the same measurements
    for(int i=0; i<max_cameras; i++) {

        // Reset the start time
        double time_synth = spline.get_start_time();

        // Loop through each pose and generate our feature map in them!!!!
        while(ros::ok()) {
            // Get the pose at the current timestep
            Eigen::Matrix3d R_GtoI;
            Eigen::Vector3d p_IinG;
            bool success_pose = spline.get_pose(time_synth, R_GtoI, p_IinG);
            // We have finished generating features
            if(!success_pose)
                break;

            // Get the uv features for this frame
            std::vector<std::pair<size_t,Eigen::VectorXf>> uvs = project_pointcloud(R_GtoI, p_IinG, i, featmap);
            // If we do not have enough, generate more
            if((int)uvs.size() < num_pts) {
                generate_points(R_GtoI, p_IinG, i, featmap, num_pts-(int)uvs.size());
            }

            // Move forward in time
            time_synth += dt;
        }

        // Debug print
        ROS_INFO("[SIM]: Generated %d map features in total over %d frames (camera %d)",(int)(featmap.size()-mapsize),(int)((time_synth-spline.get_start_time())/dt),i);
        mapsize = featmap.size();

    }

    // Print our map features
    //for(const auto &feat : featmap) {
    //    cout << feat.second(0) << "," << feat.second(1) << "," << feat.second(2) << std::endl;
    //}
    sleep(3);

}





bool Simulator::get_state(double desired_time, Eigen::Matrix<double,17,1> &imustate) {

    // Set to default state
    imustate.setZero();
    imustate(4) = 1;

    // Current state values
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG, w_IinI, v_IinG;

    // Get the pose, velocity, and acceleration
    bool success_vel = spline.get_velocity(desired_time, R_GtoI, p_IinG, w_IinI, v_IinG);

    // Find the bounding bias values
    bool success_bias = false;
    size_t id_loc = 0;
    for(size_t i=0; i<hist_true_bias_time.size()-1; i++) {
        if(hist_true_bias_time.at(i) < desired_time && hist_true_bias_time.at(i+1) >= desired_time) {
            id_loc = i;
            success_bias = true;
            break;
        }
    }

    // If failed, then that means we don't have any more spline or bias
    if(!success_vel || !success_bias) {
        return false;
    }

    // Interpolate our biases (they will be at every IMU timestep)
    double lambda = (desired_time-hist_true_bias_time.at(id_loc))/(hist_true_bias_time.at(id_loc+1)-hist_true_bias_time.at(id_loc));
    Eigen::Vector3d true_bg_interp = (1-lambda)*hist_true_bias_gyro.at(id_loc) + lambda*hist_true_bias_gyro.at(id_loc+1);
    Eigen::Vector3d true_ba_interp = (1-lambda)*hist_true_bias_accel.at(id_loc) + lambda*hist_true_bias_accel.at(id_loc+1);

    // Finally lets create the current state
    imustate(0,0) = desired_time;
    imustate.block(1,0,4,1) = rot_2_quat(R_GtoI);
    imustate.block(5,0,3,1) = p_IinG;
    imustate.block(8,0,3,1) = v_IinG;
    imustate.block(11,0,3,1) = true_bg_interp;
    imustate.block(14,0,3,1) = true_ba_interp;
    return true;

}




bool Simulator::get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am) {

    // Return if the camera measurement should go before us
    if(timestamp_last_cam+1.0/freq_cam < timestamp_last_imu+1.0/freq_imu)
        return false;

    // Else lets do a new measurement!!!
    timestamp_last_imu += 1.0/freq_imu;
    timestamp = timestamp_last_imu;
    time_imu = timestamp_last_imu;

    // Current state values
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

    // Get the pose, velocity, and acceleration
    // NOTE: we get the acceleration between our two IMU
    // NOTE: this is because we are using a constant measurement model for integration
    //bool success_accel = spline.get_acceleration(timestamp+0.5/freq_imu, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
    bool success_accel = spline.get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

    // If failed, then that means we don't have any more spline
    // Thus we should stop the simulation
    if(!success_accel) {
        is_running = false;
        return false;
    }

    // Transform omega and linear acceleration into imu frame
    Eigen::Vector3d omega_inI = w_IinI;
    Eigen::Vector3d accel_inI = R_GtoI*(a_IinG+gravity);

    // Now add noise to these measurements
    double dt = 1.0/freq_imu;
    std::normal_distribution<double> w(0,1);
    wm(0) = omega_inI(0) + true_bias_gyro(0) + sigma_w/std::sqrt(dt)*w(gen_meas_imu);
    wm(1) = omega_inI(1) + true_bias_gyro(1) + sigma_w/std::sqrt(dt)*w(gen_meas_imu);
    wm(2) = omega_inI(2) + true_bias_gyro(2) + sigma_w/std::sqrt(dt)*w(gen_meas_imu);
    am(0) = accel_inI(0) + true_bias_accel(0) + sigma_a/std::sqrt(dt)*w(gen_meas_imu);
    am(1) = accel_inI(1) + true_bias_accel(1) + sigma_a/std::sqrt(dt)*w(gen_meas_imu);
    am(2) = accel_inI(2) + true_bias_accel(2) + sigma_a/std::sqrt(dt)*w(gen_meas_imu);

    // Move the biases forward in time
    true_bias_gyro(0) += sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
    true_bias_gyro(1) += sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
    true_bias_gyro(2) += sigma_wb*std::sqrt(dt)*w(gen_meas_imu);
    true_bias_accel(0) += sigma_ab*std::sqrt(dt)*w(gen_meas_imu);
    true_bias_accel(1) += sigma_ab*std::sqrt(dt)*w(gen_meas_imu);
    true_bias_accel(2) += sigma_ab*std::sqrt(dt)*w(gen_meas_imu);

    // Append the current true bias to our history
    hist_true_bias_time.push_back(timestamp_last_imu);
    hist_true_bias_gyro.push_back(true_bias_gyro);
    hist_true_bias_accel.push_back(true_bias_accel);

    // Return success
    return true;

}



bool Simulator::get_next_cam(double &time_cam, std::vector<int> &camids, std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {

    // Return if the imu measurement should go before us
    if(timestamp_last_imu+1.0/freq_imu < timestamp_last_cam+1.0/freq_cam)
        return false;

    // Else lets do a new measurement!!!
    timestamp_last_cam += 1.0/freq_cam;
    timestamp = timestamp_last_cam;
    time_cam = timestamp_last_cam-calib_camimu_dt;

    // Get the pose at the current timestep
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG;
    bool success_pose = spline.get_pose(timestamp, R_GtoI, p_IinG);

    // We have finished generating measurements
    if(!success_pose) {
        is_running = false;
        return false;
    }

    // Loop through each camera
    for(int i=0; i<max_cameras; i++) {

        // Get the uv features for this frame
        std::vector<std::pair<size_t,Eigen::VectorXf>> uvs = project_pointcloud(R_GtoI, p_IinG, i, featmap);

        // If we do not have enough, generate more
        if((int)uvs.size() < num_pts) {
            ROS_WARN("[SIM]: cam %d was unable to generate enough features (%d < %d projections)",(int)i,(int)uvs.size(),num_pts);
        }

        // If greater than only select the first set
        if((int)uvs.size() > num_pts) {
            uvs.erase(uvs.begin()+num_pts, uvs.end());
        }

        // Append the map size so all cameras have unique features in them (but the same map)
        // Only do this if we are not enforcing stereo constraints between all our cameras
        for (size_t f=0; f<uvs.size() && !use_stereo; f++){
            uvs.at(f).first += i*featmap.size();
        }

        // Loop through and add noise to each uv measurement
        std::normal_distribution<double> w(0,1);
        for(size_t j=0; j<uvs.size(); j++) {
            uvs.at(j).second(0) += sigma_pix*w(gen_meas_cams.at(i));
            uvs.at(j).second(1) += sigma_pix*w(gen_meas_cams.at(i));
        }

        // Push back for this camera
        feats.push_back(uvs);
        camids.push_back(i);

    }


    // Return success
    return true;

}




void Simulator::load_data(std::string path_traj) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
        ROS_ERROR("ERROR: Unable to open simulation trajectory file...");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Debug print
    std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
    ROS_INFO("[SIM]: loaded trajectory %s",base_filename.c_str());

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line) && ros::ok()) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        int i = 0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<double,8,1> data;

        // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
        while(std::getline(s,field,' ') && ros::ok()) {
            // Skip if empty
            if(field.empty() || i >= data.rows())
                continue;
            // save the data to our vector
            data(i) = std::atof(field.c_str());
            i++;
        }

        // Only a valid line if we have all the parameters
        if(i > 7) {
            traj_data.push_back(data);
            //std::cout << std::setprecision(15) << data.transpose() << std::endl;
        }

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (traj_data.empty()) {
        ROS_ERROR("ERROR: Could not parse any data from the file!!");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

}







std::vector<std::pair<size_t,Eigen::VectorXf>> Simulator::project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                                                             int camid, const std::unordered_map<size_t,Eigen::Vector3d> &feats) {

    // Assert we have good camera
    assert(camid < max_cameras);
    assert((int)camera_fisheye.size() == max_cameras);
    assert((int)camera_wh.size() == max_cameras);
    assert((int)camera_intrinsics.size() == max_cameras);
    assert((int)camera_extrinsics.size() == max_cameras);

    // Grab our extrinsic and intrinsic values
    Eigen::Matrix<double,3,3> R_ItoC = quat_2_Rot(camera_extrinsics.at(camid).block(0,0,4,1));
    Eigen::Matrix<double,3,1> p_IinC = camera_extrinsics.at(camid).block(4,0,3,1);
    Eigen::Matrix<double,8,1> cam_d = camera_intrinsics.at(camid);

    // Our projected uv true measurements
    std::vector<std::pair<size_t,Eigen::VectorXf>> uvs;

    // Loop through our map
    for(const auto &feat : feats) {

        // Transform feature into current camera frame
        Eigen::Vector3d p_FinI = R_GtoI*(feat.second-p_IinG);
        Eigen::Vector3d p_FinC = R_ItoC*p_FinI+p_IinC;

        // Skip cloud if too far away
        if(p_FinC(2) > 15 || p_FinC(2) < 0.5)
            continue;

        // Project to normalized coordinates
        Eigen::Vector2f uv_norm;
        uv_norm << p_FinC(0)/p_FinC(2),p_FinC(1)/p_FinC(2);

        // Distort the normalized coordinates (false=radtan, true=fisheye)
        Eigen::Vector2f uv_dist;

        // Calculate distortion uv and jacobian
        if(camera_fisheye.at(camid)) {

            // Calculate distorted coordinates for fisheye
            double r = sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
            double theta = std::atan(r);
            double theta_d = theta+cam_d(4)*std::pow(theta,3)+cam_d(5)*std::pow(theta,5)+cam_d(6)*std::pow(theta,7)+cam_d(7)*std::pow(theta,9);

            // Handle when r is small (meaning our xy is near the camera center)
            double inv_r = r > 1e-8 ? 1.0/r : 1;
            double cdist = r > 1e-8 ? theta_d * inv_r : 1;

            // Calculate distorted coordinates for fisheye
            double x1 = uv_norm(0)*cdist;
            double y1 = uv_norm(1)*cdist;
            uv_dist(0) = cam_d(0)*x1 + cam_d(2);
            uv_dist(1) = cam_d(1)*y1 + cam_d(3);

        } else {

            // Calculate distorted coordinates for radial
            double r = std::sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
            double r_2 = r*r;
            double r_4 = r_2*r_2;
            double x1 = uv_norm(0)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+2*cam_d(6)*uv_norm(0)*uv_norm(1)+cam_d(7)*(r_2+2*uv_norm(0)*uv_norm(0));
            double y1 = uv_norm(1)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+cam_d(6)*(r_2+2*uv_norm(1)*uv_norm(1))+2*cam_d(7)*uv_norm(0)*uv_norm(1);
            uv_dist(0) = cam_d(0)*x1 + cam_d(2);
            uv_dist(1) = cam_d(1)*y1 + cam_d(3);

        }

        // Check that it is inside our bounds
        if(uv_dist(0) < 0 || uv_dist(0) > camera_wh.at(camid).first || uv_dist(1) < 0 || uv_dist(1) > camera_wh.at(camid).second) {
            continue;
        }

        // Else we can add this as a good projection
        uvs.push_back({feat.first, uv_dist});

    }

    // Return our projections
    return uvs;

}




void Simulator::generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                int camid, std::unordered_map<size_t,Eigen::Vector3d> &feats, int numpts) {

    // Assert we have good camera
    assert(camid < max_cameras);
    assert((int)camera_fisheye.size() == max_cameras);
    assert((int)camera_wh.size() == max_cameras);
    assert((int)camera_intrinsics.size() == max_cameras);
    assert((int)camera_extrinsics.size() == max_cameras);

    // Grab our extrinsic and intrinsic values
    Eigen::Matrix<double,3,3> R_ItoC = quat_2_Rot(camera_extrinsics.at(camid).block(0,0,4,1));
    Eigen::Matrix<double,3,1> p_IinC = camera_extrinsics.at(camid).block(4,0,3,1);
    Eigen::Matrix<double,8,1> cam_d = camera_intrinsics.at(camid);

    // Convert to opencv format since we will use their undistort functions
    cv::Matx33d camK;
    camK(0, 0) = cam_d(0);
    camK(0, 1) = 0;
    camK(0, 2) = cam_d(2);
    camK(1, 0) = 0;
    camK(1, 1) = cam_d(1);
    camK(1, 2) = cam_d(3);
    camK(2, 0) = 0;
    camK(2, 1) = 0;
    camK(2, 2) = 1;
    cv::Vec4d camD;
    camD(0) = cam_d(4);
    camD(1) = cam_d(5);
    camD(2) = cam_d(6);
    camD(3) = cam_d(7);


    // Generate the desired number of features
    for(int i=0; i<numpts; i++) {

        // Uniformly randomly generate within our fov
        std::uniform_real_distribution<double> gen_u(0,camera_wh.at(camid).first);
        std::uniform_real_distribution<double> gen_v(0,camera_wh.at(camid).second);
        double u_dist = gen_u(gen_state_init);
        double v_dist = gen_v(gen_state_init);

        // Convert to opencv format
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = u_dist;
        mat.at<float>(0, 1) = v_dist;
        mat = mat.reshape(2); // Nx1, 2-channel

        // Undistort this point to our normalized coordinates (false=radtan, true=fisheye)
        if(camera_fisheye.at(camid)) {
            cv::fisheye::undistortPoints(mat, mat, camK, camD);
        } else {
            cv::undistortPoints(mat, mat, camK, camD);
        }

        // Construct our return vector
        Eigen::Vector3d uv_norm;
        mat = mat.reshape(1); // Nx2, 1-channel
        uv_norm(0) = mat.at<float>(0, 0);
        uv_norm(1) = mat.at<float>(0, 1);
        uv_norm(2) = 1;

        // Generate a random depth
        std::uniform_real_distribution<double> gen_depth(5,10);
        double depth = gen_depth(gen_state_init);

        // Get the 3d point
        Eigen::Vector3d p_FinC;
        p_FinC = depth*uv_norm;

        // Move to the global frame of reference
        Eigen::Vector3d p_FinI = R_ItoC.transpose()*(p_FinC-p_IinC);
        Eigen::Vector3d p_FinG = R_GtoI.transpose()*p_FinI+p_IinG;

        // Append this as a new feature
        featmap.insert({id_map,p_FinG});
        id_map++;

    }


}
