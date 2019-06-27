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
    std::string path_traj = "/home/patrick/workspace/catkin_ws_ov/src/rpg_trajectory_evaluation/results/laptop/vio_mono/laptop_vio_mono_V2_01/stamped_groundtruth.txt";
    nh.param<std::string>("sim_traj_path", path_traj, path_traj);
    load_data(path_traj);
    spline.feed_trajectory(traj_data);

    // Set all our timestamps as starting from the minimum spline time
    timestamp = spline.get_start_time();
    timestamp_last_imu = spline.get_start_time();
    timestamp_last_cam = spline.get_start_time();

    // Our simulation is running
    is_running = true;

    //===============================================================
    //===============================================================

    // Load the seeds for the random number generators
    int seed_state_init, sim_seed_measurements;
    nh.param<int>("sim_seed_state_init", seed_state_init, 0);
    nh.param<int>("sim_seed_measurements", sim_seed_measurements, 0);
    gen_state_init = std::mt19937(seed_state_init);
    gen_meas = std::mt19937(sim_seed_measurements);

    // Read in sensor simulation frequencies
    nh.param<int>("sim_freq_cam", freq_cam, 1);
    nh.param<int>("sim_freq_imu", freq_imu, 20);

    // Load number of cameras and number of points
    nh.param<int>("max_cameras", max_cameras, 1);
    nh.param<int>("num_pts", num_pts, 1);

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
    ROS_INFO("\t- \033[1;31mbold measurement seed: %d \033[0m", sim_seed_measurements);
    ROS_INFO("\t- cam feq: %d", freq_cam);
    ROS_INFO("\t- imu feq: %d", freq_imu);
    ROS_INFO("\t- max cameras: %d", max_cameras);
    ROS_INFO("\t- max features: %d", num_pts);
    ROS_INFO("\t- gravity: %.3f, %.3f, %.3f", vec_gravity.at(0), vec_gravity.at(1), vec_gravity.at(2));
    ROS_INFO("\t- cam+imu timeoff: %.3f", calib_camimu_dt);


    // Loop through through, and load each of the cameras
    ROS_INFO("CAMERA PARAMETERS:");
    for(int i=0; i<max_cameras; i++) {

        // If our distortions are fisheye or not!
        bool is_fisheye;
        nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);

        // If the desired fov we should simulate
        double fov;
        nh.param<double>("cam"+std::to_string(i)+"_fov", fov, 45.0);

        // Camera intrinsic properties
        Eigen::Matrix<double,8,1> cam_calib;
        std::vector<double> matrix_k, matrix_d;
        std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
        std::vector<double> matrix_d_default = {0,0,0,0};
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
        cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

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

        // Load these into our state
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,is_fisheye});
        camera_fov.insert({i,fov});
        camera_intrinsics.insert({i,cam_calib});
        camera_extrinsics.insert({i,cam_eigen});

        // Debug printing
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


    // TODO: perturb the initial state estimates for this system


    // TODO: generate the map of points for projection


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
    Eigen::Vector3d p_IinG, w_IinG, v_IinG, alpha_IinG, a_IinG;

    // Get the pose, velocity, and acceleration
    bool success_pose = spline.get_pose(timestamp, R_GtoI, p_IinG);
    bool success_vel = spline.get_velocity(timestamp, w_IinG, v_IinG);
    bool success_accel = spline.get_acceleration(timestamp, alpha_IinG, a_IinG);

    // If any of these failed, then that means we don't have any more spline
    // Thus we should stop the simulation
    if(!success_pose || !success_vel || !success_accel) {
        is_running = false;
        return false;
    }

    // Transform omega and linear acceleration into imu frame
    Eigen::Vector3d omega_inI = R_GtoI.transpose()*w_IinG;
    Eigen::Vector3d accel_inI = R_GtoI.transpose()*(a_IinG+gravity);

    // Now add noise to these measurements
    double dt = 1.0/freq_imu;
    std::normal_distribution<double> w(0,1);
    wm = omega_inI + true_bias_gyro + sigma_w*1/std::sqrt(dt)*w(gen_meas)*Eigen::Vector3d::Ones();
    am = accel_inI + true_bias_accel + sigma_a*1/std::sqrt(dt)*w(gen_meas)*Eigen::Vector3d::Ones();

    // Move the biases forward in time
    true_bias_gyro += sigma_wb*std::sqrt(dt)*w(gen_meas)*Eigen::Vector3d::Ones();
    true_bias_accel += sigma_ab*std::sqrt(dt)*w(gen_meas)*Eigen::Vector3d::Ones();

    // Return success
    return true;

}



bool Simulator::get_next_cam(double &time_cam, std::vector<int> &camids,
                             std::vector<size_t> &featids,
                             std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> &featuvs) {

    // Return if the imu measurement should go before us
    if(timestamp_last_imu+1.0/freq_imu < timestamp_last_cam+1.0/freq_cam)
        return false;

    // Else lets do a new measurement!!!
    timestamp_last_cam += 1.0/freq_cam;
    timestamp = timestamp_last_cam;
    time_cam = timestamp_last_cam;

    // TODO: do these feature measurement logic




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







