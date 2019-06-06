#include "VioManager.h"



using namespace ov_core;
using namespace ov_msckf;





VioManager::VioManager(ros::NodeHandle &nh) {


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load our state options
    StateOptions state_options;
    nh.param<bool>("use_fej", state_options.do_fej, false);
    nh.param<bool>("use_imuavg", state_options.imu_avg, false);
    nh.param<bool>("calib_cam_extrinsics", state_options.do_calib_camera_pose, false);
    nh.param<bool>("calib_cam_intrinsics", state_options.do_calib_camera_intrinsics, false);
    nh.param<bool>("calib_camimu_dt", state_options.do_calib_camera_timeoffset, false);
    nh.param<int>("max_clones", state_options.max_clone_size, 10);
    nh.param<int>("max_slam", state_options.max_slam_features, 0);
    nh.param<int>("max_cameras", state_options.num_cameras, 1);

    // Enforce that if we are doing stereo tracking, we have two cameras
    if(state_options.num_cameras < 1) {
        ROS_ERROR("VioManager(): Specified number of cameras needs to be greater than zero");
        ROS_ERROR("VioManager(): num cameras = %d", state_options.num_cameras);
        std::exit(EXIT_FAILURE);
    }

    // Read in what representation our feature is
    std::string feat_rep_str;
    nh.param<std::string>("feat_representation", feat_rep_str, "GLOBAL_3D");
    std::transform(feat_rep_str.begin(), feat_rep_str.end(),feat_rep_str.begin(), ::toupper);

    // Set what representation we should be using
    if(feat_rep_str == "GLOBAL_3D") state_options.feat_representation = StateOptions::FeatureRepresentation::GLOBAL_3D;
    else if(feat_rep_str == "GLOBAL_FULL_INVERSE_DEPTH") state_options.feat_representation = StateOptions::FeatureRepresentation::GLOBAL_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_3D") state_options.feat_representation = StateOptions::FeatureRepresentation::ANCHORED_3D;
    else if(feat_rep_str == "ANCHORED_FULL_INVERSE_DEPTH") state_options.feat_representation = StateOptions::FeatureRepresentation::ANCHORED_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_MSCKF_INVERSE_DEPTH") state_options.feat_representation = StateOptions::FeatureRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH;
    else {
        ROS_ERROR("VioManager(): invalid feature representation specified = %s", feat_rep_str.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Create the state!!
    state = new State(state_options);

    // Global gravity
    Eigen::Matrix<double,3,1> gravity;
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.81};
    nh.param<std::vector<double>>("gravity", vec_gravity, vec_gravity_default);
    gravity << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    // Debug, print to the console!
    ROS_INFO("FILTER PARAMETERS:");
    ROS_INFO("\t- do fej: %d", state_options.do_fej);
    ROS_INFO("\t- do imu avg: %d", state_options.imu_avg);
    ROS_INFO("\t- calibrate cam to imu: %d", state_options.do_calib_camera_pose);
    ROS_INFO("\t- calibrate cam intrinsics: %d", state_options.do_calib_camera_intrinsics);
    ROS_INFO("\t- calibrate cam imu timeoff: %d", state_options.do_calib_camera_timeoffset);
    ROS_INFO("\t- max clones: %d", state_options.max_clone_size);
    ROS_INFO("\t- max slam: %d", state_options.max_slam_features);
    ROS_INFO("\t- max cameras: %d", state_options.num_cameras);
    ROS_INFO("\t- feature representation: %s", feat_rep_str.c_str());
    ROS_INFO("\t- gravity: %.3f, %.3f, %.3f", vec_gravity.at(0), vec_gravity.at(1), vec_gravity.at(2));


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Debug message
    ROS_INFO("=====================================");
    ROS_INFO("CAMERA PARAMETERS:");

    // Camera intrincs that we will load in
    std::unordered_map<size_t,bool> camera_fisheye;
    std::unordered_map<size_t,Eigen::Matrix3d> camera_k;
    std::unordered_map<size_t,Eigen::Matrix<double,4,1>> camera_d;

    // Loop through through, and load each of the cameras
    for(int i=0; i<state->options().num_cameras; i++) {

        // If our distortions are fisheye or not!
        bool is_fisheye;
        nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);
        state->get_model_CAM(i) = is_fisheye;

        // Camera intrinsic properties
        Eigen::Matrix3d cam_k;
        Eigen::Matrix<double,4,1> cam_d;
        std::vector<double> matrix_k, matrix_d;
        std::vector<double> matrix_k_default = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        std::vector<double> matrix_d_default = {0.0,0.0,0.0,0.0};
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
        cam_k << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_k.at(4),matrix_k.at(5),matrix_k.at(6),matrix_k.at(7),matrix_k.at(8);
        cam_d << matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

        // Stack the focal lengths, camera center, and distortion into our state representation
        Eigen::Matrix<double,8,1> intrinsics;
        intrinsics << cam_k(0,0), cam_k(1,1), cam_k(0,2), cam_k(1,2), cam_d;
        state->get_intrinsics_CAM(i)->set_value(intrinsics);

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
        state->get_calib_IMUtoCAM(i)->set_value(cam_eigen);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,is_fisheye});
        camera_k.insert({i,cam_k});
        camera_d.insert({i,cam_d});

        // Debug printing
        cout << "cam_" << i << "K:" << endl << cam_k << endl;
        cout << "cam_" << i << "d:" << endl << cam_d.transpose() << endl;
        cout << "T_C" << i << "toI:" << endl << T_CtoI << endl << endl;

    }

    // Debug message
    ROS_INFO("=====================================");

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load config values for our feature initializer
    FeatureInitializerOptions featinit_options;
    nh.param<int>("fi_max_runs", featinit_options.max_runs, 20);
    nh.param<double>("fi_init_lamda", featinit_options.init_lamda, 1e-3);
    nh.param<double>("fi_max_lamda", featinit_options.max_lamda, 1e10);
    nh.param<double>("fi_min_dx", featinit_options.min_dx, 1e-6);
    nh.param<double>("fi_min_dcost", featinit_options.min_dcost, 1e-6);
    nh.param<double>("fi_lam_mult", featinit_options.lam_mult, 10);
    nh.param<double>("fi_min_dist", featinit_options.min_dist, 0.25);
    nh.param<double>("fi_max_dist", featinit_options.max_dist, 40);
    nh.param<double>("fi_max_baseline", featinit_options.max_baseline, 40);
    nh.param<double>("fi_max_cond_number", featinit_options.max_cond_number, 1000);

    // Debug, print to the console!
    ROS_INFO("FEATURE INITIALIZER PARAMETERS:");
    ROS_INFO("\t- max runs: %d", featinit_options.max_runs);
    ROS_INFO("\t- init lambda: %e", featinit_options.init_lamda);
    ROS_INFO("\t- max lambda: %.4f", featinit_options.max_lamda);
    ROS_INFO("\t- min final dx: %.4f", featinit_options.min_dx);
    ROS_INFO("\t- min delta cost: %.4f", featinit_options.min_dcost);
    ROS_INFO("\t- lambda multiple: %.4f", featinit_options.lam_mult);
    ROS_INFO("\t- closest feature dist: %.4f", featinit_options.min_dist);
    ROS_INFO("\t- furthest feature dist: %.4f", featinit_options.max_dist);
    ROS_INFO("\t- max baseline ratio: %.4f", featinit_options.max_baseline);
    ROS_INFO("\t- max condition number: %.4f", featinit_options.max_cond_number);

    // Parameters for our extractor
    int num_pts, num_aruco, fast_threshold, grid_x, grid_y, min_px_dist, sfm_window, sfm_min_feat;
    double knn_ratio;
    bool use_klt, use_aruco, do_downsizing;
    nh.param<bool>("use_klt", use_klt, true);
    nh.param<bool>("use_aruco", use_aruco, true);
    nh.param<int>("num_pts", num_pts, 500);
    nh.param<int>("num_aruco", num_aruco, 1024);
    nh.param<int>("fast_threshold", fast_threshold, 10);
    nh.param<int>("grid_x", grid_x, 10);
    nh.param<int>("grid_y", grid_y, 8);
    nh.param<int>("min_px_dist", min_px_dist, 10);
    nh.param<double>("knn_ratio", knn_ratio, 0.85);
    nh.param<bool>("downsize_aruco", do_downsizing, true);
    nh.param<int>("init_sfm_window", sfm_window, 10);
    nh.param<int>("init_sfm_min_feat", sfm_min_feat, 15);

    // Debug, print to the console!
    ROS_INFO("TRACKING PARAMETERS:");
    ROS_INFO("\t- use klt: %d", use_klt);
    ROS_INFO("\t- use aruco: %d", use_aruco);
    ROS_INFO("\t- max track features: %d", num_pts);
    ROS_INFO("\t- max aruco tags: %d", num_aruco);
    ROS_INFO("\t- grid size: %d x %d", grid_x, grid_y);
    ROS_INFO("\t- fast threshold: %d", fast_threshold);
    ROS_INFO("\t- min pixel distance: %d", min_px_dist);
    ROS_INFO("\t- downsize aruco image: %d", do_downsizing);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our noise values for inertial sensor
    Propagator::NoiseManager imu_noises;
    nh.param<double>("gyroscope_noise_density", imu_noises.sigma_w, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", imu_noises.sigma_a, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", imu_noises.sigma_wb, 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", imu_noises.sigma_ab, 3.0000e-03);


    // Debug print out
    ROS_INFO("PROPAGATOR NOISES:");
    ROS_INFO("\t- sigma_w: %.4f", imu_noises.sigma_w);
    ROS_INFO("\t- sigma_a: %.4f", imu_noises.sigma_a);
    ROS_INFO("\t- sigma_wb: %.4f", imu_noises.sigma_wb);
    ROS_INFO("\t- sigma_ab: %.4f", imu_noises.sigma_ab);

    // Load inertial state initialize parameters
    double init_window_time, init_imu_thresh;
    nh.param<double>("init_window_time", init_window_time, 0.5);
    nh.param<double>("init_imu_thresh", init_imu_thresh, 1.0);

    // Debug print out
    ROS_INFO("INITIALIZATION PARAMETERS:");
    ROS_INFO("\t- init_window_time: %.4f", init_window_time);
    ROS_INFO("\t- init_imu_thresh: %.4f", init_imu_thresh);


    // Read in update parameters
    UpdaterOptions msckf_options;
    nh.param<double>("up_sigma_pxmsckf", msckf_options.sigma_pix, 1);
    nh.param<int>("up_chi2_multipler", msckf_options.chi2_multipler, 5);
    //nh.param<double>("up_sigma_pxslam", sigma_pxslam, 1);
    //nh.param<double>("up_sigma_pxaruco", sigma_pxaruco, 1);

    // If downsampling aruco, then double our noise values
    //sigma_norm_pxaruco = (do_downsizing) ? 2*sigma_norm_pxaruco : sigma_norm_pxaruco;

    ROS_INFO("MSCKFUPDATER PARAMETERS:");
    ROS_INFO("\t- sigma_pxmsckf: %.4f", msckf_options.sigma_pix);
    ROS_INFO("\t- sigma_pxslam: %.4f", msckf_options.sigma_pix);
    ROS_INFO("\t- sigma_pxaruco: %.4f", msckf_options.sigma_pix);
    ROS_INFO("\t- chi2_multipler: %d", msckf_options.chi2_multipler);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Lets make a feature extractor
    if(use_klt) {
        trackFEATS = new TrackKLT(camera_k,camera_d,camera_fisheye,num_pts,num_aruco,fast_threshold,grid_x,grid_y,min_px_dist);
    } else {
        trackFEATS = new TrackDescriptor(camera_k,camera_d,camera_fisheye,num_pts,num_aruco,fast_threshold,grid_x,grid_y,knn_ratio);
    }

    // Initialize our aruco tag extractor
    if(use_aruco) {
        trackARUCO = new TrackAruco(camera_k,camera_d,camera_fisheye,num_aruco,do_downsizing);
    }

    // Initialize our state propagator
    propagator = new Propagator(imu_noises,gravity);

    // Our state initialize
    initializer = new InertialInitializer(gravity,init_window_time, init_imu_thresh);

    // Make the updater!
    updaterMSCKF = new UpdaterMSCKF(msckf_options, featinit_options);


}




void VioManager::feed_measurement_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

    // Push back to our propagator
    propagator->feed_imu(timestamp,wm,am);

    // Push back to our initializer
    if(!is_initialized_vio) {
        initializer->feed_imu(timestamp, wm, am);
    }

}





void VioManager::feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id) {

    // Feed our trackers
    trackFEATS->feed_monocular(timestamp, img0, cam_id);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_monocular(timestamp, img0, cam_id);
    }

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        bool success = try_to_initialize();
        if(!success) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}





void VioManager::feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1) {

    // Feed our trackers
    trackFEATS->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    }

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        bool success = try_to_initialize();
        if(!success) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);

}



bool VioManager::try_to_initialize() {


        // Returns from our initializer
        double time0;
        Eigen::Matrix<double, 4, 1> q_GtoI0;
        Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

        // Try to initialize the system
        is_initialized_vio = initializer->initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

        // Return if it failed
        if (!is_initialized_vio) {
            return false;
        }

        // Make big vector (q,p,v,bg,ba), and update our state
        // Note: start from zero position, as this is what our covariance is based off of
        Eigen::Matrix<double,16,1> imu_val;
        imu_val.block(0,0,4,1) = q_GtoI0;
        imu_val.block(4,0,3,1) << 0,0,0;
        imu_val.block(7,0,3,1) = v_I0inG;
        imu_val.block(10,0,3,1) = b_w0;
        imu_val.block(13,0,3,1) = b_a0;
        //imu_val.block(10,0,3,1) << 0,0,0;
        //imu_val.block(13,0,3,1) << 0,0,0;
        state->imu()->set_value(imu_val);
        state->set_timestamp(time0);

        // Else we are good to go, print out our stats
        ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
        ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
        ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
        ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
        ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
        return true;

}



void VioManager::do_feature_propagate_update(double timestamp) {


    //===================================================================================
    // State propagation, and clone augmentation
    //===================================================================================

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    propagator->propagate_and_clone(state, timestamp);

    // If we have not reached max clones, we should just return...
    // This isn't super ideal, but it keeps the logic after this easier...
    if((int)state->n_clones() < state->options().max_clone_size) {
        ROS_INFO("waiting for enough clone states (%d of %d) ....",(int)state->n_clones(),state->options().max_clone_size);
        return;
    }


    //===================================================================================
    // MSCKF features and KLT tracks that are SLAM features
    //===================================================================================


    // Now, lets get all features that should be used for an update
    std::vector<Feature*> feats_lost, feats_marg, feats_slam;
    feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->timestamp());
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());

    // We also need to make sure that the max tracks does not contain any lost features
    // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
    auto it1 = feats_lost.begin();
    while(it1 != feats_lost.end()) {
        if(std::find(feats_marg.begin(),feats_marg.end(),(*it1)) != feats_marg.end()) {
            //ROS_WARN("FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!");
            it1 = feats_lost.erase(it1);
        } else {
            it1++;
        }
    }


    // TODO: do some SLAM feature logic here!!!!



    // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
    std::vector<Feature*> featsup_MSCKF = feats_lost;
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());


    //===================================================================================
    // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
    //===================================================================================


    // Pass them to our MSCKF updater
    updaterMSCKF->update(state, featsup_MSCKF);


    //===================================================================================
    // Update our visualization feature set, and clean up the old features
    //===================================================================================

    // Remove features that where used for the update from our extractors at the last timestep
    // This allows for measurements to be used in the future if they failed to be used this time
    // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
    trackFEATS->get_feature_database()->cleanup();



    //===================================================================================
    // Cleanup, marginalize out what we don't need any more...
    //===================================================================================


    // Marginalize the oldest clone of the state if we are at max length
    StateHelper::marginalize_old_clone(state);


}























