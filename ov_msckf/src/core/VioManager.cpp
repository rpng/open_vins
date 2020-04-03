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
#include "VioManager.h"
#include "types/Landmark.h"



using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;





VioManager::VioManager(VioManagerOptions& params_) {


    // Nice startup message
    printf("=======================================\n");
    printf("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
    printf("=======================================\n");

    // Nice debug
    this->params = params_;
    params.print_estimator();
    params.print_noise();
    params.print_state();
    params.print_trackers();

    // Create the state!!
    state = new State(params.state_options);

    // Timeoffset from camera to IMU
    Eigen::VectorXd temp_camimu_dt;
    temp_camimu_dt.resize(1);
    temp_camimu_dt(0) = params.calib_camimu_dt;
    state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
    state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

    // Loop through through, and load each of the cameras
    for(int i=0; i<state->_options.num_cameras; i++) {

        // If our distortions are fisheye or not!
        state->_cam_intrinsics_model.at(i) = params.camera_fisheye.at(i);

        // Camera intrinsic properties
        state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i));
        state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i));

        // Our camera extrinsic transform
        state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
        state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));

    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // If we are recording statistics, then open our file
    if(params.record_timing_information) {
        // If the file exists, then delete it
        if (boost::filesystem::exists(params.record_timing_filepath)) {
            boost::filesystem::remove(params.record_timing_filepath);
            printf(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
        }
        // Create the directory that we will open the file in
        boost::filesystem::path p(params.record_timing_filepath);
        boost::filesystem::create_directories(p.parent_path());
        // Open our statistics file!
        of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
        // Write the header information into it
        of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
        if(state->_options.max_slam_features > 0) {
            of_statistics << "slam update,slam delayed,";
        }
        of_statistics << "marginalization,total" << std::endl;
    }


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Lets make a feature extractor
    if(params.use_klt) {
        trackFEATS = new TrackKLT(params.num_pts,state->_options.max_aruco_features,params.fast_threshold,params.grid_x,params.grid_y,params.min_px_dist);
        trackFEATS->set_calibration(params.camera_intrinsics, params.camera_fisheye);
    } else {
        trackFEATS = new TrackDescriptor(params.num_pts,state->_options.max_aruco_features,params.fast_threshold,params.grid_x,params.grid_y,params.knn_ratio);
        trackFEATS->set_calibration(params.camera_intrinsics, params.camera_fisheye);
    }

    // Initialize our aruco tag extractor
    if(params.use_aruco) {
        trackARUCO = new TrackAruco(state->_options.max_aruco_features, params.downsize_aruco);
        trackARUCO->set_calibration(params.camera_intrinsics, params.camera_fisheye);
    }

    // Initialize our state propagator
    propagator = new Propagator(params.imu_noises, params.gravity);

    // Our state initialize
    initializer = new InertialInitializer(params.gravity,params.init_window_time,params.init_imu_thresh);

    // Make the updater!
    updaterMSCKF = new UpdaterMSCKF(params.msckf_options,params.featinit_options);
    updaterSLAM = new UpdaterSLAM(params.slam_options,params.aruco_options,params.featinit_options);

}




void VioManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {

    // Push back to our propagator
    propagator->feed_imu(timestamp,wm,am);

    // Push back to our initializer
    if(!is_initialized_vio) {
        initializer->feed_imu(timestamp, wm, am);
    }

}





void VioManager::feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Feed our trackers
    trackFEATS->feed_monocular(timestamp, img0, cam_id);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_monocular(timestamp, img0, cam_id);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}


void VioManager::feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Assert we have good ids
    assert(cam_id0!=cam_id1);

    // Feed our stereo trackers, if we are not doing binocular
    if(params.use_stereo) {
        trackFEATS->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    } else {
        boost::thread t_l = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img0), boost::ref(cam_id0));
        boost::thread t_r = boost::thread(&TrackBase::feed_monocular, trackFEATS, boost::ref(timestamp), boost::ref(img1), boost::ref(cam_id1));
        t_l.join();
        t_r.join();
    }

    // If aruoc is avalible, the also pass to it
    // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
    // NOTE: thus we just call the stereo tracking if we are doing binocular!
    if(trackARUCO != nullptr) {
        trackARUCO->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);

}



void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Check if we actually have a simulated tracker
    TrackSIM *trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    if(trackSIM == nullptr) {
        //delete trackFEATS; //(fix this error in the future)
        trackFEATS = new TrackSIM(state->_options.max_aruco_features);
        trackFEATS->set_calibration(params.camera_intrinsics, params.camera_fisheye);
        printf(RED "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
    }

    // Cast the tracker to our simulation tracker
    trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    trackSIM->set_width_height(params.camera_wh);

    // Feed our simulation tracker
    trackSIM->feed_measurement_simulation(timestamp, camids, feats);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then return an error
    if(!is_initialized_vio) {
        printf(RED "[SIM]: your vio system should already be initialized before simulating features!!!\n" RESET);
        printf(RED "[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!\n" RESET);
        std::exit(EXIT_FAILURE);
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
    bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

    // Return if it failed
    if (!success) {
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
    state->_imu->set_value(imu_val);
    state->_timestamp = time0;
    startup_time = time0;

    // Cleanup any features older then the initialization time
    trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
    if(trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
    }

    // Else we are good to go, print out our stats
    printf(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET,state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3));
    printf(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_g()(0),state->_imu->bias_g()(1),state->_imu->bias_g()(2));
    printf(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET,state->_imu->vel()(0),state->_imu->vel()(1),state->_imu->vel()(2));
    printf(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_a()(0),state->_imu->bias_a()(1),state->_imu->bias_a()(2));
    printf(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET,state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2));
    return true;

}



void VioManager::do_feature_propagate_update(double timestamp) {


    //===================================================================================
    // State propagation, and clone augmentation
    //===================================================================================

    // Return if the camera measurement is out of order
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "image received out of order (prop dt = %3f)\n" RESET,(timestamp-state->_timestamp));
        return;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    propagator->propagate_and_clone(state, timestamp);
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // If we have not reached max clones, we should just return...
    // This isn't super ideal, but it keeps the logic after this easier...
    // We can start processing things when we have at least 5 clones since we can start triangulating things...
    if((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size,5)) {
        printf("waiting for enough clone states (%d of %d)....\n",(int)state->_clones_IMU.size(),std::min(state->_options.max_clone_size,5));
        return;
    }

    // Return if we where unable to propagate
    if(state->_timestamp != timestamp) {
        printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
        printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,timestamp-state->_timestamp);
        return;
    }

    //===================================================================================
    // MSCKF features and KLT tracks that are SLAM features
    //===================================================================================


    // Now, lets get all features that should be used for an update that are lost in the newest frame
    std::vector<Feature*> feats_lost, feats_marg, feats_slam;
    feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp);

    // Don't need to get the oldest features untill we reach our max number of clones
    if((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
        feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());
        if(trackARUCO != nullptr && timestamp-startup_time >= params.dt_slam_delay) {
            feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep());
        }
    }

    // We also need to make sure that the max tracks does not contain any lost features
    // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
    auto it1 = feats_lost.begin();
    while(it1 != feats_lost.end()) {
        if(std::find(feats_marg.begin(),feats_marg.end(),(*it1)) != feats_marg.end()) {
            //printf(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
            it1 = feats_lost.erase(it1);
        } else {
            it1++;
        }
    }

    // Find tracks that have reached max length, these can be made into SLAM features
    std::vector<Feature*> feats_maxtracks;
    auto it2 = feats_marg.begin();
    while(it2 != feats_marg.end()) {
        // See if any of our camera's reached max track
        bool reached_max = false;
        for (const auto &cams: (*it2)->timestamps) {
            if ((int)cams.second.size() > state->_options.max_clone_size) {
                reached_max = true;
                break;
            }
        }
        // If max track, then add it to our possible slam feature list
        if(reached_max) {
            feats_maxtracks.push_back(*it2);
            it2 = feats_marg.erase(it2);
        } else {
            it2++;
        }
    }

    // Count how many aruco tags we have in our state
    int curr_aruco_tags = 0;
    auto it0 = state->_features_SLAM.begin();
    while(it0 != state->_features_SLAM.end()) {
        if ((int) (*it0).second->_featid <= state->_options.max_aruco_features) curr_aruco_tags++;
        it0++;
    }

    // Append a new SLAM feature if we have the room to do so
    // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
    if(state->_options.max_slam_features > 0 && timestamp-startup_time >= params.dt_slam_delay && (int)state->_features_SLAM.size() < state->_options.max_slam_features+curr_aruco_tags) {
        // Get the total amount to add, then the max amount that we can add given our marginalize feature array
        int amount_to_add = (state->_options.max_slam_features+curr_aruco_tags)-(int)state->_features_SLAM.size();
        int valid_amount = (amount_to_add > (int)feats_maxtracks.size())? (int)feats_maxtracks.size() : amount_to_add;
        // If we have at least 1 that we can add, lets add it!
        // Note: we remove them from the feat_marg array since we don't want to reuse information...
        if(valid_amount > 0) {
            feats_slam.insert(feats_slam.end(), feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
            feats_maxtracks.erase(feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
        }
    }

    // Loop through current SLAM features, we have tracks of them, grab them for this update!
    // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
    // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
    for (std::pair<const size_t, Landmark*> &landmark : state->_features_SLAM) {
        if(trackARUCO != nullptr) {
            Feature* feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
            if(feat1 != nullptr) feats_slam.push_back(feat1);
        }
        Feature* feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
        if(feat2 != nullptr) feats_slam.push_back(feat2);
        if(feat2 == nullptr) landmark.second->should_marg = true;
    }

    // Lets marginalize out all old SLAM features here
    // These are ones that where not successfully tracked into the current frame
    // We do *NOT* marginalize out our aruco tags
    StateHelper::marginalize_slam(state);

    // Separate our SLAM features into new ones, and old ones
    std::vector<Feature*> feats_slam_DELAYED, feats_slam_UPDATE;
    for(size_t i=0; i<feats_slam.size(); i++) {
        if(state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
            feats_slam_UPDATE.push_back(feats_slam.at(i));
            //printf("[UPDATE-SLAM]: found old feature %d (%d measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        } else {
            feats_slam_DELAYED.push_back(feats_slam.at(i));
            //printf("[UPDATE-SLAM]: new feature ready %d (%d measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        }
    }

    // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
    std::vector<Feature*> featsup_MSCKF = feats_lost;
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());


    //===================================================================================
    // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
    //===================================================================================


    // Pass them to our MSCKF updater
    // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
    // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
    if((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
        featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end()-state->_options.max_msckf_in_update);
    updaterMSCKF->update(state, featsup_MSCKF);
    rT4 =  boost::posix_time::microsec_clock::local_time();

    // Perform SLAM delay init and update
    // NOTE: that we provide the option here to do a *sequential* update
    // NOTE: this will be a lot faster but won't be as accurate.
    std::vector<Feature*> feats_slam_UPDATE_TEMP;
    while(!feats_slam_UPDATE.empty()) {
        // Get sub vector of the features we will update with
        std::vector<Feature*> featsup_TEMP;
        featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(), feats_slam_UPDATE.begin()+std::min(state->_options.max_slam_in_update,(int)feats_slam_UPDATE.size()));
        feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(), feats_slam_UPDATE.begin()+std::min(state->_options.max_slam_in_update,(int)feats_slam_UPDATE.size()));
        // Do the update
        updaterSLAM->update(state, featsup_TEMP);
        feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
    }
    feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
    rT5 =  boost::posix_time::microsec_clock::local_time();
    updaterSLAM->delayed_init(state, feats_slam_DELAYED);
    rT6 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Update our visualization feature set, and clean up the old features
    //===================================================================================


    // Save all the MSCKF features used in the update
    good_features_MSCKF.clear();
    for(Feature* feat : featsup_MSCKF) {
        good_features_MSCKF.push_back(feat->p_FinG);
        feat->to_delete = true;
    }

    // Remove features that where used for the update from our extractors at the last timestep
    // This allows for measurements to be used in the future if they failed to be used this time
    // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
    trackFEATS->get_feature_database()->cleanup();
    if(trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup();
    }

    //===================================================================================
    // Cleanup, marginalize out what we don't need any more...
    //===================================================================================

    // First do anchor change if we are about to lose an anchor pose
    updaterSLAM->change_anchors(state);

    // Marginalize the oldest clone of the state if we are at max length
    if((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
        // Cleanup any features older then the marginalization time
        trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
        if(trackARUCO != nullptr) {
            trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
        }
        // Finally marginalize that clone
        StateHelper::marginalize_old_clone(state);
    }

    // Finally if we are optimizing our intrinsics, update our trackers
    if(state->_options.do_calib_camera_intrinsics) {
        // Get vectors arrays
        std::map<size_t, Eigen::VectorXd> cameranew_calib;
        std::map<size_t, bool> cameranew_fisheye;
        for(int i=0; i<state->_options.num_cameras; i++) {
            Vec* calib = state->_cam_intrinsics.at(i);
            bool isfish = state->_cam_intrinsics_model.at(i);
            cameranew_calib.insert({i,calib->value()});
            cameranew_fisheye.insert({i,isfish});
        }
        // Update the trackers and their databases
        trackFEATS->set_calibration(cameranew_calib, cameranew_fisheye, true);
        if(trackARUCO != nullptr) {
            trackARUCO->set_calibration(cameranew_calib, cameranew_fisheye, true);
        }
    }
    rT7 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Debug info, and stats tracking
    //===================================================================================

    // Get timing statitics information
    double time_track = (rT2-rT1).total_microseconds() * 1e-6;
    double time_prop = (rT3-rT2).total_microseconds() * 1e-6;
    double time_msckf = (rT4-rT3).total_microseconds() * 1e-6;
    double time_slam_update = (rT5-rT4).total_microseconds() * 1e-6;
    double time_slam_delay = (rT6-rT5).total_microseconds() * 1e-6;
    double time_marg = (rT7-rT6).total_microseconds() * 1e-6;
    double time_total = (rT7-rT1).total_microseconds() * 1e-6;

    // Timing information
    printf(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
    printf(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
    printf(BLUE "[TIME]: %.4f seconds for MSCKF update (%d features)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
    if(state->_options.max_slam_features > 0) {
        printf(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET, time_slam_update, (int)feats_slam_UPDATE.size());
        printf(BLUE "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
    }
    printf(BLUE "[TIME]: %.4f seconds for marginalization (%d clones in state)\n" RESET, time_marg, (int)state->_clones_IMU.size());
    printf(BLUE "[TIME]: %.4f seconds for total\n" RESET, time_total);

    // Finally if we are saving stats to file, lets save it to file
    if(params.record_timing_information && of_statistics.is_open()) {
        // We want to publish in the IMU clock frame
        // The timestamp in the state will be the last camera time
        double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
        double timestamp_inI = state->_timestamp + t_ItoC;
        // Append to the file
        of_statistics << std::fixed << std::setprecision(15)
                      << timestamp_inI << ","
                      << std::fixed << std::setprecision(5)
                      << time_track << "," << time_prop << "," << time_msckf << ",";
        if(state->_options.max_slam_features > 0) {
            of_statistics << time_slam_update << "," << time_slam_delay << ",";
        }
        of_statistics << time_marg << "," << time_total << std::endl;
        of_statistics.flush();
    }


    // Update our distance traveled
    if(timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
        Eigen::Matrix<double,3,1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
        distance += dx.norm();
    }
    timelastupdate = timestamp;

    // Debug, print our current state
    printf("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n",
            state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3),
            state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2),distance);
    printf("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n",
             state->_imu->bias_g()(0),state->_imu->bias_g()(1),state->_imu->bias_g()(2),
             state->_imu->bias_a()(0),state->_imu->bias_a()(1),state->_imu->bias_a()(2));


    // Debug for camera imu offset
    if(state->_options.do_calib_camera_timeoffset) {
        printf("camera-imu timeoffset = %.5f\n",state->_calib_dt_CAMtoIMU->value()(0));
    }

    // Debug for camera intrinsics
    if(state->_options.do_calib_camera_intrinsics) {
        for(int i=0; i<state->_options.num_cameras; i++) {
            Vec* calib = state->_cam_intrinsics.at(i);
            printf("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n",(int)i,
                     calib->value()(0),calib->value()(1),calib->value()(2),calib->value()(3),
                     calib->value()(4),calib->value()(5),calib->value()(6),calib->value()(7));
        }
    }

    // Debug for camera extrinsics
    if(state->_options.do_calib_camera_pose) {
        for(int i=0; i<state->_options.num_cameras; i++) {
            PoseJPL* calib = state->_calib_IMUtoCAM.at(i);
            printf("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n",(int)i,
                     calib->quat()(0),calib->quat()(1),calib->quat()(2),calib->quat()(3),
                     calib->pos()(0),calib->pos()(1),calib->pos()(2));
        }
    }


}























