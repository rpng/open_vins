#include "UpdaterSLAM.h"



using namespace ov_core;
using namespace ov_msckf;





void UpdaterSLAM::delayed_init(State *state, std::vector<Feature*>& feature_vec) {


    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;
    for(const auto& clone_imu : state->get_clones()) {
        clonetimes.emplace_back(clone_imu.first);
    }

    // 1. Clean all feature measurements and make sure they all have valid clone times
    auto it0 = feature_vec.begin();
    while(it0 != feature_vec.end()) {

        // Clean the feature
        (*it0)->clean_old_measurements(clonetimes);

        // Count how many measurements
        int ct_meas = 0;
        for(const auto &pair : (*it0)->timestamps) {
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        // Remove if we don't have enough
        if(ct_meas < 3) {
            (*it0)->to_delete = true;
            it0 = feature_vec.erase(it0);
        } else {
            it0++;
        }

    }

    // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
    std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
    for(const auto &clone_calib : state->get_calib_IMUtoCAMs()) {

        // For this camera, create the vector of camera poses
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for(const auto &clone_imu : state->get_clones()) {

            // Get current camera pose
            Eigen::Matrix<double,3,3> R_GtoCi = clone_calib.second->Rot()*clone_imu.second->Rot();
            Eigen::Matrix<double,3,1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose()*clone_calib.second->pos();

            // Append to our map
            clones_cami.insert({clone_imu.first,FeatureInitializer::ClonePose(R_GtoCi,p_CioinG)});

        }

        // Append to our map
        clones_cam.insert({clone_calib.first,clones_cami});

    }



    // TODO:!@#!@#!#!#!@#!@#!#!#!@#!#!@#!@#!#!@$!@#$!@$!@#!@#!@#!@#!@#!@!##@
    // TODO: Re-normalized image uvs here with the current best guess
    // TODO:!@#!@#!#!#!@#!@#!#!#!@#!#!@#!@#!#!@$!@#$!@$!@#!@#!@#!@#!@#!@!##@



    // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
    auto it1 = feature_vec.begin();
    while(it1 != feature_vec.end()) {

        // Triangulate the feature and remove if it fails
        bool success = initializer_feat->single_triangulation(*it1, clones_cam);
        if(!success) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);
            continue;
        }

        // Gauss-newton refine the feature
        success = initializer_feat->single_gaussnewton(*it1, clones_cam);
        if(!success) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);
            continue;
        }
        it1++;

    }

    // 4. Compute linear system for each feature, nullspace project, and reject
    auto it2 = feature_vec.begin();
    while(it2 != feature_vec.end()) {

        // Convert our feature into our current format
        UpdaterHelper::UpdaterHelperFeature feat;
        feat.featid = (*it2)->featid;
        feat.uvs = (*it2)->uvs;
        feat.uvs_norm = (*it2)->uvs_norm;
        feat.timestamps = (*it2)->timestamps;
        feat.anchor_cam_id = (*it2)->anchor_cam_id;
        feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
        feat.p_FinG = (*it2)->p_FinG;

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        std::vector<Type*> Hx_order;

        // Get the Jacobian for this feature
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        //Create feature pointer
        Landmark* landmark = new Landmark();
        landmark->_featid = feat.featid;
        landmark->_anchor_cam_id = feat.anchor_cam_id;
        landmark->_anchor_clone_timestamp = feat.anchor_clone_timestamp;
        landmark->set_from_global_xyz(state, feat.p_FinG);

        // Measurement noise matrix
        double sigma_pix_sq = ((int)feat.featid < state->options().max_aruco_features)? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
        Eigen::MatrixXd R = sigma_pix_sq*Eigen::MatrixXd::Identity(res.rows(), res.rows());

        // Try to initialize, delete new pointer if we failed
        double chi2_multipler = ((int)feat.featid < state->options().max_aruco_features)? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
        if (StateHelper::initialize(state, landmark, Hx_order, H_x, H_f, R, res, chi2_multipler)){
            state->insert_SLAM_feature((*it2)->featid, landmark);
            (*it2)->to_delete = true;
            it2++;
        } else {
            delete landmark;
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
        }

    }

}






void UpdaterSLAM::update(State *state, std::vector<Feature*>& feature_vec) {


    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;
    for(const auto& clone_imu : state->get_clones()) {
        clonetimes.emplace_back(clone_imu.first);
    }


    // 1. Clean all feature measurements and make sure they all have valid clone times
    auto it0 = feature_vec.begin();
    while(it0 != feature_vec.end()) {

        // Clean the feature
        (*it0)->clean_old_measurements(clonetimes);

        // Count how many measurements
        int ct_meas = 0;
        for(const auto &pair : (*it0)->timestamps) {
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        // Remove if we don't have enough
        if(ct_meas < 1) {
            (*it0)->to_delete = true;
            it0 = feature_vec.erase(it0);
        } else {
            it0++;
        }

    }


    // Calculate the max possible measurement size
    size_t max_meas_size = 0;
    for(size_t i=0; i<feature_vec.size(); i++) {
        for (const auto &pair : feature_vec.at(i)->timestamps) {
            max_meas_size += 2*feature_vec.at(i)->timestamps[pair.first].size();
        }
    }

    // Calculate max possible state size (i.e. the size of our covariance)
    size_t max_hx_size = state->n_vars();

    // Large Jacobian, residual, and measurement noise of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Identity(max_meas_size,max_meas_size);
    std::unordered_map<Type*,size_t> Hx_mapping;
    std::vector<Type*> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;

    //Get slam features
    auto& features_SLAM = state->features_SLAM();

    // 4. Compute linear system for each feature, nullspace project, and reject
    auto it2 = feature_vec.begin();
    while(it2 != feature_vec.end()) {

        // Ensure we have the landmark and it is the same
        assert(features_SLAM.find((*it2)->featid) != features_SLAM.end());
        assert(features_SLAM.at((*it2)->featid)->_featid == (*it2)->featid);

        // Get our landmark from the state
        Landmark* landmark = features_SLAM.at((*it2)->featid);

        // Convert the state landmark into our current format
        UpdaterHelper::UpdaterHelperFeature feat;
        feat.featid = (*it2)->featid;
        feat.uvs = (*it2)->uvs;
        feat.uvs_norm = (*it2)->uvs_norm;
        feat.timestamps = (*it2)->timestamps;
        feat.anchor_cam_id = landmark->_anchor_cam_id;
        feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
        feat.p_FinG = landmark->get_global_xyz(state);

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        std::vector<Type*> Hx_order;

        // Get the Jacobian for this feature
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        //Place Jacobians in one big Jacobian, since the landmark is already in our state vector
        Eigen::MatrixXd H_xf = H_x;
        H_xf.conservativeResize(H_x.rows(), H_x.cols()+3);
        H_xf.block(0, H_x.cols(), H_x.rows(), 3) = H_f;

        std::vector<Type*> Hxf_order = Hx_order;
        Hxf_order.push_back(landmark);

        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hxf_order);
        Eigen::MatrixXd S = H_xf*P_marg*H_xf.transpose();
        double sigma_pix_sq = ((int)feat.featid < state->options().max_aruco_features)? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
        S.diagonal() += sigma_pix_sq*Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;
        if(res.rows() < 500) {
            chi2_check = chi_squared_table[res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(res.rows());
            chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
        }

        // Check if we should delete or not
        double chi2_multipler = ((int)feat.featid < state->options().max_aruco_features)? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
        if(chi2 > chi2_multipler*chi2_check) {
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
            continue;
        }

        // We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for(const auto &var : Hxf_order) {

            // Ensure that this variable is in our Jacobian
            if(Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->size();
            }

            // Append to our large Jacobian
            Hx_big.block(ct_meas,Hx_mapping[var],H_xf.rows(),var->size()) = H_xf.block(0,ct_hx,H_xf.rows(),var->size());
            ct_hx += var->size();

        }

        // Our isotropic measurement noise
        R_big.block(ct_meas,ct_meas,res.rows(),res.rows()) *= sigma_pix_sq;

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1) = res;
        ct_meas += res.rows();
        it2++;

    }

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information
    for (size_t f=0; f < feature_vec.size(); f++){
        feature_vec[f]->to_delete = true;
    }

    // Return if we don't have anything and resize our matrices
    if(ct_meas < 1) {
        return;
    }
    assert(ct_meas<=max_meas_size);
    assert(ct_jacob<=max_hx_size);
    res_big.conservativeResize(ct_meas,1);
    Hx_big.conservativeResize(ct_meas,ct_jacob);
    R_big.conservativeResize(ct_meas,ct_meas);

    // 6. With all good SLAM features update the state
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    std::cout << "updated with " << ct_meas << " slam measurements" << std::endl;

}





void UpdaterSLAM::change_anchors(State* state){

    // If we do not have an anchored representation, just return
    if (state->options().feat_representation == StateOptions::GLOBAL_3D ||
        state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {
        return;
    }

    // Return if we do not have enough clones
    if ((int) state->n_clones() <= state->options().max_clone_size) {
        return;
    }

    // Get the marginalization timestep, and change the anchor for any feature seen from it
    // NOTE: for now we have anchor the feature in the same camera as it is before
    // NOTE: this also does not change the representation of the feature at all right now
    double marg_timestep = state->margtimestep();
    for (auto &f : state->features_SLAM()){
        assert(marg_timestep <= f.second->_anchor_clone_timestamp);
        if (f.second->_anchor_clone_timestamp == marg_timestep){
            perform_anchor_change(state, f.second, state->timestamp(), f.second->_anchor_cam_id);
        }
    }

}




void UpdaterSLAM::perform_anchor_change(State* state, Landmark* landmark, double new_anchor_timestamp, size_t new_cam_id){


    // Create current feature representation
    UpdaterHelper::UpdaterHelperFeature old_feat;
    old_feat.featid = landmark->_featid;
    old_feat.anchor_cam_id = landmark->_anchor_cam_id;
    old_feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
    old_feat.p_FinG = landmark->get_global_xyz(state);


    // Get Jacobians of p_FinG wrt old representation
    Eigen::Matrix<double,3,3> H_f_old;
    std::vector<Eigen::Matrix<double,3,Eigen::Dynamic>> H_x_old;
    std::vector<Type*> x_order_old;
    UpdaterHelper::get_feature_jacobian_representation(state, old_feat, H_f_old, H_x_old, x_order_old);


    // Create future feature representation
    UpdaterHelper::UpdaterHelperFeature new_feat;
    new_feat.featid = landmark->_featid;
    new_feat.anchor_cam_id = new_cam_id;
    new_feat.anchor_clone_timestamp = new_anchor_timestamp;
    new_feat.p_FinG = landmark->get_global_xyz(state);


    // Get Jacobians of p_FinG wrt new representation
    Eigen::Matrix<double,3,3> H_f_new;
    std::vector<Eigen::Matrix<double,3,Eigen::Dynamic>> H_x_new;
    std::vector<Type*> x_order_new;
    UpdaterHelper::get_feature_jacobian_representation(state, new_feat, H_f_new, H_x_new, x_order_new);


    // Anchor change Jacobian
    Eigen::MatrixXd Phi(3, state->Cov().rows());
    Phi.setZero();

    // Inverse of our new representation
    // pf_new_error = Hfnew^{-1}*(Hfold*pf_olderror+Hxold*x_olderror+Hxnew*x_newerror)
    Eigen::Matrix<double,3,3> H_f_new_inv = H_f_new.inverse();

    // Place Jacobians for old anchor
    for (size_t i = 0; i < H_x_old.size(); i++){
        Phi.block(0,x_order_old[i]->id(),3,x_order_old[i]->size()).noalias() += H_f_new_inv*H_x_old[i];
    }

    // Place Jacobians for old feat
    Phi.block(0, landmark->id(), 3,3) = H_f_new_inv*H_f_old;

    // Place Jacobians for new anchor
    for (size_t i = 0; i < H_x_new.size(); i++){
        Phi.block(0,x_order_new[i]->id(),3,x_order_new[i]->size()).noalias() -= H_f_new_inv*H_x_new[i];
    }

    // Perform covariance propagation
    auto &Cov = state->Cov();
    Eigen::Matrix<double,-1,3> Pxf = Cov*Phi.transpose();
    Eigen::Matrix<double,3,3> Pff = Phi*Pxf;

    // Replace the blocks in our covariance
    Cov.block(landmark->id(), 0, 3, Cov.rows()) = Pxf.transpose();
    Cov.block(0, landmark->id(), Cov.rows(), 3) = Pxf;
    Cov.block(landmark->id(),landmark->id(), 3, 3) = Pff;
    Cov = 0.5*(Cov + Cov.transpose());

    // Set state from new feature
    landmark->_featid = new_feat.featid;
    landmark->_anchor_cam_id = new_feat.anchor_cam_id;
    landmark->_anchor_clone_timestamp = new_feat.anchor_clone_timestamp;
    landmark->set_from_global_xyz(state, new_feat.p_FinG);

}





