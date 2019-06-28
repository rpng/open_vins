#include "UpdaterMSCKF.h"



using namespace ov_core;
using namespace ov_msckf;





void UpdaterMSCKF::update(State *state, std::vector<Feature*>& feature_vec) {

    // Return if no features
    if(feature_vec.empty())
        return;

    // Start timing
    boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5, rT6, rT7;
    rT0 =  boost::posix_time::microsec_clock::local_time();

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
    rT1 =  boost::posix_time::microsec_clock::local_time();

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
    rT2 =  boost::posix_time::microsec_clock::local_time();


    // Calculate the max possible measurement size
    size_t max_meas_size = 0;
    for(size_t i=0; i<feature_vec.size(); i++) {
        for (const auto &pair : feature_vec.at(i)->timestamps) {
            max_meas_size += 2*feature_vec.at(i)->timestamps[pair.first].size();
        }
    }

    // Calculate max possible state size (i.e. the size of our covariance)
    size_t max_hx_size = state->n_vars();
    max_hx_size -= 3*state->features_SLAM().size();

    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    std::unordered_map<Type*,size_t> Hx_mapping;
    std::vector<Type*> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;


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
        feat.p_FinG_fej = (*it2)->p_FinG;

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        std::vector<Type*> Hx_order;

        // Get the Jacobian for this feature
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        // Nullspace project
        UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
        Eigen::MatrixXd S = H_x*P_marg*H_x.transpose();
        S.diagonal() += _options.sigma_pix_sq*Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;
        if(res.rows() < 500) {
            chi2_check = chi_squared_table[res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(res.rows());
            chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
            std::cout << "chi2_check over the residual limit - " << res.rows() << std::endl;
        }

        // Check if we should delete or not
        if(chi2 > _options.chi2_multipler*chi2_check) {
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
            continue;
        }

        // We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for(const auto &var : Hx_order) {

            // Ensure that this variable is in our Jacobian
            if(Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->size();
            }

            // Append to our large Jacobian
            Hx_big.block(ct_meas,Hx_mapping[var],H_x.rows(),var->size()) = H_x.block(0,ct_hx,H_x.rows(),var->size());
            ct_hx += var->size();

        }

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1) = res;
        ct_meas += res.rows();
        it2++;

    }
    rT3 =  boost::posix_time::microsec_clock::local_time();

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


    // 5. Perform measurement compression
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
    if(Hx_big.rows() < 1) {
        return;
    }
    rT4 =  boost::posix_time::microsec_clock::local_time();


    // Our noise is isotropic, so make it here after our compression
    Eigen::MatrixXd R_big = _options.sigma_pix_sq*Eigen::MatrixXd::Identity(res_big.rows(),res_big.rows());

    // 6. With all good features update the state
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    rT5 =  boost::posix_time::microsec_clock::local_time();

    // Debug print timing information
    ROS_INFO("[MSCKF-UP]: %.4f seconds to clean",(rT1-rT0).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds to triangulate",(rT2-rT1).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds create system (%d features)",(rT3-rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    ROS_INFO("[MSCKF-UP]: %.4f seconds compress system",(rT4-rT3).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds update state (%d size)",(rT5-rT4).total_microseconds() * 1e-6, (int)res_big.rows());
    ROS_INFO("[MSCKF-UP]: %.4f seconds total",(rT5-rT1).total_microseconds() * 1e-6);

}










