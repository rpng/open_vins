#include "UpdaterMSCKF.h"



using namespace ov_core;
using namespace ov_msckf;





void UpdaterMSCKF::update(State *state, std::vector<Feature*>& feature_vec) {


    // 1. Clean all feature measurements and make sure they all have valid clone times
    auto it0 = feature_vec.begin();
    while(it0 != feature_vec.end()) {

        // Clean the feature
        clean_feature(state, *it0);

        // Count how many measurements
        int ct_meas = 0;
        for(const auto &pair : (*it0)->timestamps) {
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        // Remove if we don't have enough
        if(ct_meas < 3) {
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
            it1 = feature_vec.erase(it1);
            std::cout << "failed triangulation - " << (*it1)->featid << std::endl;
            continue;
        }

        // Gauss-newton refine the feature
        success = initializer_feat->single_gaussnewton(*it1, clones_cam);
        if(!success) {
            it1 = feature_vec.erase(it1);
            std::cout << "failed gauss newton - " << (*it1)->featid << std::endl;
            continue;
        }
        it1++;

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
    //max_hx_size -= 3*state->get_slam_feats();

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

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        std::vector<Type*> Hx_order;

        // Get the Jacobian for this feature
        UpdaterHelper::get_feature_jacobian_full(state, *it2, H_f, H_x, res, Hx_order);

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
        }

        // Check if we should delete or not
        if(chi2 > _options.chi2_multipler*chi2_check) {
            it2 = feature_vec.erase(it2);
            std::cout << "chi2 failed - " << chi2 << " > " << _options.chi2_multipler*chi2_check << std::endl;
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

    // Return if we don't have anything and resize our matrices
    if(ct_meas < 1) {
        return;
    }
    res_big.conservativeResize(ct_meas,1);
    Hx_big.conservativeResize(ct_meas,ct_jacob);


    // 5. Perform measurement compression
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
    if(Hx_big.rows() < 1) {
        return;
    }

    // Our noise is isotropic, so make it here after our compression
    Eigen::MatrixXd R_big = _options.sigma_pix_sq*Eigen::MatrixXd::Identity(res_big.rows(),res_big.rows());


    // 6. With all good features update the state
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    std::cout << "state has been updated " << std::endl;
    std::cout << "Hx_big - " << Hx_big.norm() << std::endl;
    std::cout << "res_big - " << res_big.norm() << std::endl;
    std::cout << "R_big - " << R_big.norm() << std::endl;
    std::cout << "good feats - " << feature_vec.size() << std::endl;
    sleep(3);


}




void UpdaterMSCKF::clean_feature(State *state, Feature* feature) {

    // Loop through each of the cameras we have
    for(auto const &pair : feature->timestamps) {

        // Assert that we have all the parts of a measurement
        assert(feature->timestamps[pair.first].size() == feature->uvs[pair.first].size());
        assert(feature->timestamps[pair.first].size() == feature->uvs_norm[pair.first].size());

        // Our iterators
        auto it1 = feature->timestamps[pair.first].begin();
        auto it2 = feature->uvs[pair.first].begin();
        auto it3 = feature->uvs_norm[pair.first].begin();

        // Loop through measurement times, remove ones that are not in our clone times
        while (it1 != feature->timestamps[pair.first].end()) {
            if (state->get_clones().find(*it1) == state->get_clones().end()) {
                it1 = feature->timestamps[pair.first].erase(it1);
                it2 = feature->uvs[pair.first].erase(it2);
                it3 = feature->uvs_norm[pair.first].erase(it3);
            } else {
                ++it1;
                ++it2;
                ++it3;
            }
        }
    }

}





