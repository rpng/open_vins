#include "UpdaterHelper.h"


using namespace ov_core;
using namespace ov_msckf;


void UpdaterHelper::get_feature_jacobian_representation(State* state, UpdaterHelperFeature &feature, Eigen::Matrix<double,3,3> &H_f,
                                                        std::vector<Eigen::Matrix<double,3,Eigen::Dynamic>> &H_x, std::vector<Type*> &x_order) {

    // Global XYZ representation
    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        H_f.resize(3,3);
        H_f.setIdentity();
        return;
    }

    // Global inverse depth representation
    if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {

        // Get inverse depth representation (should match what is in Landmark.cpp)
        double g_rho = 1/feature.p_FinG.norm();
        double g_phi = std::acos(g_rho*feature.p_FinG(2));
        //double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
        double g_theta = std::atan2(feature.p_FinG(1),feature.p_FinG(0));
        Eigen::Matrix<double,3,1> p_invFinG;
        p_invFinG(0) = g_theta;
        p_invFinG(1) = g_phi;
        p_invFinG(2) = g_rho;

        // Get inverse depth bearings
        double sin_th = std::sin(p_invFinG(0,0));
        double cos_th = std::cos(p_invFinG(0,0));
        double sin_phi = std::sin(p_invFinG(1,0));
        double cos_phi = std::cos(p_invFinG(1,0));
        double rho = p_invFinG(2,0);

        // Construct the Jacobian
        H_f.resize(3,3);
        H_f << -(1.0/rho)*sin_th*sin_phi, (1.0/rho)*cos_th*cos_phi, -(1.0/(rho*rho))*cos_th*sin_phi,
                (1.0/rho)*cos_th*sin_phi, (1.0/rho)*sin_th*cos_phi, -(1.0/(rho*rho))*sin_th*sin_phi,
                0.0, -(1.0/rho)*sin_phi, -(1.0/(rho*rho))*cos_phi;
        return;
    }


    //======================================================================
    //======================================================================
    //======================================================================


    // Assert that we have an anchor pose for this feature
    assert(feature.anchor_cam_id!=-1);

    // Get calibration for our anchor camera
    // TODO: should we do fej with calibration?
    Eigen::Matrix<double,3,3> R_ItoC = state->get_calib_IMUtoCAM(feature.anchor_cam_id)->Rot();
    Eigen::Matrix<double,3,1> p_IinC = state->get_calib_IMUtoCAM(feature.anchor_cam_id)->pos();

    // Anchor pose orientation and position
    Eigen::Matrix<double,3,3> R_GtoI = (state->options().do_fej)? state->get_clone(feature.anchor_clone_timestamp)->Rot_fej() : state->get_clone(feature.anchor_clone_timestamp)->Rot();
    Eigen::Matrix<double,3,1> p_IinG = (state->options().do_fej)? state->get_clone(feature.anchor_clone_timestamp)->pos_fej() : state->get_clone(feature.anchor_clone_timestamp)->pos();;
    Eigen::Matrix<double,3,3> R_CtoG = R_GtoI.transpose()*R_ItoC.transpose();

    // Calculate position of the feature in our anchor position
    Eigen::Matrix<double,3,1> p_FinA = R_ItoC*R_GtoI*(feature.p_FinG-p_IinG)+p_IinC;

    // TODO: when using fej with SLAM need to replace the value of feature position
    Eigen::Matrix<double,3,6> H_anc;
    H_anc.block(0,0,3,3).noalias() = -R_GtoI.transpose()*skew_x(R_ItoC.transpose()*(p_FinA-p_IinC));
    H_anc.block(0,3,3,3).setIdentity();

    // Add anchor Jacobians to our return vector
    x_order.push_back(state->get_clone(feature.anchor_clone_timestamp));
    H_x.push_back(H_anc);

    // Get calibration Jacobians (for anchor clone)
    if (state->options().do_calib_camera_pose){
        Eigen::Matrix<double,3,6> H_calib;
        H_calib.block(0,0,3,3).noalias() = -R_CtoG*skew_x(p_FinA-p_IinC);
        H_calib.block(0,3,3,3) = -R_CtoG;
        x_order.push_back(state->get_calib_IMUtoCAM(feature.anchor_cam_id));
        H_x.push_back(H_calib);
    }

    // If we are doing anchored XYZ feature
    if(state->options().feat_representation == StateOptions::ANCHORED_3D) {
        H_f = R_CtoG;
        return;
    }

    // If we are doing full inverse depth
    if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH) {

        // Get inverse depth representation (should match what is in Landmark.cpp)
        double a_rho = 1/p_FinA.norm();
        double a_phi = std::acos(a_rho*p_FinA(2));
        double a_theta = std::atan2(p_FinA(1),p_FinA(0));
        Eigen::Matrix<double,3,1> p_invFinA;
        p_invFinA(0) = a_theta;
        p_invFinA(1) = a_phi;
        p_invFinA(2) = a_rho;

        // Using anchored inverse depth
        double sin_th = std::sin(p_invFinA(0,0));
        double cos_th = std::cos(p_invFinA(0,0));
        double sin_phi = std::sin(p_invFinA(1,0));
        double cos_phi = std::cos(p_invFinA(1,0));
        double rho = p_invFinA(2,0);

        // Jacobian of anchored 3D position wrt inverse depth parameters
        Eigen::Matrix<double,3,3> d_pfinA_dpinv;
        d_pfinA_dpinv << -(1.0/rho)*sin_th*sin_phi, (1.0/rho)*cos_th*cos_phi, -(1.0/(rho*rho))*cos_th*sin_phi,
                (1.0/rho)*cos_th*sin_phi, (1.0/rho)*sin_th*cos_phi, -(1.0/(rho*rho))*sin_th*sin_phi,
                0.0, -(1.0/rho)*sin_phi, -(1.0/(rho*rho))*cos_phi;
        H_f = R_CtoG*d_pfinA_dpinv;
        return;
    }

    // If we are doing the MSCKF version of inverse depth
    if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {

        // Get inverse depth representation (should match what is in Landmark.cpp)
        Eigen::Matrix<double,3,1> p_invFinA_MSCKF;
        p_invFinA_MSCKF(0) = p_FinA(0)/p_FinA(2);
        p_invFinA_MSCKF(1) = p_FinA(1)/p_FinA(2);
        p_invFinA_MSCKF(2) = 1/p_FinA(2);

        // Using the MSCKF version of inverse depth
        double alpha = p_invFinA_MSCKF(0,0);
        double beta = p_invFinA_MSCKF(1,0);
        double rho = p_invFinA_MSCKF(2,0);

        // Jacobian of anchored 3D position wrt inverse depth parameters
        Eigen::Matrix<double,3,3> d_pfinA_dpinv;
        d_pfinA_dpinv << (1.0/rho), 0.0, -(1.0/(rho*rho))*alpha,
                0.0, (1.0/rho), -(1.0/(rho*rho))*beta,
                0.0, 0.0, -(1.0/(rho*rho));
        H_f = R_CtoG*d_pfinA_dpinv;
        return;
    }

}




void UpdaterHelper::get_feature_jacobian_full(State* state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<Type*> &x_order) {

    // Total number of measurements for this feature
    int total_meas = 0;
    for (auto const& pair : feature.timestamps) {
        total_meas += (int)pair.second.size();
    }

    // Compute the size of the states involved with this feature
    int total_hx = 0;
    std::unordered_map<Type*,size_t> map_hx;
    for (auto const& pair : feature.timestamps) {

        // Our extrinsics and intrinsics
        PoseJPL *calibration = state->get_calib_IMUtoCAM(pair.first);
        Vec *distortion = state->get_intrinsics_CAM(pair.first);

        // If doing calibration extrinsics
        if(state->options().do_calib_camera_pose) {
            map_hx.insert({calibration,total_hx});
            x_order.push_back(calibration);
            total_hx += calibration->size();
        }

        // If doing calibration intrinsics
        if(state->options().do_calib_camera_intrinsics) {
            map_hx.insert({distortion,total_hx});
            x_order.push_back(distortion);
            total_hx += distortion->size();
        }

        // Loop through all measurements for this specific camera
        for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

            // Add this clone if it is not added already
            PoseJPL *clone_Ci = state->get_clone(feature.timestamps[pair.first].at(m));
            if(map_hx.find(clone_Ci) == map_hx.end()) {
                map_hx.insert({clone_Ci,total_hx});
                x_order.push_back(clone_Ci);
                total_hx += clone_Ci->size();
            }

        }

    }

    // If we are using an anchored representation, make sure that the anchor is also added
    if (state->options().feat_representation == StateOptions::ANCHORED_3D
        || state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH
        || state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {

        // Add this anchor if it is not added already
        PoseJPL *clone_Ai = state->get_clone(feature.anchor_clone_timestamp);
        if(map_hx.find(clone_Ai) == map_hx.end()) {
            map_hx.insert({clone_Ai,total_hx});
            x_order.push_back(clone_Ai);
            total_hx += clone_Ai->size();
        }

        // Also add its calibration if we are doing calibration
        if(state->options().do_calib_camera_pose) {
            // Add this anchor if it is not added already
            PoseJPL *clone_calib = state->get_calib_IMUtoCAM(feature.anchor_cam_id);
            if(map_hx.find(clone_calib) == map_hx.end()) {
                map_hx.insert({clone_calib,total_hx});
                x_order.push_back(clone_calib);
                total_hx += clone_calib->size();
            }
        }

    }

    //=========================================================================
    //=========================================================================

    // Allocate our residual and Jacobians
    int c = 0;
    res = Eigen::VectorXd::Zero(2*total_meas);
    H_f = Eigen::MatrixXd::Zero(2*total_meas,3);
    H_x = Eigen::MatrixXd::Zero(2*total_meas,total_hx);

    // Derivative of p_FinG in respect to feature representation. This only needs to be computed once
    // and thus we pull it out of the loop
    Eigen::Matrix<double,3,3> dpfg_dlambda;
    std::vector<Eigen::Matrix<double,3,Eigen::Dynamic>> dpfg_dx;
    std::vector<Type*> dpfg_dx_order;
    UpdaterHelper::get_feature_jacobian_representation(state, feature, dpfg_dlambda, dpfg_dx, dpfg_dx_order);

    // Loop through each camera for this feature
    for (auto const& pair : feature.timestamps) {

        // Our calibration between the IMU and CAMi frames
        Vec* distortion = state->get_intrinsics_CAM(pair.first);
        PoseJPL* calibration = state->get_calib_IMUtoCAM(pair.first);
        Eigen::Matrix<double,3,3> R_ItoC = calibration->Rot();
        Eigen::Matrix<double,3,1> p_IinC = calibration->pos();
        Eigen::Matrix<double,8,1> cam_d = distortion->value();

        // Loop through all measurements for this specific camera
        for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

            //=========================================================================
            //=========================================================================

            // Get current IMU clone state
            PoseJPL* clone_Ii = state->get_clone(feature.timestamps[pair.first].at(m));
            Eigen::Matrix<double,3,3> R_GtoIi = clone_Ii->Rot();
            Eigen::Matrix<double,3,1> p_IiinG = clone_Ii->pos();

            Eigen::Matrix<double,3,1> p_FinIi = R_GtoIi*(feature.p_FinG-p_IiinG);

            // Project the current feature into the current frame of reference
            Eigen::Matrix<double,3,1> p_FinCi = R_ItoC*p_FinIi+p_IinC;
            Eigen::Matrix<double,2,1> uv_norm;
            uv_norm << p_FinCi(0)/p_FinCi(2),p_FinCi(1)/p_FinCi(2);

            // Distort the normalized coordinates (false=radtan, true=fisheye)
            Eigen::Matrix<double,2,1> uv_dist;

            // Derivative of measurement in respect to normalized coordinates
            Eigen::Matrix<double,2,2> dz_dzn = Eigen::Matrix<double,2,2>::Zero();
            Eigen::Matrix<double,2,8> dz_dzeta = Eigen::Matrix<double,2,8>::Zero();

            // Calculate distortion uv and jacobian
            if(state->get_model_CAM(pair.first)) {

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

                // If our r is small (meaning our xy is near the camera center) then don't distort
                //if(r < 1e-5) {
                //    uv_dist(0) = cam_d(0)*uv_norm(0) + cam_d(2);
                //    uv_dist(1) = cam_d(1)*uv_norm(1) + cam_d(3);
                //}

                // Jacobian of distorted pixel to "normalized" pixel
                Eigen::Matrix<double,2,2> duv_dxy1 = Eigen::Matrix<double,2,2>::Zero();
                duv_dxy1 << cam_d(0), 0, 0, cam_d(1);

                // Jacobian of "normalized" pixel to normalized pixel
                Eigen::Matrix<double,2,2> dxy1_dxy = Eigen::Matrix<double,2,2>::Zero();
                dxy1_dxy << theta_d*inv_r, 0, 0, theta_d*inv_r;

                // Jacobian of "normalized" pixel to r
                Eigen::Matrix<double,2,1> dxy1_dr = Eigen::Matrix<double,2,1>::Zero();
                dxy1_dr << -uv_norm(0)*theta_d*inv_r*inv_r, -uv_norm(1)*theta_d*inv_r*inv_r;

                // Jacobian of r pixel to normalized xy
                Eigen::Matrix<double,1,2> dr_dxy = Eigen::Matrix<double,1,2>::Zero();
                dxy1_dr << -uv_norm(0)*inv_r, -uv_norm(1)*inv_r;

                // Jacobian of "normalized" pixel to theta_d
                Eigen::Matrix<double,2,1> dxy1_dthd = dr_dxy.transpose();

                // Jacobian of theta_d to theta
                double dthd_dth = 1+3*cam_d(4)*std::pow(theta,2)+5*cam_d(5)*std::pow(theta,4) +7*cam_d(6)*std::pow(theta,6)+9*cam_d(7)*std::pow(theta,8);

                // Jacobian of theta to r
                double dth_dr = 1/(r*r+1);

                // Total Jacobian wrt normalized pixel coordinates
                dz_dzn = duv_dxy1*(dxy1_dxy+(dxy1_dr+dxy1_dthd*dthd_dth*dth_dr)*dr_dxy);


                // Compute the Jacobian in respect to the intrinsics if we are calibrating
                if(state->options().do_calib_camera_intrinsics) {
                    dz_dzeta(0,0) = x1;
                    dz_dzeta(0,2) = 1;
                    dz_dzeta(0,4) = cam_d(0)*uv_norm(0)*inv_r*std::pow(theta,3);
                    dz_dzeta(0,5) = cam_d(0)*uv_norm(0)*inv_r*std::pow(theta,5);
                    dz_dzeta(0,6) = cam_d(0)*uv_norm(0)*inv_r*std::pow(theta,7);
                    dz_dzeta(0,7) = cam_d(0)*uv_norm(0)*inv_r*std::pow(theta,9);
                    dz_dzeta(1,1) = y1;
                    dz_dzeta(1,3) = 1;
                    dz_dzeta(1,4) = cam_d(1)*uv_norm(1)*inv_r*std::pow(theta,3);
                    dz_dzeta(1,5) = cam_d(1)*uv_norm(1)*inv_r*std::pow(theta,5);
                    dz_dzeta(1,6) = cam_d(1)*uv_norm(1)*inv_r*std::pow(theta,7);
                    dz_dzeta(1,7) = cam_d(1)*uv_norm(1)*inv_r*std::pow(theta,9);
                }


            } else {



                // Calculate distorted coordinates for radial
                double r = std::sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
                double r_2 = r*r;
                double r_4 = r_2*r_2;
                double x1 = uv_norm(0)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+2*cam_d(6)*uv_norm(0)*uv_norm(1)+cam_d(7)*(r_2+2*uv_norm(0)*uv_norm(0));
                double y1 = uv_norm(1)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+cam_d(6)*(r_2+2*uv_norm(1)*uv_norm(1))+2*cam_d(7)*uv_norm(0)*uv_norm(1);
                uv_dist(0) = cam_d(0)*x1 + cam_d(2);
                uv_dist(1) = cam_d(1)*y1 + cam_d(3);

                // Jacobian of distorted pixel to normalized pixel
                dz_dzn(0,0) = cam_d(0)*((1+cam_d(4)*r_2+cam_d(5)*r_4)+(2*cam_d(4)*uv_norm(0)*uv_norm(0)+4*cam_d(5)*uv_norm(0)*uv_norm(0)*r)+2*cam_d(6)*uv_norm(1)+(2*cam_d(7)*uv_norm(0)+4*cam_d(7)*uv_norm(0)));
                dz_dzn(0,1) = cam_d(0)*(2*cam_d(4)*uv_norm(0)*uv_norm(1)+4*cam_d(5)*uv_norm(0)*uv_norm(1)*r+2*cam_d(6)*uv_norm(0)+2*cam_d(7)*uv_norm(1));
                dz_dzn(0,1) = cam_d(1)*(2*cam_d(4)*uv_norm(0)*uv_norm(1)+4*cam_d(5)*uv_norm(0)*uv_norm(1)*r+2*cam_d(6)*uv_norm(0)+2*cam_d(7)*uv_norm(1));
                dz_dzn(1,1) = cam_d(1)*((1+cam_d(4)*r_2+cam_d(5)*r_4)+(2*cam_d(4)*uv_norm(1)*uv_norm(1)+4*cam_d(5)*uv_norm(1)*uv_norm(1)*r)+2*cam_d(7)*uv_norm(0)+(2*cam_d(6)*uv_norm(1)+4*cam_d(6)*uv_norm(1)));

                // Compute the Jacobian in respect to the intrinsics if we are calibrating
                if(state->options().do_calib_camera_intrinsics) {
                    dz_dzeta(0,0) = x1;
                    dz_dzeta(0,2) = 1;
                    dz_dzeta(0,4) = cam_d(0)*uv_norm(0)*r_2;
                    dz_dzeta(0,5) = cam_d(0)*uv_norm(0)*r_4;
                    dz_dzeta(0,6) = 2*cam_d(0)*uv_norm(0)*uv_norm(1);
                    dz_dzeta(0,7) = cam_d(0)*(r_2+2*uv_norm(0)*uv_norm(0));
                    dz_dzeta(1,1) = y1;
                    dz_dzeta(1,3) = 1;
                    dz_dzeta(1,4) = cam_d(1)*uv_norm(1)*r_2;
                    dz_dzeta(1,5) = cam_d(1)*uv_norm(1)*r_4;
                    dz_dzeta(1,6) = cam_d(1)*(r_2+2*uv_norm(1)*uv_norm(1));
                    dz_dzeta(1,7) = 2*cam_d(1)*uv_norm(0)*uv_norm(1);
                }

            }

            // Our residual
            Eigen::Matrix<double,2,1> uv_m;
            uv_m << (double)feature.uvs[pair.first].at(m)(0), (double)feature.uvs[pair.first].at(m)(1);
            res.block(2*c,0,2,1) = uv_m - uv_dist;


            //=========================================================================
            //=========================================================================


            // Normalized coordinates in respect to projection function
            Eigen::Matrix<double,2,3> dzn_dpfc = Eigen::Matrix<double,2,3>::Zero();
            dzn_dpfc << 1/p_FinCi(2),0,-p_FinCi(0)/(p_FinCi(2)*p_FinCi(2)),
                    0, 1/p_FinCi(2),-p_FinCi(1)/(p_FinCi(2)*p_FinCi(2));

            // Derivative of p_FinCi in respect to p_FinIi
            Eigen::Matrix<double,3,3> dpfc_dpfg = R_ItoC*R_GtoIi;

            // Derivative of p_FinCi in respect to camera clone state
            Eigen::Matrix<double,3,6> dpfc_dclone = Eigen::Matrix<double,3,6>::Zero();
            dpfc_dclone.block(0,0,3,3).noalias() = R_ItoC*skew_x(p_FinIi);
            dpfc_dclone.block(0,3,3,3) = -dpfc_dpfg;


            //=========================================================================
            //=========================================================================


            // Precompute some matrices
            Eigen::Matrix<double,2,3> dz_dpfc = dz_dzn*dzn_dpfc;
            Eigen::Matrix<double,2,3> dz_dpfg = dz_dpfc*dpfc_dpfg;

            // CHAINRULE: get the total feature Jacobian
            H_f.block(2*c,0,2,3).noalias() = dz_dpfg*dpfg_dlambda;

            // CHAINRULE: get state clone Jacobian
            H_x.block(2*c,map_hx[clone_Ii],2,clone_Ii->size()).noalias() = dz_dpfc*dpfc_dclone;


            // CHAINRULE: loop through all extra states and add their
            // NOTE: we add the Jacobian here as we might be in the anchoring pose for this measurement
            for(size_t i=0; i<dpfg_dx_order.size(); i++) {
                H_x.block(2*c,map_hx[dpfg_dx_order.at(i)],2,dpfg_dx_order.at(i)->size()).noalias() += dz_dpfg*dpfg_dx.at(i);
            }


            //=========================================================================
            //=========================================================================


            // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)
            if(state->options().do_calib_camera_pose) {

                // Calculate the Jacobian
                Eigen::Matrix<double,3,6> dpfc_dcalib = Eigen::Matrix<double,3,6>::Zero();
                dpfc_dcalib.block(0,0,3,3) = skew_x(p_FinCi-p_IinC);
                dpfc_dcalib.block(0,3,3,3) = Eigen::Matrix<double,3,3>::Identity();

                // Chainrule it and add it to the big jacobian
                H_x.block(2*c,map_hx[calibration],2,calibration->size()).noalias() += dz_dpfc*dpfc_dcalib;

            }

            // Derivative of measurement in respect to distortion parameters
            if(state->options().do_calib_camera_intrinsics) {
                H_x.block(2*c,map_hx[distortion],2,distortion->size()) = dz_dzeta;
            }

            // Move the Jacobian and residual index forward
            c++;

        }

    }


}


void UpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

    // Apply the left nullspace of H_f to all variables

    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_f.cols(); ++n) {
        for (int m = (int) H_f.rows() - 1; m > n; m--) {
            // Givens matrix G
            tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
            // Multiply G to the corresponding lines (m-1,m) in each matrix
            // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
            //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
            (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        }
    }

    // The H_f jacobian max rank is 3, thus size of the left nullspace is Hf.rows()-3
    //H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols());
    H_x = H_x.block(H_f.cols(),0,H_x.rows()-H_f.cols(),H_x.cols());
    res = res.block(H_f.cols(),0,res.rows()-H_f.cols(),res.cols());

    // Sanity check
    assert(H_x.rows()==res.rows());
}




void UpdaterHelper::measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {


    // Do measurement compression through givens rotations
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n=0; n<H_x.cols(); ++n) {
        for (int m=(int)H_x.rows()-1; m>n; m--) {
            // Givens matrix G
            tempHo_GR.makeGivens(H_x(m-1,n), H_x(m,n));
            // Multiply G to the corresponding lines (m-1,m) in each matrix
            // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
            //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
            (H_x.block(m-1,n,2,H_x.cols()-n)).applyOnTheLeft(0,1,tempHo_GR.adjoint());
            (res.block(m-1,0,2,1)).applyOnTheLeft(0,1,tempHo_GR.adjoint());
        }
    }

    // Find rank of the system
    int r = 0;
    bool found_rank = false;
    int n = (int)H_x.cols();
    while (!found_rank && r < n+1 && r < H_x.rows()) {
        double eps = H_x.block(r, 0, 1, H_x.cols()).squaredNorm();
        if (eps < 1e-20) {
            found_rank = true;
        } else {
            r++;
        }
    }

    // Construct the smaller jacobian and residual after measurement compression
    H_x.conservativeResize(r, H_x.cols());
    res.conservativeResize(r, res.cols());


}



