#include "Landmark.h"


using namespace ov_msckf;



Eigen::Matrix<double,3,1> Landmark::get_global_xyz(State *state, bool getfej) {

    /// CASE: Global 3d feature representation
    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        return (getfej)? fej() : value();
    }

    /// CASE: Global inverse depth feature representation
    if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {
        Eigen::Matrix<double, 3, 1> p_invFinG = (getfej)? fej() : value();
        Eigen::Matrix<double, 3, 1> p_FinG;
        p_FinG << (1 / p_invFinG(2)) * std::cos(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::sin(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::cos(p_invFinG(1));
        return p_FinG;
    }

    /// CASE: Anchored feature representation
    Eigen::Matrix<double,3,1> p_FinA = Eigen::Matrix<double,3,1>::Zero();

    /// CASE: Anchored 3D feature representation
    if (state->options().feat_representation == StateOptions::ANCHORED_3D) {
        p_FinA = (getfej)? fej() : value();
    }

    /// CASE: Anchored full inverse depth feature representation
    if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH){
        Eigen::Matrix<double, 3, 1> p_invFinA = (getfej)? fej() : value();
        p_FinA << (1 / p_invFinA(2)) * std::cos(p_invFinA(0)) * std::sin(p_invFinA(1)),
                (1 / p_invFinA(2)) * std::sin(p_invFinA(0)) * std::sin(p_invFinA(1)),
                (1 / p_invFinA(2)) * std::cos(p_invFinA(1));
    }

    /// CASE: Anchored MSCKF inverse depth feature representation
    if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH){
        Eigen::Matrix<double, 3, 1> p_invFinA = (getfej)? fej() : value();
        p_FinA << (1 / p_invFinA(2)) * p_invFinA(0),
                (1 / p_invFinA(2)) * p_invFinA(1),
                1 / p_invFinA(2);
    }

    // Get anchor camera calib
    //Eigen::Matrix<double, 3, 3> R_ItoC = state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
    //Eigen::Matrix<double, 3, 1> p_IinC = state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();
    Eigen::Matrix<double, 3, 3> R_ItoC = (getfej)? state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot_fej() : state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinC = (getfej)? state->get_calib_IMUtoCAM(_anchor_cam_id)->pos_fej() : state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();

    // Get anchor IMU calib
    Eigen::Matrix<double, 3, 3> R_GtoI = (getfej)? state->get_clone(_anchor_clone_timestamp)->Rot_fej() : state->get_clone(_anchor_clone_timestamp)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinG = (getfej)? state->get_clone(_anchor_clone_timestamp)->pos_fej() : state->get_clone(_anchor_clone_timestamp)->pos();

    // Compute global xyz
    Eigen::Matrix<double, 3, 1> p_FinG = R_GtoI.transpose() * R_ItoC.transpose()*(p_FinA - p_IinC) + p_IinG;
    return p_FinG;

}


void Landmark::set_from_global_xyz(State *state, Eigen::Matrix<double,3,1> p_FinG, bool isfej) {

    /// CASE: Global 3d feature representation
    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        if(isfej) set_fej(p_FinG);
        else set_value(p_FinG);
        return;
    }

    /// CASE: Global inverse depth feature representation
    if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {

        // Feature inverse representation
        // NOTE: This is not the MSCKF inverse form, but the standard form
        // NOTE: Thus we go from p_FinG and convert it to this form
        double g_rho = 1/p_FinG.norm();
        double g_phi = std::acos(g_rho*p_FinG(2));
        //double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
        double g_theta = std::atan2(p_FinG(1),p_FinG(0));
        Eigen::Matrix<double,3,1> p_invFinG;
        p_invFinG(0) = g_theta;
        p_invFinG(1) = g_phi;
        p_invFinG(2) = g_rho;

        // Set our feature value
        if(isfej) set_fej(p_invFinG);
        else set_value(p_invFinG);
        return;
    }

    // Get anchor camera calib
    //Eigen::Matrix<double, 3, 3> R_ItoC = state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
    //Eigen::Matrix<double, 3, 1> p_IinC = state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();
    Eigen::Matrix<double, 3, 3> R_ItoC = (isfej)? state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot_fej() : state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinC = (isfej)? state->get_calib_IMUtoCAM(_anchor_cam_id)->pos_fej() : state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();

    //Get anchor IMU calib
    Eigen::Matrix<double, 3, 3> R_GtoI = (isfej)? state->get_clone(_anchor_clone_timestamp)->Rot_fej() : state->get_clone(_anchor_clone_timestamp)->Rot();
    Eigen::Matrix<double, 3, 1> p_IinG = (isfej)? state->get_clone(_anchor_clone_timestamp)->pos_fej() : state->get_clone(_anchor_clone_timestamp)->pos();

    // Compute anchor xyz
    Eigen::Matrix<double,3,1> p_FinA = R_ItoC*R_GtoI*(p_FinG-p_IinG)+p_IinC;

    /// CASE: Anchored 3d feature representation
    if (state->options().feat_representation == StateOptions::ANCHORED_3D) {
        if(isfej) set_fej(p_FinA);
        else set_value(p_FinA);
        return;
    }

    /// CASE: Anchored inverse depth feature representation
    if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH) {

        // Feature inverse representation
        // NOTE: This is not the MSCKF inverse form, but the standard form
        // NOTE: Thus we go from p_FinA and convert it to this form
        double a_rho = 1/p_FinA.norm();
        double a_phi = std::acos(a_rho*p_FinA(2));
        double a_theta = std::atan2(p_FinA(1),p_FinA(0));
        Eigen::Matrix<double,3,1> p_invFinA;
        p_invFinA(0) = a_theta;
        p_invFinA(1) = a_phi;
        p_invFinA(2) = a_rho;

        // Set our feature value
        if(isfej) set_fej(p_invFinA);
        else set_value(p_invFinA);
        return;
    }

    /// CASE: MSCKF anchored inverse depth representation
    if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {

        // MSCKF representation
        Eigen::Matrix<double,3,1> p_invFinA_MSCKF;
        p_invFinA_MSCKF(0) = p_FinA(0)/p_FinA(2);
        p_invFinA_MSCKF(1) = p_FinA(1)/p_FinA(2);
        p_invFinA_MSCKF(2) = 1/p_FinA(2);

        // Set our feature value
        if(isfej) set_fej(p_invFinA_MSCKF);
        else set_value(p_invFinA_MSCKF);
        return;
    }

}




