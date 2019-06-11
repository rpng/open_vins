//
// Created by keck on 6/11/19.
//
#include "Landmark.h"

using namespace ov_core;
using namespace ov_msckf;

void Landmark::set_from_feature(State *state, Feature *feature) {

    _featid = feature->featid;

    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        /// CASE: Global 3d feature representation
        set_value(feature->p_FinG);
        set_fej(feature->p_FinG);
    } else if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {
        /// CASE: Global inverse depth feature representation
        set_value(feature->p_invFinG);
        set_fej(feature->p_invFinG);
    } else if (state->options().feat_representation == StateOptions::ANCHORED_3D) {
        /// CASE: Anchored 3d feature representation
        set_value(feature->p_FinA);
        set_fej(feature->p_FinA);

        _anchor_clone_timestamp = feature->anchor_clone_timestamp;
        _anchor_cam_id = feature->anchor_cam_id;
    } else if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH) {
        /// CASE: Anchored inverse depth feature representation
        set_value(feature->p_invFinA);
        set_fej(feature->p_invFinA);

        _anchor_clone_timestamp = feature->anchor_clone_timestamp;
        _anchor_cam_id = feature->anchor_cam_id;
    } else if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {
        /// CASE: MSCKF anchored inverse depth representation
        set_value(feature->p_invFinA_MSCKF);
        set_fej(feature->p_invFinA_MSCKF);

        _anchor_clone_timestamp = feature->anchor_clone_timestamp;
        _anchor_cam_id = feature->anchor_cam_id;
    } else {
        assert(false);
    }
}

/**@brief Helper function that gets the 3D anchor representation from the general xyz representation
 * @param state Pointer to state
 * @param feature Pointer to feature
 */
void Landmark::set_feature_from_landmark(State *state, Feature *feature) {
    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        /// CASE: Global 3d feature representation
        feature->set_global_from_xyz(value());
    } else if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {
        /// CASE: Global inverse depth feature representation

        Eigen::Matrix<double, 3, 1> p_invFinG = value();
        Eigen::Matrix<double, 3, 1> p_FinG;
        p_FinG << (1 / p_invFinG(2)) * std::cos(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::sin(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::cos(p_invFinG(1));

        feature->set_global_from_xyz(p_FinG);
    } else if (state->options().feat_representation == StateOptions::ANCHORED_3D ||
            state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH ||
            state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {
        /// CASE: Anchored feature representation

        if (state->options().feat_representation == StateOptions::ANCHORED_3D) {
            /// CASE: Anchored 3D feature representation
            feature->p_FinA = value();
        }
        else if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH){
            /// CASE: Anchored full inverse depth feature representation
            Eigen::Matrix<double, 3, 1> p_invFinA = value();
            Eigen::Matrix<double, 3, 1> p_FinA;
            p_FinA << (1 / p_invFinA(2)) * std::cos(p_invFinA(0)) * std::sin(p_invFinA(1)),
                    (1 / p_invFinA(2)) * std::sin(p_invFinA(0)) * std::sin(p_invFinA(1)),
                    (1 / p_invFinA(2)) * std::cos(p_invFinA(1));

            feature->p_FinA = p_FinA;
        }
        else if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH){

            /// CASE: Anchored MSCKF inverse depth feature representation
            Eigen::Matrix<double, 3, 1> p_invFinA = value();
            Eigen::Matrix<double, 3, 1> p_FinA;
            p_FinA << (1 / p_invFinA(2)) * p_invFinA(0),
                    (1 / p_invFinA(2)) * p_invFinA(1),
                    1 / p_invFinA(2);

            feature->p_FinA = p_FinA;
        }
        else{
            // THESE ARE THE ONLY SUPPORTED ANCHORED CASES
            assert(false);
        }

        feature->set_anchor_from_xyz(feature->p_FinA);

        //Get anchor camera calib
        Eigen::Matrix<double, 3, 3> R_ItoC = state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
        Eigen::Matrix<double, 3, 1> p_IinC = state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();

        //Get anchor IMU calib
        Eigen::Matrix<double, 3, 3> R_GtoI = state->get_clone(_anchor_clone_timestamp)->Rot();
        Eigen::Matrix<double, 3, 1> p_IinG = state->get_clone(_anchor_clone_timestamp)->pos();

        //Compute global xyz
        Eigen::Matrix<double, 3, 1> p_FinG =
                R_GtoI.transpose() * R_ItoC.transpose()*(feature->p_FinA - p_IinC) + p_IinG;
        feature->set_global_from_xyz(p_FinG);

        feature->anchor_clone_timestamp = _anchor_clone_timestamp;
        feature->anchor_cam_id = _anchor_cam_id;
    }
    else{
        // THESE ARE THE ONLY SUPPORTED CASES
        assert(false);
    }

}

Eigen::Matrix<double,3,1> Landmark::get_global_xyz(State *state) {
    if (state->options().feat_representation == StateOptions::GLOBAL_3D) {
        /// CASE: Global 3d feature representation
        return value();
    } else if (state->options().feat_representation == StateOptions::GLOBAL_FULL_INVERSE_DEPTH) {
        /// CASE: Global inverse depth feature representation

        Eigen::Matrix<double, 3, 1> p_invFinG = value();
        Eigen::Matrix<double, 3, 1> p_FinG;
        p_FinG << (1 / p_invFinG(2)) * std::cos(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::sin(p_invFinG(0)) * std::sin(p_invFinG(1)),
                (1 / p_invFinG(2)) * std::cos(p_invFinG(1));

        return p_FinG;
    } else if (state->options().feat_representation == StateOptions::ANCHORED_3D ||
               state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH ||
               state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH) {
        /// CASE: Anchored feature representation

        Eigen::Matrix<double,3,1> p_FinA;

        if (state->options().feat_representation == StateOptions::ANCHORED_3D) {
            /// CASE: Anchored 3D feature representation
            p_FinA = value();
        }
        else if (state->options().feat_representation == StateOptions::ANCHORED_FULL_INVERSE_DEPTH){
            /// CASE: Anchored full inverse depth feature representation
            Eigen::Matrix<double, 3, 1> p_invFinA = value();
            p_FinA << (1 / p_invFinA(2)) * std::cos(p_invFinA(0)) * std::sin(p_invFinA(1)),
                    (1 / p_invFinA(2)) * std::sin(p_invFinA(0)) * std::sin(p_invFinA(1)),
                    (1 / p_invFinA(2)) * std::cos(p_invFinA(1));
        }
        else if (state->options().feat_representation == StateOptions::ANCHORED_MSCKF_INVERSE_DEPTH){

            /// CASE: Anchored MSCKF inverse depth feature representation
            Eigen::Matrix<double, 3, 1> p_invFinA = value();
            p_FinA << (1 / p_invFinA(2)) * p_invFinA(0),
                    (1 / p_invFinA(2)) * p_invFinA(1),
                    1 / p_invFinA(2);
        }
        else{
            // THESE ARE THE ONLY SUPPORTED ANCHORED CASES
            assert(false);
        }

        //Get anchor camera calib
        Eigen::Matrix<double, 3, 3> R_ItoC = state->get_calib_IMUtoCAM(_anchor_cam_id)->Rot();
        Eigen::Matrix<double, 3, 1> p_IinC = state->get_calib_IMUtoCAM(_anchor_cam_id)->pos();

        //Get anchor IMU calib
        Eigen::Matrix<double, 3, 3> R_GtoI = state->get_clone(_anchor_clone_timestamp)->Rot();
        Eigen::Matrix<double, 3, 1> p_IinG = state->get_clone(_anchor_clone_timestamp)->pos();

        //Compute global xyz
        Eigen::Matrix<double, 3, 1> p_FinG =
                R_GtoI.transpose() * R_ItoC.transpose()*(p_FinA - p_IinC) + p_IinG;

        return p_FinG;
    }
    else{
        // THESE ARE THE ONLY SUPPORTED CASES
        assert(false);
    }

}