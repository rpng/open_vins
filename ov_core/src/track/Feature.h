#ifndef OV_CORE_FEATURE_H
#define OV_CORE_FEATURE_H


#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {

    /**
     * @brief Sparse feature class used to collect measurements
     *
     * This feature class allows for holding of all tracking information for a given feature.
     * Each feature has a unique ID assigned to it, and should have a set of feature tracks alongside it.
     * See the FeatureDatabase class for details on how we load information into this, and how we delete features.
     */
    class Feature {

    public:

        /// Unique ID of this feature
        size_t featid;

        /// If this feature should be deleted
        bool to_delete;

        /// UV coordinates that this feature has been seen from (mapped by camera ID)
        std::unordered_map<size_t, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> uvs;

        /// UV normalized coordinates that this feature has been seen from (mapped by camera ID)
        std::unordered_map<size_t, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> uvs_norm;

        /// Timestamps of each UV measurement (mapped by camera ID)
        std::unordered_map<size_t, std::vector<double>> timestamps;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double anchor_clone_timestamp;

        /// Triangulated position of this feature, in the anchor frame
        Eigen::Vector3d p_FinA;

        /// Triangulated inverse position of this feature, in the anchor frame
        Eigen::Vector3d p_invFinA;

        /// Triangulated inverse position of this feature, in the anchor frame, using MSCKF version of "inverse depth"
        Eigen::Vector3d p_invFinA_MSCKF;

        /// Triangulated position of this feature, in the global frame
        Eigen::Vector3d p_FinG;

        /// Triangulated inverse position of this feature, in the global frame
        Eigen::Vector3d p_invFinG;

        /// Helper function that sets the anchor representation from the general xyz representation
        void set_anchor_from_xyz(Eigen::Vector3d new_p_FinA) {

            // Save the global representation
            p_FinA = new_p_FinA;

            // MSCKF representation
            p_invFinA_MSCKF(0) = p_FinA(0)/p_FinA(2);
            p_invFinA_MSCKF(1) = p_FinA(1)/p_FinA(2);
            p_invFinA_MSCKF(2) = 1/p_FinA(2);

            // Feature inverse representation
            // NOTE: This is not the MSCKF inverse form, but the standard form
            // NOTE: Thus we go from p_FinA and convert it to this form
            double a_rho = 1/p_FinA.norm();
            double a_phi = std::acos(a_rho*p_FinA(2));
            double a_theta = std::atan2(p_FinA(1),p_FinA(0));
            p_invFinA(0) = a_theta;
            p_invFinA(1) = a_phi;
            p_invFinA(2) = a_rho;

            // Make sure our inverse is near our real one
            Eigen::Vector3d p_FinA_frominverse;
            p_FinA_frominverse << 1/p_invFinA(2)*std::cos(p_invFinA(0))*std::sin(p_invFinA(1)),
                    1/p_invFinA(2)*std::sin(p_invFinA(0))*std::sin(p_invFinA(1)),
                    1/p_invFinA(2)*std::cos(p_invFinA(1));
            assert((p_FinA-p_FinA_frominverse).norm() < 1e-6);

        }

        /// Helper function that sets the global representation from the general xyz representation
        void set_global_from_xyz(Eigen::Vector3d new_p_FinG) {

            // Save the global representation
            p_FinG = new_p_FinG;

            // Feature inverse representation
            // NOTE: This is not the MSCKF inverse form, but the standard form
            // NOTE: Thus we go from p_FinG and convert it to this form
            double g_rho = 1/p_FinG.norm();
            double g_phi = std::acos(g_rho*p_FinG(2));
            //double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
            double g_theta = std::atan2(p_FinG(1),p_FinG(0));
            p_invFinG(0) = g_theta;
            p_invFinG(1) = g_phi;
            p_invFinG(2) = g_rho;

            // Make sure our inverse is near our real one
            Eigen::Vector3d p_FinG_frominverse;
            p_FinG_frominverse << 1/p_invFinG(2)*std::cos(p_invFinG(0))*std::sin(p_invFinG(1)),
                    1/p_invFinG(2)*std::sin(p_invFinG(0))*std::sin(p_invFinG(1)),
                    1/p_invFinG(2)*std::cos(p_invFinG(1));
            assert((p_FinG-p_FinG_frominverse).norm() < 1e-6);

        }

    };

}


#endif /* OV_CORE_FEATURE_H */