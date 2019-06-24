#ifndef OV_MSCKF_UPDATER_HELPER_H
#define OV_MSCKF_UPDATER_HELPER_H


#include <Eigen/Eigen>
#include "feat/Feature.h"
#include "state/State.h"
#include "state/StateOptions.h"
#include "utils/quat_ops.h"


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Class that has helper functions for our updaters.
     *
     * Can compute the Jacobian for a single feature representation.
     * This will create the Jacobian based on what representation our state is in.
     * If we are using the anchor representation then we also have additional Jacobians in respect to the anchor state.
     * Also has functions such as nullspace projection and full jacobian construction.
     */
    class UpdaterHelper {

    public:


        /**
         * @brief Feature object that our UpdaterHelper leverages, has all measurements and means
         */
        struct UpdaterHelperFeature {

            /// Unique ID of this feature
            size_t featid;

            /// UV coordinates that this feature has been seen from (mapped by camera ID)
            std::unordered_map<size_t, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> uvs;

            // UV normalized coordinates that this feature has been seen from (mapped by camera ID)
            std::unordered_map<size_t, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> uvs_norm;

            /// Timestamps of each UV measurement (mapped by camera ID)
            std::unordered_map<size_t, std::vector<double>> timestamps;

            /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
            int anchor_cam_id = -1;

            /// Timestamp of anchor clone
            double anchor_clone_timestamp;

            /// Triangulated position of this feature, in the global frame
            Eigen::Vector3d p_FinG;

            /// Triangulated position of this feature, in the global frame first estimate
            Eigen::Vector3d p_FinG_fej;

        };


        /**
         * @brief This gets the feature and state Jacobian in respect to the feature representation
         *
         * @param[in] state State of the filter system
         * @param[in] feature Feature we want to get Jacobians of (must have feature means)
         * @param[out] H_f Jacobians in respect to the feature error state
         * @param[out] H_x Extra Jacobians in respect to the state (for example anchored pose)
         * @param[out] x_order Extra variables our extra Jacobian has (for example anchored pose)
         */
        static void get_feature_jacobian_representation(State* state, UpdaterHelperFeature &feature, Eigen::Matrix<double,3,3> &H_f,
                                                        std::vector<Eigen::Matrix<double,3,Eigen::Dynamic>> &H_x, std::vector<Type*> &x_order);


        /**
         * @brief Will construct the "stacked" Jacobians for a single feature from all its measurements
         *
         * @param[in] state State of the filter system
         * @param[in] feature Feature we want to get Jacobians of (must have feature means)
         * @param[out] H_f Jacobians in respect to the feature error state
         * @param[out] H_x Extra Jacobians in respect to the state (for example anchored pose)
         * @param[out] res Measurement residual for this feature
         * @param[out] x_order Extra variables our extra Jacobian has (for example anchored pose)
         */
        static void get_feature_jacobian_full(State* state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<Type*> &x_order);


        /**
         * @brief This will project the left nullspace of H_f onto the linear system.
         *
         * Please see the @ref update-null for details on how this works.
         * This is the MSCKF nullspace projection which removes the dependency on the feature state.
         * Note that this is done **in place** so all matrices will be different after a function call.
         *
         * @param H_f Jacobian with nullspace we want to project onto the system [res = Hx*(x-xhat)+Hf(f-fhat)+n]
         * @param H_x State jacobian
         * @param res Measurement residual
         */
        static void nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);


        /**
         * @brief This will perform measurement compression
         *
         * Please see the @ref update-compress for details on how this works.
         * Note that this is done **in place** so all matrices will be different after a function call.
         *
         * @param H_x State jacobian
         * @param res Measurement residual
         */
        static void measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);



    };


}




#endif //OV_MSCKF_UPDATER_HELPER_H