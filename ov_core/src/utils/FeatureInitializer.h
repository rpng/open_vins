#ifndef OPEN_VINS_FEATUREINITIALIZER_H
#define OPEN_VINS_FEATUREINITIALIZER_H

#include "quat_ops.h"
#include "track/Feature.h"
#include "FeatureInitializerOptions.h"
#include <unordered_map>

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {

    /**
     * @brief Class that triangulates feature
     *
     * This class has the functions needed to triangulate and then refine a given 3D feature.
     * As in the standard MSCKF, we know the clones of the camera from propagation and past updates.
     * Thus, we just need to triangulate a feature in 3D with the known poses and then refine it.
     * One should first call the single_triangulation() function afterwhich single_gaussnewton() allows for refinement.
     */
    class FeatureInitializer
    {

    public:

        /**
         * @brief Structure which stores pose estimates for use in triangulation
         */
        struct ClonePose {

            /// Rotation
            Eigen::Matrix<double,3,3> _Rot;

            /// Position
            Eigen::Matrix<double,3,1> _pos;

            /// @brief Constructs pose from rotation and position
            ClonePose(Eigen::Matrix<double,3,3> R, Eigen::Matrix<double,3,1> p) {
                _Rot = R;
                _pos = p;
            }

            /// @brief Constructs pose from quaternion and position
            ClonePose(Eigen::Matrix<double,4,1> q, Eigen::Matrix<double,3,1> p) {
                _Rot = quat_2_Rot(q);
                _pos = p;
            }

            /// @brief default constructor
            ClonePose() {
                _Rot = Eigen::Matrix<double,3,3>::Identity();
                _pos = Eigen::Matrix<double,3,1>::Zero();
            }

            /// Accessor for rotation
            Eigen::Matrix<double,3,3> &Rot() {
                return _Rot;
            }

            /// Accessor for position
            Eigen::Matrix<double,3,1> &pos() {
                return _pos;
            }

        };


        /**
         * @brief Default constructor
         * @param options Options for the initializer
         */
        FeatureInitializer(FeatureInitializerOptions options) : _options(options) {}

        /**
         * @brief Uses a linear triangulation to get initial estimate for the feature
         * @param feat Pointer to feature
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera in global frame)
         * @return Returns false if it fails to triangulate (based on the thresholds)
         */
        bool single_triangulation(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM);

        /**
         * @brief Uses a nonlinear triangulation to refine initial linear estimate of the feature
         * @param feat Pointer to feature
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate (rotation from global to camera, position of camera in global frame)
         * @return Returns false if it fails to be optimize (based on the thresholds)
         */
        bool single_gaussnewton(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM);


    protected:

        /// Contains options for the initializer process
        FeatureInitializerOptions _options;


        /**
         * @brief Helper function for the gauss newton method that computes error of the given estimate
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
         * @param feat Pointer to the feature
         * @param alpha x/z in anchor
         * @param beta y/z in anchor
         * @param rho inverse depth
         */
        double compute_error(std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM,Feature* feat,double alpha,double beta,double rho);

    };



}


#endif //OPEN_VINS_FEATUREINITIALIZER_H