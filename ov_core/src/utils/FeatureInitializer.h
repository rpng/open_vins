//
// Created by keck on 6/5/19.
//

#ifndef OPEN_VINS_FEATUREINITIALIZER_H
#define OPEN_VINS_FEATUREINITIALIZER_H

#include "quat_ops.h"
#include "track/Feature.h"
#include "InitializerOptions.h"
#include <unordered_map>

namespace ov_core {


    /**@brief Structure which stores pose estimates for use in triangulation
     *
     */
    struct clonePose{

        /// Rotation
        Eigen::Matrix<double,3,3> _Rot;

        /// Position
        Eigen::Matrix<double,3,1> _pos;

        /// @brief Constructs pose from rotation and position
        clonePose(Eigen::Matrix<double,3,3> R,Eigen::Matrix<double,3,1> p ){
            _Rot = R;
            _pos = p;
        }

        /// @brief Constructs pose from quaternion and position
        clonePose(Eigen::Matrix<double,4,1> q,Eigen::Matrix<double,3,1> p ){
            _Rot = quat_2_Rot(q);
            _pos = p;
        }

        /// @brief default constructor
        clonePose(){
            _Rot = Eigen::Matrix<double,3,3>::Identity();
            _pos = Eigen::Matrix<double,3,1>::Zero();
        }

        Eigen::Matrix<double,3,3> &Rot(){
            return _Rot;
        };
        Eigen::Matrix<double,3,1> &pos(){
            return _pos;
        };


    };

    /**@brief Class that triangulates feature
     *
     */
    class FeatureInitializer
    {

    public:

        /** @brief Default constructor
        *   @param options Options for the initializer
        */
        FeatureInitializer(InitializerOptions options) : _options(options) {}

        /**
         * @brief Uses a linear triangulation to get initial estimate for the feature
         * @param feat Pointer to feature
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
         */
        bool single_triangulation(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,clonePose>> &clonesCAM);

        /**
         * @brief Uses a nonlinear triangulation to refine initial linear estimate of the feature
         * @param feat Pointer to feature
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
         */
        bool single_gaussnewton(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,clonePose>> &clonesCAM);

    protected:

        /// Contains options for the initializer process
        InitializerOptions _options;


        /** @brief Helper function for the gauss newton method that computes error of the given estimate
         * @param clonesCAM Map between camera ID to map of timestamp to camera pose estimate
         * @param feat Pointer to the feature
         * @param alpha x/z in anchor
         * @param beta y/z in anchor
         * @param rho inverse depth
         */
        double compute_error(std::unordered_map<size_t,std::unordered_map<double,clonePose>> &clonesCAM,Feature* feat,double alpha,double beta,double rho);

    };



}


#endif //OPEN_VINS_FEATUREINITIALIZER_H
