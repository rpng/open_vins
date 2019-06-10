#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H


#include <Eigen/Eigen>
#include "track/Feature.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/FeatureInitializer.h"
#include "utils/FeatureInitializerOptions.h"
#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {



    /**
     * @brief Will compute the system for our sparse features and update the filter.
     *
     * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
     * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
     * After this we compress all the measurements to have an efficient update and update the state.
     */
    class UpdaterMSCKF {

    public:


        /**
         * @brief Default constructor for our MSCKF updater
         *
         * Our updater has a feature intializer which we use to initialize features as needed.
         * Also the options allow for one to tune the different parameters for update.
         *
         * @param options Updater options (include measurement noise value)
         * @param feat_init_options Feature initializer options
         */
        UpdaterMSCKF(UpdaterOptions options, FeatureInitializerOptions feat_init_options) : _options(options){

            // Save our raw pixel noise squared
            _options.sigma_pix_sq = std::pow(_options.sigma_pix,2);

            // Save our feature initializer
            initializer_feat = new FeatureInitializer(feat_init_options);

            // Initialize the chi squared test table with confidence level 0.95
            // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
            for (int i = 1; i < 500; i++) {
                boost::math::chi_squared chi_squared_dist(i);
                chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
            }

        }



        /**
         * @brief Given tracked features, this will try to use them to update the state.
         *
         * @param state State of the filter
         * @param feature_vec Features that can be used for update
         */
        void update(State *state, std::vector<Feature*>& feature_vec);



    protected:


        /**
         * @brief Given a feature this will remove all measurements that do not have a corresponding state clone.         *
         * @param feature Feature with measurements we want to clean
         */
        void clean_feature(State *state, Feature* feature);



        /// Options used during update
        UpdaterOptions _options;

        /// Feature initializer class object
        FeatureInitializer* initializer_feat;

        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_squared_table;




    };




}




#endif //OV_MSCKF_UPDATER_MSCKF_H


