#ifndef OV_CORE_INITIALIZEROPTIONS_H
#define OV_CORE_INITIALIZEROPTIONS_H


/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {


    /**
     * @brief Struct which stores all our feature initializer options
     */
    struct FeatureInitializerOptions {

        /// Max runs for Gauss Newton
        int max_runs = 20;

        /// Init lambda for LM optimization
        double init_lamda = 1e-3;

        /// Max lambda for LM optimization
        double max_lamda = 1e10;

        /// Cutoff for dx increment to consider as converged
        double min_dx = 1e-6;

        /// Cutoff for cost decrement to consider as converged
        double min_dcost = 1e-6;

        /// Multiplier to increase/decrease lambda
        double lam_mult = 10;

        /// Minimum distance to accept triangulated features
        double min_dist = .25;

        /// Minimum distance to accept triangulated features
        double max_dist = 40;

        /// Max baseline ratio to accept triangulated features
        double max_baseline = 40;

        /// Max condition number of linear triangulation matrix accept triangulated features
        double max_cond_number = 1000;


    };

}

#endif //OV_CORE_INITIALIZEROPTIONS_H
