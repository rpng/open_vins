#ifndef OV_MSCKF_UPDATER_OPTIONS_H
#define OV_MSCKF_UPDATER_OPTIONS_H


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Struct which stores general updater options
     */
    struct UpdaterOptions {

        /// What chi-squared multipler we should apply
        int chi2_multipler = 5;

        /// Noise sigma for our raw pixel measurements
        double sigma_pix;

        /// Covariance for our raw pixel measurements
        double sigma_pix_sq;


    };


}

#endif //OV_MSCKF_UPDATER_OPTIONS_H