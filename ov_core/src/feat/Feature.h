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

        // UV normalized coordinates that this feature has been seen from (mapped by camera ID)
        std::unordered_map<size_t, std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>> uvs_norm;

        /// Timestamps of each UV measurement (mapped by camera ID)
        std::unordered_map<size_t, std::vector<double>> timestamps;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double anchor_clone_timestamp;

        /// Triangulated position of this feature, in the anchor frame
        Eigen::Vector3d p_FinA;

        /// Triangulated position of this feature, in the global frame
        Eigen::Vector3d p_FinG;


        /**
         * @brief Remove measurements that do not occur at passed timestamps
         *
         * Given a series of valid timestamps, this will remove all measurements that have not occurred at these times.
         * This would normally be used to ensure that the measurements that we have occur at our clone times.
         *
         * @param valid_times Vector of timestamps that our measurements must occur at
         */
        void clean_old_measurements(std::vector<double> valid_times);

    };

}


#endif /* OV_CORE_FEATURE_H */