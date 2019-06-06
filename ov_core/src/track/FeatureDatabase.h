#ifndef OV_CORE_FEATURE_DATABASE_H
#define OV_CORE_FEATURE_DATABASE_H


#include <vector>
#include <Eigen/Eigen>

#include "Feature.h"


/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {

    /**
     * @brief Database containing features we are currently tracking.
     *
     * Each visual tracker has this database in it and it contains all features that we are tracking.
     * The trackers will insert information into this database when they get new measurements from doing tracking.
     * A user would then query this database for features that can be used for update and remove them after they have been processed.
     */
    class FeatureDatabase {

    public:

        /**
         * @brief Default constructor
         */
        FeatureDatabase() {
            features_idlookup = std::map<size_t, Feature *>();
        }


        /**
         * @brief Get a specified feature
         * @param id What feature we want to get
         * @return Either a feature object, or null if it is not in the database.
         */
        Feature *get_feature(size_t id) {
            if (features_idlookup.find(id) != features_idlookup.end()) {
                return features_idlookup[id];
            } else {
                return nullptr;
            }
        }


        /**
         * @brief Update a feature object
         * @param id ID of the feature we will update
         * @param timestamp time that this measurement occured at
         * @param cam_id which camera this measurement was from
         * @param u raw u coordinate
         * @param v raw v coordinate
         * @param u_n undistorted/normalized u coordinate
         * @param v_n undistorted/normalized v coordinate\
         *
         * This will update a given feature based on the passed ID it has.
         * It will create a new feature, if it is an ID that we have not seen before.
         */
        void update_feature(size_t id, double timestamp, size_t cam_id,
                            float u, float v, float u_n, float v_n) {

            // Find this feature using the ID lookup
            if (features_idlookup.find(id) != features_idlookup.end()) {
                // Get our feature
                Feature *feat = features_idlookup[id];
                // Append this new information to it!
                feat->uvs[cam_id].emplace_back(Eigen::Vector2f(u, v));
                feat->uvs_norm[cam_id].emplace_back(Eigen::Vector2f(u_n, v_n));
                feat->timestamps[cam_id].emplace_back(timestamp);
                return;
            }

            // Debug info
            //ROS_INFO("featdb - adding new feature %d",(int)id);

            // Else we have not found the feature, so lets make it be a new one!
            Feature *feat = new Feature();
            feat->featid = id;
            feat->uvs[cam_id].emplace_back(Eigen::Vector2f(u, v));
            feat->uvs_norm[cam_id].emplace_back(Eigen::Vector2f(u_n, v_n));
            feat->timestamps[cam_id].emplace_back(timestamp);

            // Append this new feature into our database
            features_idlookup.insert({id, feat});
        }


        /**
         * @brief Get features that do not have newer measurement then the specified time.
         *
         * This function will return all features that do not a measurement at a time greater than the specified time.
         * For example this could be used to get features that have not been successfully tracked into the newest frame.
         * All features returned will not have any measurements occurring at a time greater then the specified.
         */
        std::vector<Feature *> features_not_containing_newer(double timestamp) {

            // Our vector of features that do not have measurements after the specified time
            std::vector<Feature *> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            for (std::map<size_t, Feature *>::iterator it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Loop through each camera
                bool has_newer_measurement = false;
                for (auto const &pair : (*it).second->timestamps) {
                    // If we have a measurement greater-than or equal to the specified, this measurement is find
                    if (!pair.second.empty() && pair.second.at(pair.second.size() - 1) >= timestamp) {
                        has_newer_measurement = true;
                        break;
                    }
                }
                // If it is not being actively tracked, then it is old
                if (!has_newer_measurement) {
                    feats_old.push_back((*it).second);
                }
                it++;
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;

            // Return the old features
            return feats_old;

        }


        /**
         * @brief Get features that has measurements older then the specified time.
         *
         * This will collect all features that have measurements occurring before the specified timestamp.
         * For example, we would want to remove all features older then the last clone/state in our sliding window.
         */
        std::vector<Feature *> features_containing_older(double timestamp) {

            // Our vector of old features
            std::vector<Feature *> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            for (std::map<size_t, Feature *>::iterator it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Loop through each camera
                for (auto const &pair : (*it).second->timestamps) {
                    if (!pair.second.empty() && pair.second.at(0) < timestamp) {
                        feats_old.push_back((*it).second);
                        break;
                    }
                }
                it++;
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;

            // Return the old features
            return feats_old;

        }

        /**
         * @brief Get features that has measurements at the specified time.
         *
         * This function will return all features that have the specified time in them.
         * This would be used to get all features that occurred at a specific clone/state.
         */
        std::vector<Feature *> features_containing(double timestamp) {

            // Our vector of old features
            std::vector<Feature *> feats_has_timestamp;

            // Now lets loop through all features, and just make sure they are not
            for (std::map<size_t, Feature *>::iterator it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Boolean if it has the timestamp
                bool has_timestamp = false;
                for (auto const &pair : (*it).second->timestamps) {
                    // Loop through all timestamps, and see if it has it
                    for (auto &timefeat : pair.second) {
                        if (timefeat == timestamp) {
                            has_timestamp = true;
                            break;
                        }
                    }
                    // Break out if we found a single timestamp that is equal to the specified time
                    if (has_timestamp) {
                        break;
                    }
                }
                // Remove this feature if it contains the specified timestamp
                if (has_timestamp) {
                    feats_has_timestamp.push_back((*it).second);
                }
                it++;
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;
            //std::cout << "return vector = " << feats_has_timestamp.size() << std::endl;

            // Return the features
            return feats_has_timestamp;

        }


        /**
         * @brief Returns measurements that occurred at a given timestep
         * @todo Need to generalize this function so it works for mono+ncam
         *
         * Given a timestamp, this will return all the raw and normalized feature measurements for this frame.
         * This is used to get track information to our structure from motion initializers!
         */
        void get_frame_measurements(double timestamp, size_t cam_id_left, size_t cam_id_right,
                                    std::vector<size_t> &ids,
                                    std::vector<Eigen::Vector2f> &uvs0_n, std::vector<Eigen::Vector2f> &uvs1_n) {

            // Now lets loop through all features, and just make sure they are stereo tracks
            auto it = features_idlookup.begin();
            while (it != features_idlookup.end()) {
                // Our final ids
                bool foundl = false;
                bool foundr = false;
                Eigen::Vector2f uvl, uvr;
                // Loop through all timestamps, and see if it has it
                for (size_t i = 0; i < (*it).second->timestamps[cam_id_left].size(); i++) {
                    if ((*it).second->timestamps[cam_id_left].at(i) == timestamp) {
                        uvl = (*it).second->uvs_norm[cam_id_left].at(i);
                        foundl = true;
                        break;
                    }
                }
                for (size_t i = 0; i < (*it).second->timestamps[cam_id_right].size(); i++) {
                    if ((*it).second->timestamps[cam_id_right].at(i) == timestamp) {
                        uvr = (*it).second->uvs_norm[cam_id_right].at(i);
                        foundr = true;
                        break;
                    }
                }
                // If found in both left and right, then lets add it!
                if (foundl && foundr) {
                    ids.push_back((*it).second->featid);
                    uvs0_n.push_back(uvl);
                    uvs1_n.push_back(uvr);
                }
                it++;
            }

        }


        /**
         * This function will delete all features that have been used up
         * If a feature was unable to be used, it will still remain since it will not have a delete flag set
         */
        void cleanup() {
            // Debug
            //int sizebefore = (int)features_idlookup.size();
            // Loop through all features
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // If delete flag is set, then delete it
                if ((*it).second->to_delete) {
                    delete (*it).second;
                    features_idlookup.erase(it++);
                } else {
                    it++;
                }
            }
            // Debug
            //cout << "feat db = " << sizebefore << " -> " << (int)features_idlookup.size() << endl;
        }


    private:


        // Our lookup array that allow use to query based on ID
        std::map<size_t, Feature *> features_idlookup;


    };


}

#endif /* OV_CORE_FEATURE_DATABASE_H */