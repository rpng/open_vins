/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_CORE_FEATURE_DATABASE_H
#define OV_CORE_FEATURE_DATABASE_H


#include <vector>
#include <mutex>
#include <memory>
#include <Eigen/Eigen>

#include "Feature.h"


namespace ov_core {

    /**
     * @brief Database containing features we are currently tracking.
     *
     * Each visual tracker has this database in it and it contains all features that we are tracking.
     * The trackers will insert information into this database when they get new measurements from doing tracking.
     * A user would then query this database for features that can be used for update and remove them after they have been processed.
     *
     *
     * @m_class{m-note m-warning}
     *
     * @par A Note on Multi-Threading Support
     * There is some support for asynchronous multi-threaded access.
     * Since each feature is a pointer just directly returning and using them is not thread safe.
     * Thus, to be thread safe, use the "remove" flag for each function which will remove it from this feature database.
     * This prevents the trackers from adding new measurements and editing the feature information.
     * For example, if you are asynchronous tracking cameras and you chose to update the state, then remove all features you will use in update.
     * The feature trackers will continue to add features while you update, whose measurements can be used in the next update step!
     *
     */
    class FeatureDatabase {

    public:

        /**
         * @brief Default constructor
         */
        FeatureDatabase() {}

        /**
         * @brief Get a specified feature
         * @param id What feature we want to get
         * @param remove Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
         * @return Either a feature object, or null if it is not in the database.
         */
        std::shared_ptr<Feature> get_feature(size_t id, bool remove=false) {
            std::unique_lock<std::mutex> lck(mtx);
            if (features_idlookup.find(id) != features_idlookup.end()) {
                std::shared_ptr<Feature> temp = features_idlookup.at(id);
                if(remove) features_idlookup.erase(id);
                return temp;
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
         * @param v_n undistorted/normalized v coordinate
         *
         * This will update a given feature based on the passed ID it has.
         * It will create a new feature, if it is an ID that we have not seen before.
         */
        void update_feature(size_t id, double timestamp, size_t cam_id,
                            float u, float v, float u_n, float v_n) {

            // Find this feature using the ID lookup
            std::unique_lock<std::mutex> lck(mtx);
            if (features_idlookup.find(id) != features_idlookup.end()) {
                // Get our feature
                std::shared_ptr<Feature> feat = features_idlookup.at(id);
                // Append this new information to it!
                feat->uvs[cam_id].push_back(Eigen::Vector2f(u, v));
                feat->uvs_norm[cam_id].push_back(Eigen::Vector2f(u_n, v_n));
                feat->timestamps[cam_id].push_back(timestamp);
                return;
            }

            // Debug info
            //ROS_INFO("featdb - adding new feature %d",(int)id);

            // Else we have not found the feature, so lets make it be a new one!
            std::shared_ptr<Feature> feat = std::make_shared<Feature>();
            feat->featid = id;
            feat->uvs[cam_id].push_back(Eigen::Vector2f(u, v));
            feat->uvs_norm[cam_id].push_back(Eigen::Vector2f(u_n, v_n));
            feat->timestamps[cam_id].push_back(timestamp);

            // Append this new feature into our database
            features_idlookup[id] = feat;
        }


        /**
         * @brief Get features that do not have newer measurement then the specified time.
         *
         * This function will return all features that do not a measurement at a time greater than the specified time.
         * For example this could be used to get features that have not been successfully tracked into the newest frame.
         * All features returned will not have any measurements occurring at a time greater then the specified.
         */
        std::vector<std::shared_ptr<Feature>> features_not_containing_newer(double timestamp, bool remove=false, bool skip_deleted=false) {

            // Our vector of features that do not have measurements after the specified time
            std::vector<std::shared_ptr<Feature>> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Skip if already deleted
                if(skip_deleted && (*it).second->to_delete) {
                    it++;
                    continue;
                }
                // Loop through each camera
                bool has_newer_measurement = false;
                for (auto const &pair : (*it).second->timestamps) {
                    // If we have a measurement greater-than or equal to the specified, this measurement is find
                    if (!pair.second.empty() && pair.second.at(pair.second.size()-1) >= timestamp) {
                        has_newer_measurement = true;
                        break;
                    }
                }
                // If it is not being actively tracked, then it is old
                if (!has_newer_measurement) {
                    feats_old.push_back((*it).second);
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
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
        std::vector<std::shared_ptr<Feature>> features_containing_older(double timestamp, bool remove=false, bool skip_deleted=false) {

            // Our vector of old features
            std::vector<std::shared_ptr<Feature>> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Skip if already deleted
                if(skip_deleted && (*it).second->to_delete) {
                    it++;
                    continue;
                }
                // Loop through each camera
                bool found_containing_older = false;
                for (auto const &pair : (*it).second->timestamps) {
                    if (!pair.second.empty() && pair.second.at(0) < timestamp) {
                        found_containing_older = true;
                        break;
                    }
                }
                // If it has an older timestamp, then add it
                if(found_containing_older) {
                    feats_old.push_back((*it).second);
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
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
        std::vector<std::shared_ptr<Feature>> features_containing(double timestamp, bool remove=false, bool skip_deleted=false) {

            // Our vector of old features
            std::vector<std::shared_ptr<Feature>> feats_has_timestamp;

            // Now lets loop through all features, and just make sure they are not
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Skip if already deleted
                if(skip_deleted && (*it).second->to_delete) {
                    it++;
                    continue;
                }
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
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;
            //std::cout << "return vector = " << feats_has_timestamp.size() << std::endl;

            // Return the features
            return feats_has_timestamp;

        }

        /**
         * @brief This function will delete all features that have been used up.
         *
         * If a feature was unable to be used, it will still remain since it will not have a delete flag set
         */
        void cleanup() {
            // Debug
            //int sizebefore = (int)features_idlookup.size();
            // Loop through all features
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // If delete flag is set, then delete it
                // NOTE: if we are using a shared pointer, then no need to do this!
                if ((*it).second->to_delete) {
                    //delete (*it).second;
                    features_idlookup.erase(it++);
                } else {
                    it++;
                }
            }
            // Debug
            //std::cout << "feat db = " << sizebefore << " -> " << (int)features_idlookup.size() << std::endl;
        }

        /**
         * @brief This function will delete all feature measurements that are older then the specified timestamp
         */
        void cleanup_measurements(double timestamp) {
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Remove the older measurements
                (*it).second->clean_older_measurements(timestamp);
                // Count how many measurements
                int ct_meas = 0;
                for(const auto &pair : (*it).second->timestamps) {
                    ct_meas += (*it).second->timestamps.at(pair.first).size();
                }
                // If delete flag is set, then delete it
                // NOTE: if we are using a shared pointer, then no need to do this!
                if (ct_meas < 1) {
                    //delete (*it).second;
                    features_idlookup.erase(it++);
                } else {
                    it++;
                }
            }
        }


        /**
         * @brief Returns the size of the feature database
         */
        size_t size() {
            std::unique_lock<std::mutex> lck(mtx);
            return features_idlookup.size();
        }


        /**
         * @brief Returns the internal data (should not normally be used)
         */
        std::unordered_map<size_t, std::shared_ptr<Feature>> get_internal_data() {
            std::unique_lock<std::mutex> lck(mtx);
            return features_idlookup;
        }


        /**
         * @brief Will update the passed database with this database's latest feature information.
         */
        void append_new_measurements(const std::shared_ptr<FeatureDatabase>& database) {
            std::unique_lock<std::mutex> lck(mtx);

            // Loop through the other database's internal database
            //int sizebefore = (int)features_idlookup.size();
            for(const auto &feat : database->get_internal_data()) {
                if (features_idlookup.find(feat.first) != features_idlookup.end()) {

                    // For this feature, now try to append the new measurement data
                    std::shared_ptr<Feature> temp = features_idlookup.at(feat.first);
                    for(const auto &times : feat.second->timestamps) {
                        // Append the whole camera vector is not seen
                        // Otherwise need to loop through each and append
                        size_t cam_id = times.first;
                        if(temp->timestamps.find(cam_id)==temp->timestamps.end()) {
                            temp->timestamps[cam_id] = feat.second->timestamps.at(cam_id);
                            temp->uvs[cam_id] = feat.second->uvs.at(cam_id);
                            temp->uvs_norm[cam_id] = feat.second->uvs_norm.at(cam_id);
                        } else {
                            auto temp_times = temp->timestamps.at(cam_id);
                            for(size_t i=0; i<feat.second->timestamps.at(cam_id).size(); i++) {
                                double time_to_find = feat.second->timestamps.at(cam_id).at(i);
                                if(std::find(temp_times.begin(),temp_times.end(),time_to_find)==temp_times.end()) {
                                    temp->timestamps.at(cam_id).push_back(feat.second->timestamps.at(cam_id).at(i));
                                    temp->uvs.at(cam_id).push_back(feat.second->uvs.at(cam_id).at(i));
                                    temp->uvs_norm.at(cam_id).push_back(feat.second->uvs_norm.at(cam_id).at(i));
                                }
                            }
                        }
                    }

                } else {

                    // Else we have not found the feature, so lets make it be a new one!
                    std::shared_ptr<Feature> temp = std::make_shared<Feature>();
                    temp->featid = feat.second->featid;
                    temp->timestamps = feat.second->timestamps;
                    temp->uvs = feat.second->uvs;
                    temp->uvs_norm = feat.second->uvs_norm;
                    features_idlookup[feat.first] = temp;

                }
            }
            //std::cout << "feat db = " << sizebefore << " -> " << (int)features_idlookup.size() << std::endl;

        }

    protected:

        /// Mutex lock for our map
        std::mutex mtx;

        /// Our lookup array that allow use to query based on ID
        std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup;


    };


}

#endif /* OV_CORE_FEATURE_DATABASE_H */