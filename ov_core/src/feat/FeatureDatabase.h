/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace ov_core {

class Feature;

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
  std::shared_ptr<Feature> get_feature(size_t id, bool remove = false);

  /**
   * @brief Get a specified feature clone (pointer is thread safe)
   * @param id What feature we want to get
   * @param feat Feature with data in it
   * @return True if the feature was found
   */
  bool get_feature_clone(size_t id, Feature &feat);

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
  void update_feature(size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n);

  /**
   * @brief Get features that do not have newer measurement then the specified time.
   *
   * This function will return all features that do not a measurement at a time greater than the specified time.
   * For example this could be used to get features that have not been successfully tracked into the newest frame.
   * All features returned will not have any measurements occurring at a time greater then the specified.
   */
  std::vector<std::shared_ptr<Feature>> features_not_containing_newer(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief Get features that has measurements older then the specified time.
   *
   * This will collect all features that have measurements occurring before the specified timestamp.
   * For example, we would want to remove all features older then the last clone/state in our sliding window.
   */
  std::vector<std::shared_ptr<Feature>> features_containing_older(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief Get features that has measurements at the specified time.
   *
   * This function will return all features that have the specified time in them.
   * This would be used to get all features that occurred at a specific clone/state.
   */
  std::vector<std::shared_ptr<Feature>> features_containing(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief This function will delete all features that have been used up.
   *
   * If a feature was unable to be used, it will still remain since it will not have a delete flag set
   */
  void cleanup();

  /**
   * @brief This function will delete all feature measurements that are older then the specified timestamp
   */
  void cleanup_measurements(double timestamp);

  /**
   * @brief This function will delete all feature measurements that are at the specified timestamp
   */
  void cleanup_measurements_exact(double timestamp);

  /**
   * @brief Returns the size of the feature database
   */
  size_t size() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup.size();
  }

  /**
   * @brief Returns the internal data (should not normally be used)
   */
  std::unordered_map<size_t, std::shared_ptr<Feature>> get_internal_data() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup;
  }

  /**
   * @brief Gets the oldest time in the database
   */
  double get_oldest_timestamp();

  /**
   * @brief Will update the passed database with this database's latest feature information.
   */
  void append_new_measurements(const std::shared_ptr<FeatureDatabase> &database);

protected:
  /// Mutex lock for our map
  std::mutex mtx;

  /// Our lookup array that allow use to query based on ID
  std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup;
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_DATABASE_H */