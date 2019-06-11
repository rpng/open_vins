//
// Created by keck on 6/11/19.
//

#ifndef OPEN_VINS_LANDMARK_H
#define OPEN_VINS_LANDMARK_H

#include "Vec.h"
#include "track/Feature.h"
#include "state/State.h"

/**@brief Type that implements SLAM features
 *
 */

namespace ov_msckf {

    class Landmark : public Vec {

    public:
        /// Default constructor (feature is always a Vec of size 3)
        Landmark() : Vec(3) {}

        /**
         *
         * @param state Pointer to state
         * @param feature Pointer to track feature object
         */
        void set_from_feature(State *state, Feature *feature);

        /**@brief Helper function that gets the 3D anchor representation from the general xyz representation
         * @param state Pointer to state
         * @param feature Pointer to feature
         */
        void set_feature_from_landmark(State *state, Feature *feature);

        /**
         *
         * @param state State of filter
         * @return Position of feature in global frame
         */
        Eigen::Matrix<double,3,1> get_global_xyz(State *state);

        /// Access ID of camera
        int anchor_cam_id() {
            return _anchor_cam_id;
        }

        /// Access anchor IMU clone timestamp
        double anchor_clone_timestamp() {
            return _anchor_clone_timestamp;
        }

        /// Set ID of camera
        void set_anchor_cam_id(int new_cam_id) {
            _anchor_cam_id = new_cam_id;
        }

        /// Set anchor IMU clone timestamp
        void set_anchor_clone_timestamp(double new_anchor_clone_timestamp) {
            _anchor_clone_timestamp = new_anchor_clone_timestamp;
        }

        size_t featid(){
            return _featid;
        }

        void set_featid(size_t featid_){
            _featid = featid_;
        }

        bool should_marg = false;

    private:

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int _anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double _anchor_clone_timestamp;

        size_t _featid;


    };
}



#endif //OPEN_VINS_LANDMARK_H
