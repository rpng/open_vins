#ifndef OV_MSCKF_LANDMARK_H
#define OV_MSCKF_LANDMARK_H



#include "Vec.h"
#include "state/State.h"


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Type that implements a persistant SLAM feature
     */
    class Landmark : public Vec {

    public:

        /// Default constructor (feature is always a Vec of size 3)
        Landmark() : Vec(3) {}

        /// Feature ID of this landmark (corresponds to frontend id)
        size_t _featid;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int _anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double _anchor_clone_timestamp;

        /// Boolean if this landmark should be marginalized out
        bool should_marg = false;


        /**
         * @brief Will return the position of the feature in the global frame of reference.
         * @param state State of filter
         * @return Position of feature in global frame
         */
        Eigen::Matrix<double,3,1> get_global_xyz(State *state);


        /**
         * @brief Will return the position of the feature in the anchor frame of reference.
         * @param state State of filter
         * @return Position of feature in anchor frame
         */
        Eigen::Matrix<double,3,1> get_anchor_xyz(State *state);


        /**
         * @brief Will set the current value based on the representation.
         * @param state State of filter
         * @param p_FinG Position of the feature in the global frame
         */
        void set_from_global_xyz(State *state, Eigen::Matrix<double,3,1> p_FinG);


    };
}



#endif //OV_MSCKF_LANDMARK_H
