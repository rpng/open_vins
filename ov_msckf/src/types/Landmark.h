#ifndef OV_MSCKF_LANDMARK_H
#define OV_MSCKF_LANDMARK_H


#include "Vec.h"
#include "state/StateOptions.h"


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
        double _anchor_clone_timestamp = -1;

        /// Boolean if this landmark has had at least one anchor change
        bool has_had_anchor_change = false;

        /// Boolean if this landmark should be marginalized out
        bool should_marg = false;

        /// What feature representation this feature currently has
        StateOptions::FeatureRepresentation _feat_representation;

        /**
         * @brief Overrides the default vector update rule
         * We want to selectively update the FEJ value if we are using an anchored representation.
         * @param dx Additive error state correction
         */
        void update(const Eigen::VectorXd dx) override {
            // Update estimate
            assert(dx.rows() == _size);
            set_value(_value+dx);
            // If we are using a relative and we have not anchor changed yet, then update linearization / FEJ value
            //if(StateOptions::is_relative_representation(_feat_representation) && !has_had_anchor_change) {
            if(StateOptions::is_relative_representation(_feat_representation)) {
                set_fej(value());
            }
        }

        /**
         * @brief Will return the position of the feature in the global frame of reference.
         * @param getfej Set to true to get the landmark FEJ value
         * @return Position of feature either in global or anchor frame
         */
        Eigen::Matrix<double,3,1> get_xyz(bool getfej);


        /**
         * @brief Will set the current value based on the representation.
         * @param p_FinG Position of the feature either in global or anchor frame
         * @param isfej Set to true to set the landmark FEJ value
         */
        void set_from_xyz(Eigen::Matrix<double,3,1> p_FinG, bool isfej);


    };
}



#endif //OV_MSCKF_LANDMARK_H
