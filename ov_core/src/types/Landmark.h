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
#ifndef OV_CORE_TYPE_LANDMARK_H
#define OV_CORE_TYPE_LANDMARK_H


#include "Vec.h"
#include "feat/FeatureRepresentation.h"


namespace ov_core {


    /**
     * @brief Type that implements a persistant SLAM feature.
     *
     * We store the feature ID that should match the IDs in the trackers.
     * Additionally if this is an anchored representation we store what clone timestamp this is anchored from and what camera.
     * If this features should be marginalized its flag can be set and during cleanup it will be removed.
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
        FeatureRepresentation::Representation _feat_representation;

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
            //if(FeatureRepresentation::is_relative_representation(_feat_representation) && !has_had_anchor_change) {
            //if(FeatureRepresentation::is_relative_representation(_feat_representation)) {
            //    set_fej(value());
            //}
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



#endif //OV_CORE_TYPE_LANDMARK_H
