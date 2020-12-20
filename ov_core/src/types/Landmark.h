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
#ifndef OV_TYPE_TYPE_LANDMARK_H
#define OV_TYPE_TYPE_LANDMARK_H


#include "Vec.h"
#include "LandmarkRepresentation.h"
#include "utils/colors.h"


namespace ov_type {


    /**
     * @brief Type that implements a persistent SLAM feature.
     *
     * We store the feature ID that should match the IDs in the trackers.
     * Additionally if this is an anchored representation we store what clone timestamp this is anchored from and what camera.
     * If this features should be marginalized its flag can be set and during cleanup it will be removed.
     */
    class Landmark : public Vec {

    public:

        /// Default constructor (feature is a Vec of size 3 or Vec of size 1)
        Landmark(int dim) : Vec(dim) {}

        /// Feature ID of this landmark (corresponds to frontend id)
        size_t _featid;

        /// What unique camera stream this slam feature was observed from
        int _unique_camera_id = -1;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int _anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double _anchor_clone_timestamp = -1;

        /// Boolean if this landmark has had at least one anchor change
        bool has_had_anchor_change = false;

        /// Boolean if this landmark should be marginalized out
        bool should_marg = false;

        /// First normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero;

        /// First estimate normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero_fej;

        /// What feature representation this feature currently has
        LandmarkRepresentation::Representation _feat_representation;

        /**
         * @brief Overrides the default vector update rule
         * We want to selectively update the FEJ value if we are using an anchored representation.
         * @param dx Additive error state correction
         */
        void update(const Eigen::VectorXd& dx) override {
            // Update estimate
            assert(dx.rows() == _size);
            set_value(_value+dx);
            // Ensure we are not near zero in the z-direction
            if (LandmarkRepresentation::is_relative_representation(_feat_representation) && _value(_value.rows()-1) < 1e-8) {
                printf(YELLOW "WARNING DEPTH %.8f BECAME CLOSE TO ZERO IN UPDATE!!!\n" RESET, _value(_value.rows()-1));
                should_marg = true;
            }
        }

        /**
         * @brief Will return the position of the feature in the global frame of reference.
         * @param getfej Set to true to get the landmark FEJ value
         * @return Position of feature either in global or anchor frame
         */
        Eigen::Matrix<double,3,1> get_xyz(bool getfej) const;


        /**
         * @brief Will set the current value based on the representation.
         * @param p_FinG Position of the feature either in global or anchor frame
         * @param isfej Set to true to set the landmark FEJ value
         */
        void set_from_xyz(Eigen::Matrix<double,3,1> p_FinG, bool isfej);


    };
}



#endif //OV_TYPE_TYPE_LANDMARK_H
