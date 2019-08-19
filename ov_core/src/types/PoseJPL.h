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
#ifndef OV_CORE_TYPE_POSEJPL_H
#define OV_CORE_TYPE_POSEJPL_H

#include "Vec.h"
#include "JPLQuat.h"
#include "utils/quat_ops.h"


namespace ov_core {


    /**
     * @brief Derived Type class that implements a 6 d.o.f pose
     *
     * Internally we use a JPLQuat quaternion representation for the orientation and 3D Vec position.
     * Please see JPLQuat for details on its update procedure and its left multiplicative error.
     */
    class PoseJPL : public Type {

    public:

        PoseJPL() : Type(6) {

            //Initialize subvariables
            _q = new JPLQuat();
            _p = new Vec(3);

            Eigen::Matrix<double, 7, 1> pose0;
            pose0.setZero();
            pose0(3) = 1.0;
            set_value(pose0);
            set_fej(pose0);
        }

        ~PoseJPL() {
            delete _q;
            delete _p;
        }

        /**
         * @brief Sets id used to track location of variable in the filter covariance
         *
         * Note that we update the sub-variables also.
         *
         * @param new_id entry in filter covariance corresponding to this variable
         */
        void set_local_id(int new_id) override {
            _id = new_id;
            _q->set_local_id(new_id);
            _p->set_local_id(new_id+_q->size());
        }

        /**
         * @brief Update q and p using a the JPLQuat update for orientation and vector update for position
         *
         * @param dx Correction vector (orientation then position)
         */
        void update(const Eigen::VectorXd dx) override {

            assert(dx.rows() == _size);

            Eigen::Matrix<double, 7, 1> newX = _value;

            Eigen::Matrix<double, 4, 1> dq;
            dq << .5 * dx.block(0, 0, 3, 1), 1.0;
            dq = quatnorm(dq);

            //Update orientation
            newX.block(0, 0, 4, 1) = quat_multiply(dq, quat());

            //Update position
            newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

            set_value(newX);

        }

        /**
         * @brief Sets the value of the estimate
         * @param new_value New value we should set
         */
        void set_value(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 7);
            assert(new_value.cols() == 1);

            //Set orientation value
            _q->set_value(new_value.block(0, 0, 4, 1));

            //Set position value
            _p->set_value(new_value.block(4, 0, 3, 1));

            _value = new_value;
        }

        /**
         * @brief Sets the value of the first estimate
         * @param new_value New value we should set
         */
        void set_fej(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 7);
            assert(new_value.cols() == 1);

            //Set orientation fej value
            _q->set_fej(new_value.block(0, 0, 4, 1));

            //Set position fej value
            _p->set_fej(new_value.block(4, 0, 3, 1));

            _fej = new_value;
        }

        Type *clone() override {
            Type *Clone = new PoseJPL();
            Clone->set_value(value());
            Clone->set_fej(fej());
            return Clone;
        }

        /**
         * @brief Used to find the components inside the Pose if needed
         *
         * If the passed variable is a sub-variable or the current variable this will return it.
         * Otherwise it will return a nullptr, meaning that it was unable to be found.
         *
         * @param check variable to find
         */
        Type *check_if_same_variable(const Type *check) override {
            if (check == this) {
                return this;
            } else if (check == _q) {
                return q();
            } else if (check == _p) {
                return p();
            } else {
                return nullptr;
            }
        }

        /// Rotation access
        Eigen::Matrix<double, 3, 3> Rot() const {
            return _q->Rot();
        }

        /// FEJ Rotation access
        Eigen::Matrix<double, 3, 3> Rot_fej() const {
            return _q->Rot_fej();;
        }

        /// Rotation access as quaternion
        Eigen::Matrix<double, 4, 1> quat() const {
            return _q->value();
        }

        /// FEJ Rotation access as quaternion
        Eigen::Matrix<double, 4, 1> quat_fej() const {
            return _q->fej();
        }


        /// Position access
        Eigen::Matrix<double, 3, 1> pos() const {
            return _p->value();
        }

        // FEJ position access
        Eigen::Matrix<double, 3, 1> pos_fej() const {
            return _p->fej();
        }

        // Quaternion type access
        JPLQuat *q() {
            return _q;
        }

        // Position type access
        Vec *p() {
            return _p;
        }

    protected:

        /// Subvariable containing orientation
        JPLQuat *_q;

        /// Subvariable containg position
        Vec *_p;

    };

}

#endif //OV_CORE_TYPE_POSEJPL_H