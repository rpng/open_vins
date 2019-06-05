#ifndef OV_MSCKF_TYPE_POSEJPL_H
#define OV_MSCKF_TYPE_POSEJPL_H

#include "utils/quat_ops.h"
#include "JPLQuat.h"
#include "Vec.h"


using namespace ov_core;


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


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

        ~PoseJPL() {}

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
            dq = dq / dq.norm();

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
        void set_value(const Eigen::VectorXd new_value) override {

            assert(new_value.rows() == 7);

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
        void set_fej(const Eigen::VectorXd new_value) override {

            assert(new_value.rows() == 7);
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

#endif //OV_MSCKF_TYPE_POSEJPL_H