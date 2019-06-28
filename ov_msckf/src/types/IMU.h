#ifndef OV_MSCKF_TYPE_IMU_H
#define OV_MSCKF_TYPE_IMU_H


#include "utils/quat_ops.h"
#include "PoseJPL.h"


using namespace ov_core;

/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Derived Type class that implements an IMU state
     *
     * Contains a PoseJPL, Vec velocity, Vec gyro bias, and Vec accel bias.
     * This should be simular to that of the standard MSCKF state besides the ordering.
     * The pose is first, followed by velocity, etc.
     */
    class IMU : public Type {

    public:

        IMU() : Type(15) {
            _pose = new PoseJPL();
            _v = new Vec(3);
            _bg = new Vec(3);
            _ba = new Vec(3);

            Eigen::Matrix<double, 16, 1> imu0;
            imu0.setZero();
            imu0(3) = 1.0;

            set_value(imu0);
            set_fej(imu0);
        }

        ~IMU() {}

        /**
         * @brief Sets id used to track location of variable in the filter covariance
         *
         * Note that we update the sub-variables also.
         *
         * @param new_id entry in filter covariance corresponding to this variable
         */
        void set_local_id(int new_id) override {
            _id = new_id;
            _pose->set_local_id(new_id);
            _v->set_local_id(_pose->id()+_pose->size());
            _bg->set_local_id(_v->id()+_v->size());
            _ba->set_local_id(_bg->id()+_bg->size());
        }

        /**
        * @brief Performs update operation using JPLQuat update for orientation, then vector updates for
        * position, velocity, gyro bias, and accel bias (in that order).
         *
        * @param dx 15 DOF vector encoding update using the following order (q, p, v, bg, ba)
        */
        void update(const Eigen::VectorXd dx) override {

            assert(dx.rows() == _size);

            Eigen::Matrix<double, 16, 1> newX = _value;

            Eigen::Matrix<double, 4, 1> dq;
            dq << .5 * dx.block(0, 0, 3, 1), 1.0;
            dq = dq / dq.norm();

            newX.block(0, 0, 4, 1) = quat_multiply(dq, quat());
            newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

            newX.block(7, 0, 3, 1) += dx.block(6, 0, 3, 1);
            newX.block(10, 0, 3, 1) += dx.block(9, 0, 3, 1);
            newX.block(13, 0, 3, 1) += dx.block(12, 0, 3, 1);

            set_value(newX);

        }


        /**
         * @brief Sets the value of the estimate
         * @param new_value New value we should set
         */
        void set_value(const Eigen::VectorXd new_value) override {

            _pose->set_value(new_value.block(0, 0, 7, 1));
            _v->set_value(new_value.block(7, 0, 3, 1));
            _bg->set_value(new_value.block(10, 0, 3, 1));
            _ba->set_value(new_value.block(13, 0, 3, 1));

            _value = new_value;
        }

        /**
         * @brief Sets the value of the first estimate
         * @param new_value New value we should set
         */
        void set_fej(const Eigen::VectorXd new_value) override {

            _pose->set_fej(new_value.block(0, 0, 7, 1));
            _v->set_fej(new_value.block(7, 0, 3, 1));
            _bg->set_fej(new_value.block(10, 0, 3, 1));
            _ba->set_fej(new_value.block(13, 0, 3, 1));

            _fej = new_value;
        }

        Type *clone() override {
            Type *Clone = new IMU();
            Clone->set_value(value());
            Clone->set_fej(fej());
            return Clone;
        }

        /**
         * @brief Used to find the components inside the IMU if needed.
         * If the passed variable is a sub-variable or the current variable this will return it.
         * Otherwise it will return a nullptr, meaning that it was unable to be found.
         *
         * @param check variable to find
         */
        Type *check_if_same_variable(const Type *check) override {
            if (check == this) {
                return this;
            } else if (check == _pose) {
                return _pose;
            } else if (check == q()) {
                return q();
            } else if (check == p()) {
                return p();
            } else if (check == _v) {
                return _v;
            } else if (check == _bg) {
                return bg();
            } else if (check == _ba) {
                return ba();
            } else {
                return nullptr;
            }
        }

        /// Rotation access
        Eigen::Matrix<double, 3, 3> Rot() const {
            return _pose->Rot();
        }

        /// FEJ Rotation access
        Eigen::Matrix<double, 3, 3> Rot_fej() const {
            return _pose->Rot_fej();
        }

        /// Rotation access quaternion
        Eigen::Matrix<double, 4, 1> quat() const {
            return _pose->quat();
        }

        /// FEJ Rotation access quaternion
        Eigen::Matrix<double, 4, 1> quat_fej() const {
            return _pose->quat_fej();
        }


        /// Position access
        Eigen::Matrix<double, 3, 1> pos() const {
            return _pose->pos();
        }

        /// FEJ position access
        Eigen::Matrix<double, 3, 1> pos_fej() const {
            return _pose->pos_fej();
        }

        /// Velocity access
        Eigen::Matrix<double, 3, 1> vel() const {
            return _v->value();
        }

        // FEJ velocity access
        Eigen::Matrix<double, 3, 1> vel_fej() const {
            return _v->fej();
        }

        /// Gyro bias access
        Eigen::Matrix<double, 3, 1> bias_g() const {
            return _bg->value();
        }

        /// FEJ gyro bias access
        Eigen::Matrix<double, 3, 1> bias_g_fej() const {
            return _bg->fej();
        }

        /// Accel bias access
        Eigen::Matrix<double, 3, 1> bias_a() const {
            return _ba->value();
        }

        // FEJ accel bias access
        Eigen::Matrix<double, 3, 1> bias_a_fej() const {
            return _ba->fej();
        }

        /// Pose type access
        PoseJPL *pose() {
            return _pose;
        }

        /// Quaternion type access
        JPLQuat *q() {
            return _pose->q();
        }

        /// Position type access
        Vec *p() {
            return _pose->p();
        }

        /// Velocity type access
        Vec *v() {
            return _v;
        }

        /// Gyroscope bias access
        Vec *bg() {
            return _bg;
        }

        /// Acceleration bias access
        Vec *ba() {
            return _ba;
        }

    protected:

        /// Pose subvariable
        PoseJPL *_pose;

        /// Velocity subvariable
        Vec *_v;

        /// Gyroscope bias subvariable
        Vec *_bg;

        /// Acceleration bias subvariable
        Vec *_ba;

    };

}


#endif //OV_MSCKF_TYPE_IMU_H