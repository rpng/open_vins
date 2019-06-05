#ifndef OV_MSCKF_TYPE_VEC_H
#define OV_MSCKF_TYPE_VEC_H

#include "Type.h"



/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Derived Type class that implements vector variables
     */
    class Vec : public Type {

    public:

        /**
         * @brief Default constructor for Vec
         * @param dim Size of the vector (will be same as error state)
         */
        Vec(int dim) : Type(dim) {
            _value = Eigen::VectorXd::Zero(dim);
            _fej = Eigen::VectorXd::Zero(dim);
        }

        ~Vec() {}

        /**
         * @brief Implements the update operation through standard vector addition
         * @param dx Additive error state correction
         */
        void update(const Eigen::VectorXd dx) override {
            assert(dx.rows() == _size);
            set_value(_value + dx);
        }

        /**
         * @brief Performs all the cloning
         */
        Type *clone() override {
            Type *Clone = new Vec(_size);
            Clone->set_value(value());
            Clone->set_fej(fej());
            return Clone;
        }


    };

}

#endif //OV_MSCKF_TYPE_VEC_H