//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_JPLQUAT_H
#define PROJECT_JPLQUAT_H

#endif //PROJECT_JPLQUAT_H

#include "utils/quat_ops.h"
#include "Type.h"

/**
 * @brief Derived Type class that implements the JPL convention quaternion with left-multiplicative error state
 *
*/

class JPLQuat : public Type{

public:

    /**
    * @brief Quaternion constructor
    *
    */

    JPLQuat() : Type(3){
        Eigen::Matrix<double,4,1> q0;
        q0.setZero();
        q0(3) = 1.0;
        set_value(q0);
        set_fej(q0);
    }

    ~JPLQuat() { }

    /**
    * @brief Implements update operation by left-multiplying the current quaternion with a quaternion built
    * from a small axis-angle perturbation
    * @param dx Axis-angle representation of the perturbing quaternion
    */

    void update(const Eigen::VectorXd dx){

        assert(dx.rows()== _size);

        //Build perturbing quaternion
        Eigen::Matrix<double,4,1> dq;
        dq << .5*dx , 1.0;
        dq= dq/dq.norm();

        //Update estimate and recompute R
        set_value(quat_multiply(dq, _value));

    }

    /**
    * @brief Sets the value of the estimate and recompute the rotation matrix
    * @param new_value New value for the quaternion estimate
    */
    void set_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);

        _value = new_value;

        //compute associated rotation
        _R = quat_2_Rot(new_value);
    }

    /**
    * @brief Sets the fej value and recomputes the fej rotation matrix
    * @param new_value New value for the quaternion estimate
    */
    void set_fej_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);

        _fej = new_value;

        //compute associated rotation
        _Rfej = quat_2_Rot(new_value);
    }

    Type* clone(){
        Type* Clone= new JPLQuat();
        Clone->set_value(value());
        return Clone;
    }

    ///Rotation access
    Eigen::Matrix<double,3,3> Rot() const{
        return _R;
    }

    ///FEJ Rotation access
    Eigen::Matrix<double,3,3> Rot_fej() const{
        return _Rfej;
    }

protected:


    //Stores the rotation
    Eigen::Matrix<double,3,3> _R;

    //Stores the first-estimate rotation
    Eigen::Matrix<double,3,3> _Rfej;

};