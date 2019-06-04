//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_JPLPOSE_H
#define PROJECT_JPLPOSE_H

#include "utils/quat_ops.h"
#include "JPLQuat.h"
#include "Vec.h"


/**
 * @brief Derived Type class that implements a pose (orientation plus position) with a JPL quaternion representation for
 * the orientation
 *
*/

class PoseJPL : public Type{

public:

    PoseJPL() : Type(6){

        //Initialize subvariables
        _q = new JPLQuat();
        _p = new Vec(3);

        Eigen::Matrix<double,7,1> pose0;
        pose0.setZero();
        pose0(3) = 1.0;
        set_value(pose0);
        set_fej(pose0);
    }

    ~PoseJPL() { }

    /**
    *@brief Update q and p using a the JPLQuat update for orientation and vector update for position
    * @param dx Correction vector (orientation then position)
    */
    void update(const Eigen::VectorXd dx){

        assert(dx.rows()== _size);

        Eigen::Matrix<double,7,1> newX = _value;

        Eigen::Matrix<double,4,1> dq;
        dq << .5*dx.block(0,0,3,1) , 1.0;
        dq= dq/dq.norm();

        //Update orientation
        newX.block(0,0,4,1) = quat_multiply(dq, quat());

        //Update position
        newX.block(4,0,3,1) += dx.block(3,0,3,1);

        set_value(newX);

    }

    //Sets the value of the estimate
    void set_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);

        //Set orientation value
        _q->set_value(new_value.block(0,0,4,1));

        //Set position value
        _p->set_value(new_value.block(4,0,3,1));

        _value = new_value;
    }

    //Sets the value of the first estimate
    void set_fej_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);
        //Set orientation fej value
        _q->set_fej(new_value.block(0,0,4,1));

        //Set position fej value
        _p->set_fej(new_value.block(4,0,3,1));

        _fej = new_value;
    }

    Type* clone(){
        Type* Clone= new PoseJPL();
        Clone->set_value(value());
        Clone->set_fej(fej());
        return Clone;
    }

    /// Rotation access
    Eigen::Matrix<double,3,3> Rot() const{
        return _q->Rot();
    }

    /// FEJ Rotation access
    Eigen::Matrix<double,3,3> Rot_fej() const{
        return _q->Rot_fej();;
    }

    /// Rotation access as quaternion
    Eigen::Matrix<double,4,1> quat() const{
        return _q->value();
    }

    /// FEJ Rotation access as quaternion
    Eigen::Matrix<double,4,1> quat_fej() const{
        return _q->fej();
    }


    /// Position access
    Eigen::Matrix<double,3,1> pos() const{
        return _p->value();
    }

    // FEJ position access
    Eigen::Matrix<double,3,1> pos_fej() const{
        return _p->fej();
    }

    /** @brief Used to find the components inside the Pose if needed
     * @param check variable to find
     */
    Type* check_if_same_variable(Type* check){
        if (check== this){
            return this;
        }
        else if (check == _q){
            return q();
        }
        else if (check == _p){
            return p();
        }
        else{
            return nullptr;
        }
    }

    JPLQuat * q(){
        return _q;
    }

    Vec * p(){
        return _p;
    }

protected:

    ///Subvariable containing orientation
    JPLQuat *_q;

    ///Subvariable containg position
    Vec * _p;

};

#endif //PROJECT_JPLPOSE_H