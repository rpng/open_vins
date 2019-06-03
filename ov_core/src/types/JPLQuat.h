//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_JPLQUAT_H
#define PROJECT_JPLQUAT_H

#endif //PROJECT_JPLQUAT_H

#include "utils/quat_ops.h"
#include "Type.h"

class JPLQuat : public Type{

public:

    JPLQuat() : Type(3){}

    ~JPLQuat() { }

    void update(const Eigen::VectorXd dx){

        assert(dx.rows()== _size);

        Eigen::Matrix<double,4,1> dq;
        dq << .5*dx , 1.0;
        dq= dq/dq.norm();

        set_value(quat_multiply(dq, _value));

    }

    //Sets the value of the estimate
    void set_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);

        _value = new_value;

        //compute associated rotation
        _R = quat_2_Rot(new_value);
    }

    //Sets the value of the estimate
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

    //Rotation access
    Eigen::Matrix<double,3,3> Rot() const{
        return _R;
    }

    //FEJ Rotation access
    Eigen::Matrix<double,3,3> Rot_fej() const{
        return _Rfej;
    }

protected:


    //Stores the rotation
    Eigen::Matrix<double,3,3> _R;

    //Stores the first-estimate rotation
    Eigen::Matrix<double,3,3> _Rfej;

};