//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_JPLPOSE_H
#define PROJECT_JPLPOSE_H

#endif //PROJECT_JPLPOSE_H

#include "utils/quat_ops.h"
#include "JPLQuat.h"
#include "Vec.h"

class JPLPose : public Type{

public:

    JPLPose() : Type(6){
        q = new JPLQuat();
        p = new Vec(3);
    }

    ~JPLPose() { }

    void update(const Eigen::VectorXd dx){

        assert(dx.rows()== _size);

        Eigen::Matrix<double,7,1> newX = _value;

        Eigen::Matrix<double,4,1> dq;
        dq << .5*dx.block(0,0,3,1) , 1.0;
        dq= dq/dq.norm();

        newX.block(0,0,4,1) = quat_multiply(dq, quat());
        newX.block(4,0,3,1) += dx.block(3,0,3,1);

        set_value(newX);

    }

    //Sets the value of the estimate
    void set_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);

        q->set_value(new_value.block(0,0,4,1));
        p->set_value(new_value.block(4,0,3,1));

        _value = new_value;
    }

    //Sets the value of the estimate
    void set_fej_value(const Eigen::VectorXd new_value){

        assert(new_value.rows() == 7);
        q->set_fej(new_value.block(0,0,4,1));
        p->set_fej(new_value.block(4,0,3,1));

        _fej = new_value;
    }

    Type* clone(){
        Type* Clone= new JPLPose();
        Clone->set_value(value());
        Clone->set_fej(fej());
        return Clone;
    }

    //Rotation access
    Eigen::Matrix<double,3,3> Rot() const{
        return q->Rot();
    }

    //FEJ Rotation access
    Eigen::Matrix<double,3,3> Rot_fej() const{
        return q->Rot_fej();;
    }

    //Rotation access
    Eigen::Matrix<double,4,1> quat() const{
        return q->value();
    }

    //FEJ Rotation access
    Eigen::Matrix<double,4,1> quat_fej() const{
        return q->fej();
    }


    //position access
    Eigen::Matrix<double,3,1> pos() const{
        return p->value();
    }

    //FEJ position access
    Eigen::Matrix<double,3,1> pos_fej() const{
        return p->fej();
    }

protected:

    JPLQuat *q;
    Vec *p;

};