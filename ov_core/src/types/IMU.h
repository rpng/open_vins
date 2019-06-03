//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_JPLQUAT_H
#define PROJECT_JPLQUAT_H

#endif //PROJECT_JPLQUAT_H

#include "utils/quat_ops.h"
#include "JPLPose.h"

class IMU : public Type{

public:

    IMU() : Type(15){
        pose = new JPLPose();
        v = new Vec(3);
        bg = new Vec(3);
        ba = new Vec(3);
    }

    ~IMU() { }

    void update(const Eigen::VectorXd dx){

        assert(dx.rows()== _size);

        Eigen::Matrix<double,16,1> newX = _value;

        Eigen::Matrix<double,4,1> dq;
        dq << .5*dx.block(0,0,3,1) , 1.0;
        dq= dq/dq.norm();

        newX.block(0,0,4,1) = quat_multiply(_value, dq);
        newX.block(4,0,3,1) += dx.block(3,0,3,1);

        newX.block(7,0,3,1) += dx.block(6,0,3,1);
        newX.block(10,0,3,1) += dx.block(9,0,3,1);
        newX.block(13,0,3,1) += dx.block(12,0,3,1);

        set_value(newX);

    }

    //Sets the value of the estimate
    void set_value(const Eigen::VectorXd new_value){

        pose->set_value(new_value.block(0,0,7,1));
        v->set_value(new_value.block(7,0,3,1));
        bg->set_value(new_value.block(10,0,3,1));
        ba->set_value(new_value.block(13,0,3,1));

        _value = new_value;
    }

    //Sets the value of the estimate
    void set_fej(const Eigen::VectorXd new_value){

        pose->set_fej_value(new_value.block(0,0,7,1));
        v->set_fej(new_value.block(7,0,3,1));
        bg->set_fej(new_value.block(10,0,3,1));
        ba->set_fej(new_value.block(13,0,3,1));

        _fej = new_value;
    }

    Type* clone(){
        Type* Clone= new IMU();
        Clone->set_value(value());
        Clone->set_fej(fej());
        return Clone;
    }

    //Rotation access
    Eigen::Matrix<double,3,3> Rot() const{
        return pose->Rot();
    }

    //FEJ Rotation access
    Eigen::Matrix<double,3,3> Rot_fej() const{
        return pose->Rot_fej();
    }

    //Rotation access
    Eigen::Matrix<double,4,1> quat() const{
        return pose->quat();
    }

    //FEJ Rotation access
    Eigen::Matrix<double,4,1> quat_fej() const{
        return pose->quat_fej();
    }


    //position access
    Eigen::Matrix<double,3,1> pos() const{
        return pose->pos();
    }

    //FEJ position access
    Eigen::Matrix<double,3,1> pos_fej() const{
        return pose->pos_fej();
    }

    //Velocity access
    Eigen::Matrix<double,3,1> vel() const{
        return v->value();
    }

    //FEJ position access
    Eigen::Matrix<double,3,1> vel_fej() const{
        return v->fej();
    }

    //gyro bias access
    Eigen::Matrix<double,3,1> bias_g() const{
        return bg->value();
    }

    //FEJ gyro bias access
    Eigen::Matrix<double,3,1> bias_g_fej() const{
        return bg->fej();
    }

    //accel bias access
    Eigen::Matrix<double,3,1> bias_a() const{
        return ba->value();
    }

    //FEJ accel bias access
    Eigen::Matrix<double,3,1> bias_a_fej() const{
        return ba->fej();
    }

protected:

    JPLPose *pose;
    Vec *v;
    Vec *bg;
    Vec *ba;

};