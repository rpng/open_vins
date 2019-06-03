//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_VEC_H
#define PROJECT_VEC_H

#endif //PROJECT_VEC_H

#include "Type.h"

class Vec: public Type {

public:

    Vec(int dim) : Type(dim){}

    ~Vec() { }

    void update(const Eigen::VectorXd dx){

        assert(dx.rows() == _size);
        set_value(_value + dx);
    }

    Type* clone(){
        Type* Clone= new Vec(_size);
        Clone->set_value(value());
        Clone->set_fej(fej());
        return Clone;
    }


};