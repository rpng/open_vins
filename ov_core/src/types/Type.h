//
// Created by keck on 5/23/19.
//

#ifndef PROJECT_TYPE_H
#define PROJECT_TYPE_H

#include <Eigen/Dense>

class Type {

public:

    Type(int size_){_size= size_;}

    virtual ~Type() { };

    //Set id of variable in covariance
    virtual void set_local_id(int new_id){
        _id= new_id;
    }

    int id(){
        return _id;
    }

    int size(){
        return _size;
    }

    //Update estimate using local perturbation 0
    virtual void update(const Eigen::VectorXd dx) = 0;

    //Returns the estimate
    virtual Eigen::VectorXd value() const{
        return _value;
    }

    //Returns the first estimate for fej
    virtual Eigen::VectorXd fej() const{
        return _fej;
    }

    //Sets the value of the estimate
    virtual void set_value(const Eigen::VectorXd new_value){
        _value = new_value;
    }

    //Sets the value of the estimate
    virtual void set_fej(const Eigen::VectorXd new_value){
        _fej = new_value;
    }

    virtual Type* clone()=0;

    //Used to find if variable is this one. Returns nullptr if not.
    // Overloaded to find subvariables as well
    virtual Type* check_if_same_variable(const Type* check){
        if (check == this){
            return this;
        }
        else{
            return nullptr;
        }
    }

protected:

    Eigen::VectorXd _fej;
    Eigen::VectorXd _value;

    int _id = -1;
    int _size = -1;


};

#endif //PROJECT_TYPE_H