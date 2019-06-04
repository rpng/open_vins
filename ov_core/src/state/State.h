//
// Created by keck on 6/3/19.
//

#ifndef PROJECT_STATE_H
#define PROJECT_STATE_H

#include "types/IMU.h"
#include "types/Vec.h"
#include "types/PoseJPL.h"
#include "Options.h"
#include <map>

class Landmark;

/** @brief Class which manages the filter state
*/

class State{

public:

    /** @brief Default Constructor
     *  @param options_ Options structure containing filter options
     */
    State(Options &options_) : _options(options_){}

    ~State(){}

    /**
     * @brief Initializes pointers and covariance
     * TODO: Read initial values and covariance from options
     */
    void initialize_variables(){

        double current_id = 0;
        _imu = new IMU();
        _imu->set_local_id(current_id);
        insert_variable(_imu);

        current_id += 15;

        //Camera to IMU time offset
        _calib_dt_CAMtoIMU = new Vec(1);
        if (_options.do_calib_camera_timeoffset){
            _calib_dt_CAMtoIMU->set_local_id(current_id);
            insert_variable(_calib_dt_CAMtoIMU);
            current_id++;
        }

        for (size_t i = 0; i < _options.num_cameras; i++){
            //Allocate pose
            PoseJPL *pose = new PoseJPL();
            //Allocate intrinsics
            Vec *intrin = new Vec(8);

            //Add these to the corresponding maps
            _calib_IMUtoCAM.insert({i, pose});
            _cam_intrinsics.insert({i, intrin});

            //If calibrating camera-imu pose, add to variables
            if (_options.do_calib_camera_pose){
                pose->set_local_id(current_id);
                current_id += 6;
                insert_variable(pose);
            }
            //If calibrating camera intrinsics, add to variables
            if (_options.do_calib_camera_intrinsics){
                intrin->set_local_id(current_id);
                current_id += 8;
                insert_variable(intrin);
            }
        }

        _Cov = Eigen::MatrixXd::Zero(current_id, current_id);



    }


    /** @brief Access IMU pointer
     *
     */
    IMU *imu(){
        return _imu;
    }


    /** @brief Access reference to covariance
     */
    Eigen::MatrixXd& Cov(){
        return _Cov;
    }

    /** @brief Access variables in state
     */
    std::vector<Type*> & variables(){
        return _variables;
    }

    /** @brief Insert new variable
     *  @param newType Variable to insert
     */
    void  insert_variable(Type* newType){
        _variables.push_back(newType);
    }

    /**
    * @brief For a given set of variables, this will this will calculate a smaller covariance
    * That only includes the ones specified with all cross terms
    * @param small_variables Vector of variables whose marginal covariance is desired
    */
    Eigen::MatrixXd get_marginal_covariance(const std::vector<Type*> &small_variables){

        // Calculate the marginal covariance size we need ot make our matrix
        int cov_size=0;
        for (size_t i=0; i < small_variables.size(); i++){
            cov_size += small_variables[i]->size();
        }

        // Construct our return covariance
        Eigen::MatrixXd Small_cov(cov_size,cov_size);

        // For each variable, lets copy over all other variable cross terms
        // Note: this copies over itself to when i_index=k_index
        int i_index=0;
        for (size_t i=0; i < small_variables.size(); i++){
            int k_index=0;
            for (size_t k=0;  k < small_variables.size(); k++){
                Small_cov.block(i_index, k_index, small_variables[i]->size(),small_variables[k]->size()) =
                        _Cov.block(small_variables[i]->id(), small_variables[k]->id(), small_variables[i]->size(),
                                small_variables[k]->size());
                k_index += small_variables[k]->size();
            }
            i_index += small_variables[i]->size();
        }

        // Return the covariance
        return Small_cov;
    }

    /**
     * @brief Given an update vector, updates each variable
     * @param dx Correction vector for the entire filter state
     */
    void update(const Eigen::MatrixXd dx){
        for (size_t i=0; i < _variables.size(); i++){
            _variables[i]->update(dx.block(_variables[i]->id(),0,_variables[i]->size(),1));
        }
    }

    /**
     * @brief Insert new clone
     * @param timestamp Timestamp associated with new clone
     * @param pose Pointer to new clone pose
     */
    void insert_clone(double timestamp, PoseJPL* pose){
        _clones_IMU.insert({timestamp, pose});
    }

    /// Access current timestamp
    double timestamp(){
        return _timestamp;
    }

    //Access options
    Options& options(){
        return _options;
    }

    //Access imu-camera time offset pointer
    Vec* calib_dt_CAMtoIMU(){
        return _calib_dt_CAMtoIMU;
    }

    /**
     * @brief Get clone at a given timestamp
     * @param timestamp
     */
    PoseJPL* get_clone(double timestamp){
        return _clones_IMU[timestamp];
    }

protected:

    /// Current timestamp
    double _timestamp;

    ///Structure containing filter options
    Options _options;


    /// Covariance of the state
    Eigen::MatrixXd _Cov;

    /// Pointer to the "active" IMU state
    IMU *_imu;


    /// Calibration poses for each camera (R_ItoC, p_IinC)
    std::map<size_t,PoseJPL*> _calib_IMUtoCAM;


    /// Our current set of SLAM features (3d positions)
    std::map<size_t,Landmark*> _features_SLAM;


    /// Map between imaging times and clone poses
    std::map<double, PoseJPL*> _clones_IMU;

    /// Time offset base IMU to camera
    Vec* _calib_dt_CAMtoIMU;

    /// Camera intrinsics
    std::map<size_t,Vec*> _cam_intrinsics;


    std::vector<Type*> _variables;


};



#endif //PROJECT_STATE_H
