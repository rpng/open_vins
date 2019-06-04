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

/** @brief Class which manages the filter state
*/

class Landmark;

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
    void initialize_variables();


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

    /** @brief Access variables in state by index in variables vector
     * Index in variables vector
     */
    Type* variables(size_t i){
        return _variables[i];
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
    Eigen::MatrixXd get_marginal_covariance(const std::vector<Type*> &small_variables);

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

    /// Get size of covariance
    size_t nVars(){
        return _Cov.rows();
    }

    /// Get current number of clones
    size_t nClones(){
        return _clones_IMU.size();
    }

    /// Get marginal timestep
    double margtimestep() {
        double time = INFINITY;
        for(std::pair<const double,PoseJPL*>& clone_imu : _clones_IMU) {
            if(clone_imu.first < time) {
                time = clone_imu.first;
            }
        }
        return time;
    }

    /**
     * Erases clone associated with timestamp
     * @param timestamp Timestamp of clone to erase
     */
    void erase_clone(double timestamp){
        _clones_IMU.erase(timestamp);
    }

    /**
     * Set the current timestamp of the filter
     * @param timestamp New timestamp
     */
    void set_timestamp(double timestamp){
        _timestamp = timestamp;
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

    /// Vector of variables
    std::vector<Type*> _variables;


};



#endif //PROJECT_STATE_H
