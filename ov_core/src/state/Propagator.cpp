//
// Created by keck on 6/4/19.
//

#include "Propagator.h"


bool Propagator::propagateAndClone(State* state, double timestamp){

    // First lets construct an IMU vector of measurements we need
    vector<IMUDATA> prop_data;

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = state->calib_dt_CAMtoIMU()->value()(0,0);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Ensure we have some measurements in the first place!
    if(imu_data.empty()) {
        //ROS_ERROR("[TIME-SYNC]: There are no IMU measurements!!!!!");
        //ROS_ERROR("[TIME-SYNC]: IMU-CAMERA are likely messed up, check time offset value!!!");
        //ROS_ERROR("%s on line %d",__FILE__,__LINE__);
    //std::exit(EXIT_FAILURE);
        state->set_timestamp(timestamp);
        last_prop_time_offset = t_off_new;
        return false;
    }

    // If the difference between the current update time and state is zero
    // We should crash, as this means we would have two clones at the same time!!!!
    if(state->timestamp() == timestamp) {
        //ROS_ERROR("[TIME-SYNC]: Propagation called again at same timestep at last update timestep!!!!");
        //ROS_ERROR("[TIME-SYNC]: %.15f vs %.15f timestamps",state->timestamp,timestamp);
        //ROS_ERROR("%s on line %d",__FILE__,__LINE__);
        std::exit(EXIT_FAILURE);
    }

    // Loop through and find all the needed measurements to propagate with
    // Note we split measurements based on the given state time, and the update timestamp
    for(size_t i=0; i<imu_data.size()-1; i++) {

    // START OF THE INTEGRATION PERIOD
    // If the next timestamp is greater then our current state time
    // And the current is not greater then it yet...
    // Then we should "split" our current IMU measurement
        if(imu_data.at(i+1).timestamp > state->timestamp()+last_prop_time_offset && imu_data.at(i).timestamp <=
                                                                                    state->timestamp()+last_prop_time_offset) {
            IMUDATA data = interpolate_data(imu_data.at(i),imu_data.at(i+1),state->timestamp()+last_prop_time_offset);
            prop_data.push_back(data);
    //ROS_INFO("propagation #%d = CASE 1 = %.3f => %.3f",
    // (int)i,data.timestamp-prop_data.at(0).timestamp,state->timestamp-prop_data.at(0).timestamp);
            continue;
        }

    // MIDDLE OF INTEGRATION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then we should just append the whole measurement time to our propagation vector
        if(imu_data.at(i).timestamp >= state->timestamp()+last_prop_time_offset && imu_data.at(i+1).timestamp <=
                                                                                   timestamp+t_off_new) {
            prop_data.push_back(imu_data.at(i));
    //ROS_INFO("propagation #%d = CASE 2 = %.3f",(int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp);
            continue;
        }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the inverval (so we can get a dt)
    // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
        if(imu_data.at(i+1).timestamp > timestamp+t_off_new) {
            prop_data.push_back(imu_data.at(i));
    // If the added IMU message doesn't end exactly at the camera time
    // Then we need to add another one that is right at the ending time
            if(prop_data.at(prop_data.size()-1).timestamp != timestamp+t_off_new) {
                IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i + 1), timestamp + t_off_new);
                prop_data.push_back(data);
            }
    //ROS_INFO("propagation #%d = CASE 3 = %.3f => %.3f",
    // (int)i,data.timestamp-prop_data.at(0).timestamp,data.timestamp-prop_data.at(0).timestamp);
            break;
        }

    }

    // Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        //ROS_ERROR("[TIME-SYNC]: There are not enough measurements to propagate with %d of 2",(int)prop_data.size());
        //ROS_ERROR("[TIME-SYNC]: IMU-CAMERA are likely messed up, check time offset value!!!");
        //ROS_ERROR("%s on line %d",__FILE__,__LINE__);
    //std::exit(EXIT_FAILURE);
        state->set_timestamp(timestamp);
        last_prop_time_offset = t_off_new;
        return false;
    }


    // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to reach)
    // Then we should just "stretch" the last measurement to be the whole period
    if(imu_data.at(imu_data.size()-1).timestamp < timestamp+t_off_new) {
        IMUDATA data = interpolate_data(imu_data.at(imu_data.size()-2),imu_data.at(imu_data.size()-1),timestamp+t_off_new);
        prop_data.push_back(data);
    //ROS_INFO("propagation #%d = CASE 4 = %.3f",(int)(imu_data.size()-1),data.timestamp-prop_data.at(0).timestamp);
    }

    //ROS_INFO("end_prop - start_prop %.3f",prop_data.at(prop_data.size()-1).timestamp-prop_data.at(0).timestamp);
    //ROS_INFO("imu_data_last - state %.3f",imu_data.at(imu_data.size()-1).timestamp-state->timestamp());
    //ROS_INFO("update_time - state %.3f",timestamp-state->timestamp());

    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (size_t i=0; i < prop_data.size()-1; i++){
    //cout << "dti- " << (prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) << endl;
        if ((prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) < 1e-8){
            //ROS_ERROR("[TIME-SYNC]: Zero DT between %d and %d measurements (dt = %.8f)",(int)i,(int)(i+1),(prop_data.at(i+1).timestamp-prop_data.at(i).timestamp));
            return false;
            prop_data.erase(prop_data.begin()+i);
            i--;
        }
    }


    // Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        //ROS_ERROR("[TIME-SYNC]: There are not enough measurements to propagate with %d of 2",(int)prop_data.size());
        //ROS_ERROR("[TIME-SYNC]: IMU-CAMERA are likely messed up, check time offset value!!!");
        //ROS_ERROR("%s on line %d",__FILE__,__LINE__);
    //std::exit(EXIT_FAILURE);
        state->set_timestamp(timestamp);
        last_prop_time_offset = t_off_new;
        return false;
    }


    Eigen::Matrix<double,15,15> Phi_summed = Eigen::Matrix<double,15,15>::Identity();
    Eigen::Matrix<double,15,15> Qd_summed = Eigen::Matrix<double,15,15>::Zero();

    for(size_t i=0; i<prop_data.size()-1; i++) {
        propagateSingleInterval(state, prop_data.at(i), prop_data.at(i+1),
                                Phi_summed, Qd_summed);
    }

    //Last angular velocity (used for cloning when estimating time offset)
    Eigen::Matrix<double,3,1> last_w = prop_data.at(prop_data.size()-1).wm - state->imu()->bias_g();

    auto Cov = state->Cov();

    size_t imu_id = 0;
    // Do the update to the covariance with our "summed" state transition and IMU noise addition...
    Cov.block(imu_id,0,15,state->nVars()) = Phi_summed*Cov.block(imu_id,0,15,state->nVars());
    Cov.block(0,imu_id,state->nVars(),15) = Cov.block(0,imu_id,state->nVars(),15)*Phi_summed.transpose();
    Cov.block(imu_id,imu_id,15,15) += Qd_summed;

    // Ensure the covariance is symmetric
    Cov = 0.5*(Cov+Cov.transpose());

    //Set timestamp data
    state->set_timestamp(timestamp);
    last_prop_time_offset = t_off_new;


    //Now perform stochastic cloning
    StateHelper::augment_clone(state, last_w);

}

void Propagator::predictAndcomputeJacobians(State *state, const IMUDATA data_minus, const IMUDATA data_plus,
                                Eigen::Matrix<double,15,15> &F, Eigen::Matrix<double,15,15> &Qd){

    auto imu = state->imu();

    // Time elapsed over interval
    double dt = data_plus.timestamp-data_minus.timestamp;

    Eigen::Matrix<double,3,1> w_hat = data_minus.wm - imu->bias_g();
    Eigen::Matrix<double,3,1> a_hat = data_minus.am - imu->bias_a();


    if (state->options().imu_avg){
        Eigen::Matrix<double,3,1> w_hat2 = data_plus.wm - imu->bias_g();
        Eigen::Matrix<double,3,1> a_hat2 = data_plus.am - imu->bias_a();

        w_hat = .5*(w_hat+w_hat2);
        a_hat = .5*(a_hat+a_hat2);
    }

    // Pre-compute things
    double w_norm = w_hat.norm();
    Eigen::Matrix<double,4,4> I_4x4 = Eigen::Matrix<double,4,4>::Identity();

    // Orientation: Equation (101) and (103) and of Trawny indirect TR
    Eigen::Matrix<double,4,4> bigO;
    if(w_norm > 1e-3) {
        bigO = cos(0.5*w_norm*dt)*I_4x4 + 1/w_norm*sin(0.5*w_norm*dt)*Omega(w_hat);
    } else {
        bigO = I_4x4 + 0.5*dt*Omega(w_hat);
    }
    Eigen::Matrix<double,4,1> new_q = quatnorm(bigO*imu->quat());

    Eigen::Matrix<double,3,3> R_Gtoi = imu->Rot();

    // Velocity: just the acceleration in the local frame, minus global gravity
    Eigen::Matrix<double,3,1> new_v = imu->vel() + R_Gtoi.transpose()*a_hat*dt - _gravity*dt;

    // Position: just velocity times dt, with the acceleration integrated twice
    Eigen::Matrix<double,3,1> new_p = imu->pos() + imu->vel()*dt + 0.5*R_Gtoi.transpose()*a_hat*dt*dt -
                                      0.5*_gravity*dt*dt;

    //Indexing within Jacobian
    int th_id = 0;
    int p_id = 3;
    int v_id = 6;
    int bg_id = 9;
    int ba_id = 12;

    //Allocate noise Jacobian
    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();


    //Now compute Jacobian of new state wrt old state and noise
    if (state->options().do_fej) {

        Eigen::Matrix<double,3,3> Rfej = imu->Rot_fej();
        Eigen::Matrix<double,3,3> dR = quat_2_Rot(new_q)*Rfej.transpose();

        Eigen::Matrix<double,3,1> v_fej = imu->vel_fej();
        Eigen::Matrix<double,3,1> p_fej = imu->pos_fej();


        F.block(th_id, th_id, 3, 3) = dR;
        F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v-v_fej+_gravity*dt)*Rfej.transpose();
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() =
                -skew_x(new_p-p_fej-v_fej*dt+.5*_gravity*dt*dt)*Rfej.transpose();
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
        G.block(bg_id, 3, 3, 3).setIdentity();
        G.block(v_id, 6, 3, 3) = -Rfej.transpose() * dt;
        G.block(ba_id, 9, 3, 3).setIdentity();
        G.block(p_id, 6, 3, 3) = -.5 * Rfej.transpose() * dt * dt;

    } else {

        F.block(th_id, th_id, 3, 3) = Exp(-w_hat * dt);
        F.block(th_id, bg_id, 3, 3).noalias() = -Exp(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() =
                -0.5 *R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -Exp(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        G.block(bg_id, 3, 3, 3).setIdentity();
        G.block(v_id, 6, 3, 3) = -imu->Rot().transpose() * dt;
        G.block(ba_id, 9, 3, 3).setIdentity();
        G.block(p_id, 6, 3, 3) = -.5 * imu->Rot().transpose() * dt * dt;
    }


    // Get noise covariance
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
    Qc.block(0,0,3,3) = _noises.sigma_w_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(3,3,3,3) = _noises.sigma_wb_2*dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(6,6,3,3) = _noises.sigma_a_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(9,9,3,3) = _noises.sigma_ab_2*dt*Eigen::Matrix<double,3,3>::Identity();

    // Compute the noise injected into the state over the interval
    Qd = G*Qc*G.transpose();

    //Now replace imu estimate and fej with propagated values
    Eigen::Matrix<double,16,1> imu_x = imu->value();
    imu_x.block(0,0,4,1) = new_q;
    imu_x.block(4,0,3,1) = new_p;
    imu_x.block(7,0,3,1) = new_v;
    imu->set_value(imu_x);
    imu->set_fej(imu_x);

}
