#include "ResultTrajectory.h"


using namespace ov_eval;




ResultTrajectory::ResultTrajectory(std::string path_est, std::string path_gt, std::string alignment_method) {

    // Load from file
    Loader::load_data(path_est, est_times, est_poses, est_covori, est_covpos);
    Loader::load_data(path_gt, gt_times, gt_poses, gt_covori, gt_covpos);

    // Debug print amount
    //std::string base_filename1 = path_est.substr(path_est.find_last_of("/\\") + 1);
    //std::string base_filename2 = path_gt.substr(path_gt.find_last_of("/\\") + 1);
    //ROS_INFO("[TRAJ]: loaded %d poses from %s",(int)est_times.size(),base_filename1.c_str());
    //ROS_INFO("[TRAJ]: loaded %d poses from %s",(int)gt_times.size(),base_filename2.c_str());

    // Intersect timestamps
    perform_association(0, 0.02);

    // Return failure if we didn't have any common timestamps
    if(est_poses.size() < 2) {
        ROS_ERROR("[TRAJ]: unable to get enough common timestamps between trajectories.");
        ROS_ERROR("[TRAJ]: does the estimated trajectory publish the rosbag timestamps??");
        std::exit(EXIT_FAILURE);
    }

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT, R_GTtoEST;
    Eigen::Vector3d t_ESTinGT, t_GTinEST;
    double s_ESTtoGT, s_GTtoEST;
    AlignTrajectory::align_trajectory(est_poses, gt_poses, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_method);
    AlignTrajectory::align_trajectory(gt_poses, est_poses, R_GTtoEST, t_GTinEST, s_GTtoEST, alignment_method);

    // Debug print to the user
    Eigen::Vector4d q_ESTtoGT = Math::rot_2_quat(R_ESTtoGT);
    Eigen::Vector4d q_GTtoEST = Math::rot_2_quat(R_GTtoEST);
    ROS_INFO("[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f",q_ESTtoGT(0),q_ESTtoGT(1),q_ESTtoGT(2),q_ESTtoGT(3),t_ESTinGT(0),t_ESTinGT(1),t_ESTinGT(2),s_ESTtoGT);
    //ROS_INFO("[TRAJ]: q_GTtoEST = %.3f, %.3f, %.3f, %.3f | p_GTinEST = %.3f, %.3f, %.3f | s = %.2f",q_GTtoEST(0),q_GTtoEST(1),q_GTtoEST(2),q_GTtoEST(3),t_GTinEST(0),t_GTinEST(1),t_GTinEST(2),s_GTtoEST);

    // Finally lets calculate the aligned trajectories
    for(size_t i=0; i<gt_times.size(); i++) {
        Eigen::Matrix<double,7,1> pose_ESTinGT, pose_GTinEST;
        pose_ESTinGT.block(0,0,3,1) = s_ESTtoGT*R_ESTtoGT*est_poses.at(i).block(0,0,3,1)+t_ESTinGT;
        pose_ESTinGT.block(3,0,4,1) = Math::quat_multiply(est_poses.at(i).block(3,0,4,1),Math::Inv(q_ESTtoGT));
        pose_GTinEST.block(0,0,3,1) = s_GTtoEST*R_GTtoEST*gt_poses.at(i).block(0,0,3,1)+t_GTinEST;
        pose_GTinEST.block(3,0,4,1) = Math::quat_multiply(gt_poses.at(i).block(3,0,4,1),Math::Inv(q_GTtoEST));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
        gt_poses_aignedtoEST.push_back(pose_GTinEST);
    }

}




void ResultTrajectory::calculate_ate(Statistics &error_ori, Statistics &error_pos) {

    // Clear any old data
    error_ori.clear();
    error_pos.clear();

    // Calculate the position and orientation error at every timestep
    for(size_t i=0; i<est_poses_aignedtoGT.size(); i++) {

        // Calculate orientation error
        Eigen::Matrix3d e_R = Math::quat_2_Rot(est_poses_aignedtoGT.at(i).block(3,0,4,1)).transpose() * Math::quat_2_Rot(gt_poses.at(i).block(3,0,4,1));
        double ori_err = 180.0/M_PI*Math::log_so3(e_R).norm();

        // Calculate position error
        double pos_err = (gt_poses.at(i).block(0,0,3,1)-est_poses_aignedtoGT.at(i).block(0,0,3,1)).norm();

        // Append this error!
        error_ori.timestamps.push_back(est_times.at(i));
        error_ori.values.push_back(ori_err);
        error_pos.timestamps.push_back(est_times.at(i));
        error_pos.values.push_back(pos_err);

    }

    // Update stat information
    error_ori.calculate();
    error_pos.calculate();

}




void ResultTrajectory::calculate_rpe(const std::vector<double> &segment_lengths, std::map<double,std::pair<Statistics,Statistics>> &error_rpe) {

    // Distance at each point along the trajectory
    std::vector<double> accum_distances(gt_poses.size());
    accum_distances[0] = 0;
    for (size_t i = 1; i < gt_poses.size(); i++) {
        accum_distances[i] = accum_distances[i - 1] + (gt_poses[i] - gt_poses[i - 1]).norm();
    }

    // Loop through each segment length
    for(const double &distance : segment_lengths) {

        // Our stats for this length
        Statistics error_ori, error_pos;

        // Get end of subtrajectories for each possible starting point
        std::vector<size_t> comparisons = compute_comparison_indices_length(accum_distances, distance, 0.4*distance);

        // Loop through each relative comparison
        for (size_t id_start = 0; id_start < comparisons.size(); id_start++) {

            // Get the end id
            size_t id_end = comparisons[id_start];

            //===============================================================================
            // Get T I1 to world EST at beginning of subtrajectory (at state idx)
            Eigen::Matrix4d T_c1 = Eigen::Matrix4d::Identity();
            T_c1.block(0, 0, 3, 3) = Math::quat_2_Rot(est_poses_aignedtoGT.at(id_start).block(3,0,4,1)).transpose();
            T_c1.block(0, 3, 3, 1) = est_poses_aignedtoGT.at(id_start).block(0,0,3,1);

            // Get T I2 to world EST at end of subtrajectory starting (at state comparisons[idx])
            Eigen::Matrix4d T_c2 = Eigen::Matrix4d::Identity();
            T_c2.block(0, 0, 3, 3) = Math::quat_2_Rot(est_poses_aignedtoGT.at(id_end).block(3,0,4,1)).transpose();
            T_c2.block(0, 3, 3, 1) = est_poses_aignedtoGT.at(id_end).block(0,0,3,1);

            // Get T I2 to I1 EST
            Eigen::Matrix4d T_c1_c2 = T_c1.inverse() * T_c2;

            //===============================================================================
            // Get T I1 to world GT at beginning of subtrajectory (at state idx)
            Eigen::Matrix4d T_m1 = Eigen::Matrix4d::Identity();
            T_m1.block(0, 0, 3, 3) = Math::quat_2_Rot(gt_poses.at(id_start).block(3,0,4,1)).transpose();
            T_m1.block(0, 3, 3, 1) = gt_poses.at(id_start).block(0,0,3,1);

            // Get T I2 to world GT at end of subtrajectory starting (at state comparisons[idx])
            Eigen::Matrix4d T_m2 = Eigen::Matrix4d::Identity();
            T_m2.block(0, 0, 3, 3) = Math::quat_2_Rot(gt_poses.at(id_end).block(3,0,4,1)).transpose();
            T_m2.block(0, 3, 3, 1) = gt_poses.at(id_end).block(0,0,3,1);

            // Get T I2 to I1 GT
            Eigen::Matrix4d T_m1_m2 = T_m1.inverse() * T_m2;

            //===============================================================================
            // Compute error transform between EST and GT start-end transform
            Eigen::Matrix4d T_error_in_c2 = T_m1_m2.inverse() * T_c1_c2;

            Eigen::Matrix4d T_c2_rot = Eigen::Matrix4d::Identity();
            T_c2_rot.block(0, 0, 3, 3) = T_c2.block(0, 0, 3, 3);

            Eigen::Matrix4d T_c2_rot_inv = Eigen::Matrix4d::Identity();
            T_c2_rot_inv.block(0, 0, 3, 3) = T_c2.block(0, 0, 3, 3).transpose();

            // Rotate rotation so that rotation error is in the global frame (allows us to look at yaw error)
            Eigen::Matrix4d T_error_in_w = T_c2_rot * T_error_in_c2 * T_c2_rot_inv;

            //===============================================================================
            // Compute error for position
            error_pos.timestamps.push_back(est_times.at(id_start));
            error_pos.values.push_back(T_error_in_w.block(0, 3, 3, 1).norm());

            // Calculate orientation error
            double ori_err = 180.0/M_PI*Math::log_so3(T_error_in_w.block(0, 0, 3, 3)).norm();
            error_ori.timestamps.push_back(est_times.at(id_start));
            error_ori.values.push_back(ori_err);

        }

        // Update stat information
        error_ori.calculate();
        error_pos.calculate();
        error_rpe.insert({distance, {error_ori,error_pos}});

    }

}




void ResultTrajectory::calculate_nees(Statistics &nees_ori, Statistics &nees_pos) {

    // Check that we have our covariance matrices to normalize with
    if(est_times.size() != est_covori.size() || est_times.size() != est_covpos.size()
        || gt_times.size() != gt_covori.size() || gt_times.size() != gt_covpos.size()) {
        ROS_ERROR("[TRAJ]: Normalized Estimation Error Squared called but trajectory does not have any covariances...");
        ROS_ERROR("[TRAJ]: Did you record using a Odometry or PoseWithCovarianceStamped????");
        return;
    }

    // Clear any old data
    nees_ori.clear();
    nees_pos.clear();

    // Calculate the position and orientation error at every timestep
    for(size_t i=0; i<est_poses.size(); i++) {

        // Calculate orientation error
        // NOTE: we define our error as e_R = -Log(R*Rhat^T)
        Eigen::Matrix3d e_R = Math::quat_2_Rot(gt_poses.at(i).block(3,0,4,1)) * Math::quat_2_Rot(est_poses.at(i).block(3,0,4,1)).transpose();
        Eigen::Vector3d errori = -Math::log_so3(e_R);
        //Eigen::Vector4d e_q = Math::quat_multiply(gt_poses_aignedtoEST.at(i).block(3,0,4,1),Math::Inv(est_poses.at(i).block(3,0,4,1)));
        //Eigen::Vector3d errori = 2*e_q.block(0,0,3,1);
        double ori_nees = errori.transpose()*est_covori.at(i).inverse()*errori;

        // Calculate nees position error
        Eigen::Vector3d errpos = gt_poses_aignedtoEST.at(i).block(0,0,3,1)-est_poses.at(i).block(0,0,3,1);
        double pos_nees = errpos.transpose()*est_covpos.at(i).inverse()*errpos;

        // Skip if nan error value
        if(std::isnan(ori_nees) || std::isnan(pos_nees)) {
            ROS_WARN("[TRAJ]: nees calculation had nan number (covariance is wrong?) skipping...");
            continue;
        }

        // Append this error!
        nees_ori.timestamps.push_back(est_times.at(i));
        nees_ori.values.push_back(ori_nees);
        nees_pos.timestamps.push_back(est_times.at(i));
        nees_pos.values.push_back(pos_nees);

    }


    // Update stat information
    nees_ori.calculate();
    nees_pos.calculate();

}

void ResultTrajectory::calculate_error(Statistics &posx, Statistics &posy, Statistics &posz,
                                       Statistics &orix, Statistics &oriy, Statistics &oriz,
                                       Statistics &roll, Statistics &pitch, Statistics &yaw) {

    // Clear any old data
    posx.clear();
    posy.clear();
    posz.clear();
    orix.clear();
    oriy.clear();
    oriz.clear();
    roll.clear();
    pitch.clear();
    yaw.clear();

    // Calculate the position and orientation error at every timestep
    for(size_t i=0; i<est_poses.size(); i++) {

        // Calculate local orientation error, then propagate its covariance into the global frame of reference
        Eigen::Vector4d e_q = Math::quat_multiply(gt_poses_aignedtoEST.at(i).block(3,0,4,1),Math::Inv(est_poses.at(i).block(3,0,4,1)));
        Eigen::Vector3d errori_local = 2*e_q.block(0,0,3,1);
        Eigen::Vector3d errori_global = Math::quat_2_Rot(est_poses.at(i).block(3,0,4,1)).transpose()*errori_local;
        Eigen::Matrix3d cov_global;
        if(est_times.size() == est_covori.size()) {
            cov_global = Math::quat_2_Rot(est_poses.at(i).block(3,0,4,1)).transpose()*est_covori.at(i)*Math::quat_2_Rot(est_poses.at(i).block(3,0,4,1));
        }

        // Convert to roll pitch yaw (also need to "wrap" the error to -pi to pi)
        // NOTE: our rot2rpy is in the form R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
        // NOTE: this error is in the "global" frame since we do rot2rpy on R_ItoG rotation
        Eigen::Vector3d ypr_est_ItoG = Math::rot2rpy(Math::quat_2_Rot(est_poses.at(i).block(3,0,4,1)).transpose());
        Eigen::Vector3d ypr_gt_ItoG = Math::rot2rpy(Math::quat_2_Rot(gt_poses_aignedtoEST.at(i).block(3,0,4,1)).transpose());
        Eigen::Vector3d errori_rpy = ypr_gt_ItoG-ypr_est_ItoG;
        for(size_t idx=0; idx<3; idx++) {
            while(errori_rpy(idx)<-M_PI) {
                errori_rpy(idx) += 2*M_PI;
            }
            while(errori_rpy(idx)>M_PI) {
                errori_rpy(idx) -= 2*M_PI;
            }
        }

        // Next need to propagate our covariance to the RPY frame of reference
        // NOTE: one can derive this by perturbing the rpy error equation
        // NOTE: R*Exp(theta_erro) = Rz*Rz(error)*Ry*Ry(error)*Rx*Rx(error)
        // NOTE: example can be found here http://mars.cs.umn.edu/tr/reports/Trawny05c.pdf
        Eigen::Matrix<double,3,3> H_xyz2rpy = Eigen::Matrix<double,3,3>::Identity();
        H_xyz2rpy.block(0,1,3,1) = Math::rot_x(-ypr_est_ItoG(0))*H_xyz2rpy.block(0,1,3,1);
        H_xyz2rpy.block(0,2,3,1) = Math::rot_x(-ypr_est_ItoG(0))*Math::rot_y(-ypr_est_ItoG(1))*H_xyz2rpy.block(0,2,3,1);
        Eigen::Matrix3d cov_rpy;
        if(est_times.size() == est_covori.size()) {
            cov_rpy = H_xyz2rpy.inverse()*est_covori.at(i)*H_xyz2rpy.inverse().transpose();
        }

        // Calculate position error
        Eigen::Vector3d errpos = gt_poses_aignedtoEST.at(i).block(0,0,3,1)-est_poses.at(i).block(0,0,3,1);

        // POSITION: Append this error!
        posx.timestamps.push_back(est_times.at(i));
        posx.values.push_back(errpos(0));
        posy.timestamps.push_back(est_times.at(i));
        posy.values.push_back(errpos(1));
        posz.timestamps.push_back(est_times.at(i));
        posz.values.push_back(errpos(2));
        if(est_times.size() == est_covpos.size()) {
            posx.values_bound.push_back(3*std::sqrt(est_covpos.at(i)(0,0)));
            posy.values_bound.push_back(3*std::sqrt(est_covpos.at(i)(1,1)));
            posz.values_bound.push_back(3*std::sqrt(est_covpos.at(i)(2,2)));
        }

        // ORIENTATION: Append this error!
        orix.timestamps.push_back(est_times.at(i));
        orix.values.push_back(180.0/M_PI*errori_global(0));
        oriy.timestamps.push_back(est_times.at(i));
        oriy.values.push_back(180.0/M_PI*errori_global(1));
        oriz.timestamps.push_back(est_times.at(i));
        oriz.values.push_back(180.0/M_PI*errori_global(2));
        if(est_times.size() == est_covori.size()) {
            orix.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_global(0,0)));
            oriy.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_global(1,1)));
            oriz.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_global(2,2)));
        }

        // RPY: Append this error!
        roll.timestamps.push_back(est_times.at(i));
        roll.values.push_back(180.0/M_PI*errori_rpy(0));
        pitch.timestamps.push_back(est_times.at(i));
        pitch.values.push_back(180.0/M_PI*errori_rpy(1));
        yaw.timestamps.push_back(est_times.at(i));
        yaw.values.push_back(180.0/M_PI*errori_rpy(2));
        if(est_times.size() == est_covori.size()) {
            roll.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_rpy(0,0)));
            pitch.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_rpy(1,1)));
            yaw.values_bound.push_back(3*180.0/M_PI*std::sqrt(cov_rpy(2,2)));
        }

    }

    // Update stat information
    posx.calculate();
    posy.calculate();
    posz.calculate();
    orix.calculate();
    oriy.calculate();
    oriz.calculate();
    roll.calculate();
    pitch.calculate();
    yaw.calculate();

}



void ResultTrajectory::perform_association(double offset, double max_difference) {

    // Temp results which keeps only the matches
    std::vector<double> est_times_temp, gt_times_temp;
    std::vector<Eigen::Matrix<double,7,1>> est_poses_temp, gt_poses_temp;
    std::vector<Eigen::Matrix3d> est_covori_temp, est_covpos_temp, gt_covori_temp, gt_covpos_temp;

    // Iterator over gt (only ever increases to enforce injectivity of matches)
    size_t gt_pointer = 0;

    // Try to find closest GT pose for each estimate
    for (size_t i = 0 ; i < est_times.size(); i++){

        // Default params
        double best_diff = max_difference;
        int best_gt_idx = -1;

        // Increment while too small and is not within our difference threshold
        while (gt_pointer < gt_times.size()
                && gt_times.at(gt_pointer) < (est_times.at(i)+offset)
                && std::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) > max_difference) {
            gt_pointer++;
        }

        // If we are closer than max difference, see if we can do any better
        while (gt_pointer < gt_times.size() && std::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) <= max_difference) {
            // Break if we found a good match but are getting worse, we are done
            if (std::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset)) >= best_diff){
                break;
            }
            // We have a closer match, save it and move on
            best_diff = std::abs(gt_times.at(gt_pointer)-(est_times.at(i)+offset));
            best_gt_idx = gt_pointer;
            gt_pointer++;
        }

        // Did we get a valid match
        if (best_gt_idx != -1) {

            // Save estimate and gt states for the match
            est_times_temp.push_back(gt_times.at(best_gt_idx));
            est_poses_temp.push_back(est_poses.at(i));
            gt_times_temp.push_back(gt_times.at(best_gt_idx));
            gt_poses_temp.push_back(gt_poses.at(best_gt_idx));

            // If we have covariance then also append it
            // If the groundtruth doesn't have covariance say it is 100% certain
            if(!est_covori.empty()) {
                assert(est_covori.size()==est_covpos.size());
                est_covori_temp.push_back(est_covori.at(i));
                est_covpos_temp.push_back(est_covpos.at(i));
                if(gt_covori.empty()) {
                    gt_covori_temp.push_back(Eigen::Matrix3d::Zero());
                    gt_covpos_temp.push_back(Eigen::Matrix3d::Zero());
                } else {
                    assert(gt_covori.size()==gt_covpos.size());
                    gt_covori_temp.push_back(gt_covori.at(best_gt_idx));
                    gt_covpos_temp.push_back(gt_covpos.at(best_gt_idx));
                }
            }

        }
    }

    // Ensure that we have enough assosiations
    if(est_times.size() < 3) {
        ROS_ERROR("[TRAJ]: Was unable to assosiate groundtruth and estimate trajectories");
        ROS_ERROR("[TRAJ]: %d total matches....",(int)est_times.size());
        ROS_ERROR("[TRAJ]: Do the time of the files match??");
        std::exit(EXIT_FAILURE);
    }
    assert(est_times_temp.size()==gt_times_temp.size());
    //ROS_INFO("[TRAJ]: %d est poses and %d gt poses => %d matches",(int)est_times.size(),(int)gt_times.size(),(int)est_times_temp.size());

    // Overwrite with intersected values
    est_times = est_times_temp;
    est_poses = est_poses_temp;
    est_covori = est_covori_temp;
    est_covpos = est_covpos_temp;
    gt_times = gt_times_temp;
    gt_poses = gt_poses_temp;
    gt_covori = gt_covori_temp;
    gt_covpos = gt_covpos_temp;


}

