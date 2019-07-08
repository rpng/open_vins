#include "Trajectory.h"


using namespace ov_eval;




Trajectory::Trajectory(std::string path_est, std::string path_gt, std::string alignment_method) {

    // Load from file
    load_data(path_est, est_times, est_poses, est_covori, est_covpos);
    load_data(path_gt, gt_times, gt_poses, gt_covori, gt_covpos);

    // Intersect timestamps
    perform_association(0, 0.02);

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT, R_GTtoEST;
    Eigen::Vector3d t_ESTinGT, t_GTinEST;
    double s_ESTtoGT, s_GTtoEST;
    AlignTrajectory::align_trajectory(est_poses, gt_poses, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_method);
    AlignTrajectory::align_trajectory(gt_poses, est_poses, R_GTtoEST, t_GTinEST, s_GTtoEST, alignment_method);

    // Debug print to the user
    Eigen::Vector4d q_ESTtoGT = Math::rot_2_quat(R_ESTtoGT);
    Eigen::Vector4d q_GTtoEST = Math::rot_2_quat(R_GTtoEST);
    ROS_INFO("[ALIGN]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f",q_ESTtoGT(0),q_ESTtoGT(1),q_ESTtoGT(2),q_ESTtoGT(3),t_ESTinGT(0),t_ESTinGT(1),t_ESTinGT(2),s_ESTtoGT);
    ROS_INFO("[ALIGN]: q_GTtoEST = %.3f, %.3f, %.3f, %.3f | t_GTinEST = %.3f, %.3f, %.3f | s = %.2f",q_GTtoEST(0),q_GTtoEST(1),q_GTtoEST(2),q_GTtoEST(3),t_GTinEST(0),t_GTinEST(1),t_GTinEST(2),s_GTtoEST);

    // Finally lets calculate the aligned trajectories
    for(size_t i=0; i<gt_times.size(); i++) {
        Eigen::Matrix<double,7,1> pose_ESTinGT, pose_GTinEST;

        pose_ESTinGT.block(0,0,3,1) = s_ESTtoGT*R_ESTtoGT*est_poses.at(i).block(0,0,3,1)+t_ESTinGT;
        pose_ESTinGT.block(3,0,4,1) = Math::quat_multiply(est_poses.at(i).block(3,0,4,1),Math::rot_2_quat(R_ESTtoGT.transpose()));

        pose_GTinEST.block(0,0,3,1) = s_GTtoEST*R_GTtoEST*gt_poses.at(i).block(0,0,3,1)+t_GTinEST;
        pose_GTinEST.block(3,0,4,1) = Math::quat_multiply(gt_poses.at(i).block(3,0,4,1),Math::rot_2_quat(R_GTtoEST.transpose()));
        est_poses_aignedtoGT.push_back(pose_ESTinGT);
        gt_poses_aignedtoEST.push_back(pose_GTinEST);
    }

}




void Trajectory::calculate_ate(Statistics &error_ori, Statistics &error_pos) {

    // Clear any old data
    error_ori.timestamps.clear();
    error_ori.values.clear();
    error_ori.values_bound.clear();
    error_pos.timestamps.clear();
    error_pos.values.clear();
    error_pos.values_bound.clear();

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




void Trajectory::calculate_rpe(const std::vector<double> &segment_lengths, std::map<double,std::pair<Statistics,Statistics>> &error_rpe) {

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
        std::vector<size_t> comparisons = compute_comparison_indices_length(accum_distances, distance, 0.2*distance);

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




void Trajectory::load_data(std::string path_traj,
                           std::vector<double> &times, std::vector<Eigen::Matrix<double,7,1>> &poses,
                           std::vector<Eigen::Matrix3d> &cov_ori, std::vector<Eigen::Matrix3d> &cov_pos) {

    // Try to open our trajectory file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
        ROS_ERROR("ERROR: Unable to open trajectory file...");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Loop through each line of this file
    std::string current_line;
    while(std::getline(file, current_line) && ros::ok()) {

        // Skip if we start with a comment
        if(!current_line.find("#"))
            continue;

        // Loop variables
        int i = 0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<double,20,1> data;

        // Loop through this line (timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33)
        while(std::getline(s,field,' ') && ros::ok()) {
            // Skip if empty
            if(field.empty() || i >= data.rows())
                continue;
            // save the data to our vector
            data(i) = std::atof(field.c_str());
            i++;
        }

        // Only a valid line if we have all the parameters
        if(i >= 20) {
            // time and pose
            times.push_back(data(0));
            poses.push_back(data.block(1,0,7,1));
            // covariance values
            Eigen::Matrix3d c_ori, c_pos;
            c_ori << data(8),data(9),data(10),
                    data(9),data(11),data(12),
                    data(10),data(12),data(13);
            c_pos << data(14),data(15),data(16),
                    data(15),data(17),data(18),
                    data(16),data(18),data(19);
            cov_ori.push_back(c_ori);
            cov_pos.push_back(c_pos);
        } else if(i >= 8) {
            times.push_back(data(0));
            poses.push_back(data.block(1,0,7,1));
        }

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (times.empty()) {
        ROS_ERROR("ERROR: Could not parse any data from the file!!");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that they are all equal
    if(times.size() != poses.size()) {
        ROS_ERROR("ERROR: Parsing error, pose and timestamps do not match!!");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Assert that they are all equal
    if(!cov_ori.empty() && (times.size() != cov_ori.size() || times.size() != cov_pos.size())) {
        ROS_ERROR("ERROR: Parsing error, timestamps covariance size do not match!!");
        ROS_ERROR("ERROR: %s",path_traj.c_str());
        std::exit(EXIT_FAILURE);
    }

    // Debug print amount
    std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
    ROS_INFO("[TRAJECTORY]: loaded %d poses from %s",(int)poses.size(),base_filename.c_str());

}



void Trajectory::perform_association(double offset, double max_difference) {

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
        ROS_ERROR("ERROR: Was unable to assosiate groundtruth and estimate trajectories");
        ROS_ERROR("ERROR: %d total matches....",(int)est_times.size());
        ROS_ERROR("ERROR: Do the time of the files match??");
        std::exit(EXIT_FAILURE);
    }
    assert(est_times_temp.size()==gt_times_temp.size());
    ROS_INFO("[TRAJECTORY]: %d est poses and %d gt poses => %d matches",(int)est_times.size(),(int)gt_times.size(),(int)est_times_temp.size());

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

