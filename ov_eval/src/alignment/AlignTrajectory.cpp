#include "AlignTrajectory.h"


using namespace ov_eval;



void AlignTrajectory::align_trajectory(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                       const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                       Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, std::string method, int n_aligned){

    // Use the correct method
    if (method == "posyaw"){
        s = 1;
        align_posyaw(p_es, p_gt, q_es, q_gt, R, t, n_aligned);
    } else if (method == "se3"){
        s = 1;
        align_se3(p_es, p_gt, q_es, q_gt, R, t, n_aligned);
    } else if (method == "sim3"){
        assert(n_aligned >= 2 || n_aligned == -1);
        align_sim3(p_es, p_gt, q_es, q_gt, R, t,s, n_aligned);
    } else if (method == "none"){
        s = 1;
        R.setIdentity();
        t.setZero();
    } else {
        assert(false);
    }

}


void AlignTrajectory::align_posyaw_single(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                          const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                          Eigen::Matrix3d &R, Eigen::Vector3d &t){

    //Get first ever poses
    Eigen::Vector4d q_es_0 = q_es[0];
    Eigen::Vector3d p_es_0 = p_es[0];

    Eigen::Vector4d q_gt_0 = q_gt[0];
    Eigen::Vector3d p_gt_0 = p_gt[0];


    // Get rotations from IMU frame to World (note JPL!)
    Eigen::Matrix3d g_rot = AlignUtils::quat_2_Rot(q_gt_0).transpose();
    Eigen::Matrix3d est_rot = AlignUtils::quat_2_Rot(q_es_0).transpose();

    // Data matrix for the Frobenius problem
    Eigen::Matrix3d C_R = est_rot*g_rot.transpose();

    // Recover yaw
    double theta = AlignUtils::get_best_yaw(C_R);

    // Compute rotation
    R = AlignUtils::rot_z(theta);

    // Compute translation
    t.noalias() = p_gt_0 - R*p_es_0;

}


void AlignTrajectory::align_posyaw(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                   const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                   Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned){

    // If we only have one, just use the single alignment
    if (n_aligned == 1){
        align_posyaw_single(p_es, p_gt, q_es, q_gt, R, t);
    } else {

        // Align using the method of Umeyama
        //Grab poses used for alignment
        assert(p_es.size() > 0);
        std::vector<Eigen::Vector3d> est_pos(p_es.begin(), p_es.begin()+std::min((size_t) n_aligned, p_es.size()));
        std::vector<Eigen::Vector3d> gt_pos(p_gt.begin(), p_gt.begin()+std::min((size_t) n_aligned, p_gt.size()));

        double s;
        AlignUtils::align_umeyama(gt_pos, est_pos, R,t,s, true, true);
        assert(s == 1);
    }


}


void AlignTrajectory::align_se3_single(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                       const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                       Eigen::Matrix3d &R, Eigen::Vector3d &t){
    // Get initial poses
    Eigen::Vector4d q_es_0 = q_es[0];
    Eigen::Vector3d p_es_0 = p_es[0];

    Eigen::Vector4d q_gt_0 = q_gt[0];
    Eigen::Vector3d p_gt_0 = p_gt[0];

    // Get rotations from IMU frame to World (note JPL!)
    Eigen::Matrix3d g_rot = AlignUtils::quat_2_Rot(q_gt_0).transpose();
    Eigen::Matrix3d est_rot = AlignUtils::quat_2_Rot(q_es_0).transpose();

    R.noalias() = g_rot*est_rot.transpose();
    t.noalias() = p_gt_0 - R*p_es_0;

}


void AlignTrajectory::align_se3(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned){

    // If we only have one, just use the single alignment
    if (n_aligned == 1){
        align_se3_single(p_es, p_gt, q_es, q_gt, R, t);
    } else {

        // Align using the method of Umeyama
        assert(p_es.size() > 0);

        //Grab poses used for alignment
        std::vector<Eigen::Vector3d> est_pos(p_es.begin(), p_es.begin()+std::min((size_t) n_aligned, p_es.size()));
        std::vector<Eigen::Vector3d> gt_pos(p_gt.begin(), p_gt.begin()+std::min((size_t) n_aligned, p_gt.size()));

        double s;
        AlignUtils::align_umeyama(gt_pos, est_pos, R,t,s, true, false);
    }


}




void AlignTrajectory::align_sim3(const std::vector<Eigen::Vector3d> &p_es, const std::vector<Eigen::Vector3d> &p_gt,
                                 const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                 Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, int n_aligned){

    // Need to have more than two to get
    assert(n_aligned >= 2 || n_aligned == -1);

    //Grab poses used for alignment
    assert(p_es.size() > 0);
    std::vector<Eigen::Vector3d> est_pos(p_es.begin(), p_es.begin()+std::min((size_t) n_aligned, p_es.size()));
    std::vector<Eigen::Vector3d> gt_pos(p_gt.begin(), p_gt.begin()+std::min((size_t) n_aligned, p_gt.size()));

    // Align using the method of Umeyama
    AlignUtils::align_umeyama(gt_pos, est_pos, R,t,s, false, false);

}






