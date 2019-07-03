#include "AlignTrajectory.h"


using namespace ov_eval;



void AlignTrajectory::alignTrajectory(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                      std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                      Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, std::string method, int n_aligned){

    // Use the correct method
    if (method == "sim3"){
        assert(n_aligned >= 2 || n_aligned == -1);
        alignSIM3(p_es, p_gt, q_es, q_gt, R, t,s, n_aligned);
    }
    else if (method == "se3"){
        s = 1;
        alignSE3(p_es, p_gt, q_es, q_gt, R, t, n_aligned);
    }
    else if (method == "posyaw"){
        s = 1;
        alignPositionYaw(p_es, p_gt, q_es, q_gt, R, t, n_aligned);
    }
    else if (method == "none"){
        R.setIdentity();
        t.setZero();
    } else {
        assert(false);
    }

}




void AlignTrajectory::alignPositionYawSingle(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                      std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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




void AlignTrajectory::alignPositionYaw(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned){

    if (n_aligned == 1){
        // If we only have one, just use the single alignment
        alignPositionYawSingle(p_es, p_gt, q_es, q_gt, R, t);
    }

    else {

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



void AlignTrajectory::alignSE3Single(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                              std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
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


void AlignTrajectory::alignSE3(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                        Eigen::Matrix3d &R, Eigen::Vector3d &t, int n_aligned){

    // If we only have one, just use the single alignment
    if (n_aligned == 1){
        alignSE3Single(p_es, p_gt, q_es, q_gt, R, t);
    }
    else {

        // Align using the method of Umeyama

        assert(p_es.size() > 0);

        //Grab poses used for alignment
        std::vector<Eigen::Vector3d> est_pos(p_es.begin(), p_es.begin()+std::min((size_t) n_aligned, p_es.size()));
        std::vector<Eigen::Vector3d> gt_pos(p_gt.begin(), p_gt.begin()+std::min((size_t) n_aligned, p_gt.size()));

        double s;
        AlignUtils::align_umeyama(gt_pos, est_pos, R,t,s, true, false);
    }


}


void AlignTrajectory::alignSIM3(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                         std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                         Eigen::Matrix3d &R, Eigen::Vector3d &t, double &s, int n_aligned){


    //Grab poses used for alignment
    assert(p_es.size() > 0);
    std::vector<Eigen::Vector3d> est_pos(p_es.begin(), p_es.begin()+std::min((size_t) n_aligned, p_es.size()));
    std::vector<Eigen::Vector3d> gt_pos(p_gt.begin(), p_gt.begin()+std::min((size_t) n_aligned, p_gt.size()));

    // Align using the method of Umeyama
    AlignUtils::align_umeyama(gt_pos, est_pos, R,t,s, false, false);
}



void AlignTrajectory::alignBodySE3Single(std::vector<Eigen::Vector3d> &p_es, std::vector<Eigen::Vector3d> &p_gt,
                                  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_es,
                                  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &q_gt,
                                  Eigen::Matrix3d &R, Eigen::Vector3d &t){

    // Get initial poses
    Eigen::Vector4d q_es_0 = q_es[0];
    Eigen::Vector3d p_es_0 = p_es[0];

    Eigen::Vector4d q_gt_0 = q_gt[0];
    Eigen::Vector3d p_gt_0 = p_gt[0];

    Eigen::Matrix<double,4,4> T_I_to_W = Eigen::Matrix<double,4,4>::Identity();
    T_I_to_W.block(0,0,3,3) = AlignUtils::quat_2_Rot(q_es_0).transpose();
    T_I_to_W.block(0,3,3,1) = p_es_0;

    Eigen::Matrix<double,4,4> T_B_to_W = Eigen::Matrix<double,4,4>::Identity();
    T_B_to_W.block(0,0,3,3) = AlignUtils::quat_2_Rot(q_gt_0).transpose();
    T_B_to_W.block(0,3,3,1) = p_gt_0;


    Eigen::Matrix<double,4,4> T_B_to_I = T_I_to_W.inverse()*T_B_to_W;

    R = T_B_to_I.block(0,0,3,3);
    t = T_B_to_I.block(0,3,3,1);

}


