#include "AlignUtils.h"

using namespace ov_eval;


void AlignUtils::align_umeyama(const std::vector<Eigen::Matrix<double, 3, 1>> &model,
                             const std::vector<Eigen::Matrix<double, 3, 1>> &data,
                             Eigen::Matrix<double, 3, 3> &R, Eigen::Matrix<double, 3, 1> &t,
                             double &s, bool known_scale, bool yaw_only){

    assert(model.size() == data.size());

    // substract mean
    Eigen::Matrix<double, 3, 1> mu_M = get_mean(model);
    Eigen::Matrix<double, 3, 1> mu_D = get_mean(data);


    std::vector<Eigen::Matrix<double, 3, 1>> model_zerocentered, data_zerocentered;
    for (size_t i = 0; i < model.size(); i++) {
        model_zerocentered.push_back(model[i] - mu_M);
        data_zerocentered.push_back(data[i] - mu_D);
    }

    double n = model.size();

    Eigen::Matrix<double, 3, 3> C = Eigen::Matrix<double, 3, 3>::Zero();

    // Get correlation
    for (size_t i = 0; i < model_zerocentered.size(); i++) {
        C.noalias() += model_zerocentered[i] * data_zerocentered[i].transpose();
    }

    C *= 1.0 / n;


    double sigma2 = 0;

    //Get data sigma
    for (size_t i = 0; i < data_zerocentered.size(); i++) {
        sigma2 += data_zerocentered[i].dot(data_zerocentered[i]);
    }

    sigma2 *= 1.0 / n;


    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(C, Eigen::ComputeFullV | Eigen::ComputeFullU);

    Eigen::Matrix<double, 3, 3> U_svd = svd.matrixU();
    Eigen::Matrix<double, 3, 1> D_svd = svd.singularValues();
    Eigen::Matrix<double, 3, 3> V_svd = svd.matrixV();

    Eigen::Matrix<double, 3, 3> S = Eigen::Matrix<double, 3, 3>::Identity();
    if (U_svd.determinant() * V_svd.determinant() < 0) {
        S(2, 2) = -1;
    }

    if (yaw_only) {

        //If only yaw, use that specific solver (optimizes over yaw angle)
        Eigen::Matrix<double, 3, 3> rot_C = n * C.transpose();

        double theta = get_best_yaw(rot_C);
        R = rot_z(theta);
    } else {

        //Get best full rotation
        R.noalias() = U_svd * S * V_svd.transpose();
    }

    //If known scale, fix it
    if (known_scale) {
        s = 1;
    } else {

        //Get best scale
        s = 1.0 / sigma2 * (D_svd.asDiagonal() * S).trace();
    }

    // Get best translation
    t.noalias() = mu_M - s * R * mu_D;

    std::cout << "R- " << std::endl << R << std::endl;
    std::cout << "t- " << std::endl << t << std::endl;
}

