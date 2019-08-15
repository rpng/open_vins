/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "AlignUtils.h"

using namespace ov_eval;


void AlignUtils::align_umeyama(const std::vector<Eigen::Matrix<double, 3, 1>> &data,
                               const std::vector<Eigen::Matrix<double, 3, 1>> &model,
                               Eigen::Matrix<double, 3, 3> &R, Eigen::Matrix<double, 3, 1> &t,
                               double &s, bool known_scale, bool yaw_only){

    assert(model.size() == data.size());

    // Substract mean of each trajectory
    Eigen::Matrix<double, 3, 1> mu_M = get_mean(model);
    Eigen::Matrix<double, 3, 1> mu_D = get_mean(data);
    std::vector<Eigen::Matrix<double, 3, 1>> model_zerocentered, data_zerocentered;
    for (size_t i = 0; i < model.size(); i++) {
        model_zerocentered.push_back(model[i] - mu_M);
        data_zerocentered.push_back(data[i] - mu_D);
    }

    // Get correlation matrix
    double n = model.size();
    Eigen::Matrix<double, 3, 3> C = Eigen::Matrix<double, 3, 3>::Zero();
    for (size_t i = 0; i < model_zerocentered.size(); i++) {
        C.noalias() += model_zerocentered[i] * data_zerocentered[i].transpose();
    }
    C *= 1.0 / n;


    // Get data sigma
    double sigma2 = 0;
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

    //If only yaw, use that specific solver (optimizes over yaw angle)
    if (yaw_only) {
        Eigen::Matrix<double, 3, 3> rot_C = n * C.transpose();
        double theta = AlignUtils::get_best_yaw(rot_C);
        R = Math::rot_z(theta);
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

    //std::cout << "R- " << std::endl << R << std::endl;
    //std::cout << "t- " << std::endl << t << std::endl;
}

