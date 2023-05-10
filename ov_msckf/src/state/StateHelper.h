/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#ifndef OV_MSCKF_STATE_HELPER_H
#define OV_MSCKF_STATE_HELPER_H

#include <Eigen/Eigen>
#include <memory>

namespace ov_type {
class Type;
} // namespace ov_type

namespace ov_msckf {

class State;

/**
 * @brief Helper which manipulates the State and its covariance.
 *
 * In general, this class has all the core logic for an Extended Kalman Filter (EKF)-based system.
 * This has all functions that change the covariance along with addition and removing elements from the state.
 * All functions here are static, and thus are self-contained so that in the future multiple states could be tracked and updated.
 * We recommend you look directly at the code for this class for clarity on what exactly we are doing in each and the matching documentation
 * pages.
 */
class StateHelper {

public:
  /**
   * @brief Performs EKF propagation of the state covariance.
   *
   * The mean of the state should already have been propagated, thus just moves the covariance forward in time.
   * The new states that we are propagating the old covariance into, should be **contiguous** in memory.
   * The user only needs to specify the sub-variables that this block is a function of.
   * \f[
   * \tilde{\mathbf{x}}' =
   * \begin{bmatrix}
   * \boldsymbol\Phi_1 &
   * \boldsymbol\Phi_2 &
   * \boldsymbol\Phi_3
   * \end{bmatrix}
   * \begin{bmatrix}
   * \tilde{\mathbf{x}}_1 \\
   * \tilde{\mathbf{x}}_2 \\
   * \tilde{\mathbf{x}}_3
   * \end{bmatrix}
   * +
   * \mathbf{n}
   * \f]
   *
   * @param state Pointer to state
   * @param order_NEW Contiguous variables that have evolved according to this state transition
   * @param order_OLD Variable ordering used in the state transition
   * @param Phi State transition matrix (size order_NEW by size order_OLD)
   * @param Q Additive state propagation noise matrix (size order_NEW by size order_NEW)
   */
  static void EKFPropagation(std::shared_ptr<State> state, const std::vector<std::shared_ptr<ov_type::Type>> &order_NEW,
                             const std::vector<std::shared_ptr<ov_type::Type>> &order_OLD, const Eigen::MatrixXd &Phi,
                             const Eigen::MatrixXd &Q);

  /**
   * @brief Performs EKF update of the state (see @ref linear-meas page)
   * @param state Pointer to state
   * @param H_order Variable ordering used in the compressed Jacobian
   * @param H Condensed Jacobian of updating measurement
   * @param res Residual of updating measurement
   * @param R Updating measurement covariance
   */
  static void EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<ov_type::Type>> &H_order, const Eigen::MatrixXd &H,
                        const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

  /**
   * @brief This will set the initial covaraince of the specified state elements.
   * Will also ensure that proper cross-covariances are inserted.
   * @param state Pointer to state
   * @param covariance The covariance of the system state
   * @param order Order of the covariance matrix
   */
  static void set_initial_covariance(std::shared_ptr<State> state, const Eigen::MatrixXd &covariance,
                                     const std::vector<std::shared_ptr<ov_type::Type>> &order);

  /**
   * @brief For a given set of variables, this will this will calculate a smaller covariance.
   *
   * That only includes the ones specified with all crossterms.
   * Thus the size of the return will be the summed dimension of all the passed variables.
   * Normal use for this is a chi-squared check before update (where you don't need the full covariance).
   *
   * @param state Pointer to state
   * @param small_variables Vector of variables whose marginal covariance is desired
   * @return Marginal covariance of the passed variables
   */
  static Eigen::MatrixXd get_marginal_covariance(std::shared_ptr<State> state,
                                                 const std::vector<std::shared_ptr<ov_type::Type>> &small_variables);

  /**
   * @brief This gets the full covariance matrix.
   *
   * Should only be used during simulation as operations on this covariance will be slow.
   * This will return a copy, so this cannot be used to change the covariance by design.
   * Please use the other interface functions in the StateHelper to progamatically change to covariance.
   *
   * @param state Pointer to state
   * @return Covariance of current state
   */
  static Eigen::MatrixXd get_full_covariance(std::shared_ptr<State> state);

  /**
   * @brief Marginalizes a variable, properly modifying the ordering/covariances in the state
   *
   * This function can support any Type variable out of the box.
   * Right now the marginalization of a sub-variable/type is not supported.
   * For example if you wanted to just marginalize the orientation of a PoseJPL, that isn't supported.
   * We will first remove the rows and columns corresponding to the type (i.e. do the marginalization).
   * After we update all the type ids so that they take into account that the covariance has shrunk in parts of it.
   *
   * @param state Pointer to state
   * @param marg Pointer to variable to marginalize
   */
  static void marginalize(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> marg);

  /**
   * @brief Clones "variable to clone" and places it at end of covariance
   * @param state Pointer to state
   * @param variable_to_clone Pointer to variable that will be cloned
   */
  static std::shared_ptr<ov_type::Type> clone(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> variable_to_clone);

  /**
   * @brief Initializes new variable into covariance.
   *
   * Uses Givens to separate into updating and initializing systems (therefore system must be fed as isotropic).
   * If you are not isotropic first whiten your system (TODO: we should add a helper function to do this).
   * If your H_L Jacobian is already directly invertable, the just call the initialize_invertible() instead of this function.
   * Please refer to @ref update-delay page for detailed derivation.
   *
   * @param state Pointer to state
   * @param new_variable Pointer to variable to be initialized
   * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
   * @param H_R Jacobian of initializing measurements wrt variables in H_order
   * @param H_L Jacobian of initializing measurements wrt new variable
   * @param R Covariance of initializing measurements (isotropic)
   * @param res Residual of initializing measurements
   * @param chi_2_mult Value we should multiply the chi2 threshold by (larger means it will be accepted more measurements)
   */
  static bool initialize(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> new_variable,
                         const std::vector<std::shared_ptr<ov_type::Type>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L,
                         Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult);

  /**
   * @brief Initializes new variable into covariance (H_L must be invertible)
   *
   * Please refer to @ref update-delay page for detailed derivation.
   * This is just the update assuming that H_L is invertable (and thus square) and isotropic noise.
   *
   * @param state Pointer to state
   * @param new_variable Pointer to variable to be initialized
   * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
   * @param H_R Jacobian of initializing measurements wrt variables in H_order
   * @param H_L Jacobian of initializing measurements wrt new variable (needs to be invertible)
   * @param R Covariance of initializing measurements
   * @param res Residual of initializing measurements
   */
  static void initialize_invertible(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> new_variable,
                                    const std::vector<std::shared_ptr<ov_type::Type>> &H_order, const Eigen::MatrixXd &H_R,
                                    const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res);

  /**
   * @brief Augment the state with a stochastic copy of the current IMU pose
   *
   * After propagation, normally we augment the state with an new clone that is at the new update timestep.
   * This augmentation clones the IMU pose and adds it to our state's clone map.
   * If we are doing time offset calibration we also make our cloning a function of the time offset.
   * Time offset logic is based on Li and Mourikis @cite Li2014IJRR.
   *
   * We can write the current clone at the true imu base clock time as the
   * follow: \f{align*}{
   * {}^{I_{t+t_d}}_G\bar{q} &= \begin{bmatrix}\frac{1}{2} {}^{I_{t+\hat{t}_d}}\boldsymbol\omega \tilde{t}_d \\
   * 1\end{bmatrix}\otimes{}^{I_{t+\hat{t}_d}}_G\bar{q} \\
   * {}^G\mathbf{p}_{I_{t+t_d}} &= {}^G\mathbf{p}_{I_{t+\hat{t}_d}} + {}^G\mathbf{v}_{I_{t+\hat{t}_d}}\tilde{t}_d
   * \f}
   * where we say that we have propagated our state up to the current estimated true imaging time for the current image,
   * \f${}^{I_{t+\hat{t}_d}}\boldsymbol\omega\f$ is the angular velocity at the end of propagation with biases removed.
   * This is off by some smaller error, so to get to the true imaging time in the imu base clock, we can append some small timeoffset error.
   * Thus the Jacobian in respect to our time offset during our cloning procedure is the following:
   * \f{align*}{
   * \frac{\partial {}^{I_{t+t_d}}_G\tilde{\boldsymbol\theta}}{\partial \tilde{t}_d} &= {}^{I_{t+\hat{t}_d}}\boldsymbol\omega \\
   * \frac{\partial {}^G\tilde{\mathbf{p}}_{I_{t+t_d}}}{\partial \tilde{t}_d} &= {}^G\mathbf{v}_{I_{t+\hat{t}_d}}
   * \f}
   *
   * @param state Pointer to state
   * @param last_w The estimated angular velocity at cloning time (used to estimate imu-cam time offset)
   */
  static void augment_clone(std::shared_ptr<State> state, Eigen::Matrix<double, 3, 1> last_w);

  /**
   * @brief Remove the oldest clone, if we have more then the max clone count!!
   *
   * This will marginalize the clone from our covariance, and remove it from our state.
   * This is mainly a helper function that we can call after each update.
   * It will marginalize the clone specified by State::margtimestep() which should return a clone timestamp.
   *
   * @param state Pointer to state
   */
  static void marginalize_old_clone(std::shared_ptr<State> state);

  /**
   * @brief Marginalize bad SLAM features
   * @param state Pointer to state
   */
  static void marginalize_slam(std::shared_ptr<State> state);

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  StateHelper() {}
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_HELPER_H
