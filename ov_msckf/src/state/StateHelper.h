#ifndef OV_MSCKF_STATE_HELPER_H
#define OV_MSCKF_STATE_HELPER_H


#include "State.h"


/**
 * @namespace ov_msckf
 * @brief The Open VINS MSCKF
 */
namespace ov_msckf {


    /**
     * @brief Helper which manipulates the State its covariance.
     *
     * In general, this class has all the core logic for the MSCKF system in it.
     * This has all functions that change the covariance along with addition and removing elements from the state.
     * All functions here are static, and thus are self-contained so that in the future multiple states could be tracked and updated.
     * We recommend you look directly at the code for this class for clarity on what exactly we are doing in each.
     */
    class StateHelper {

    public:


        /**
         * @brief Performs EKF update of the state
         *
         * @param state Pointer to state
         * @param H_order Variable ordering used in the compressed Jacobian
         * @param H Condensed Jacobian of updating measurement
         * @param res residual of updating measurement
         * @param R updating measurement covariance
         */
        static void EKFUpdate(State *state, const std::vector<Type *> &H_order, const Eigen::MatrixXd &H,
                              const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

        /**
        * @brief For a given set of variables, this will this will calculate a smaller covariance.
        *
        * That only includes the ones specified with all crossterms.
        * Thus the size of the return will be the summed dimension of all the passed variables.
        * Normal use for this is a chi-squared check before update (where you don't need the full covariance).
        *
        * @param small_variables Vector of variables whose marginal covariance is desired
        * @return marginal covariance of the passed variables
        */
        static Eigen::MatrixXd get_marginal_covariance(State *state, const std::vector<Type *> &small_variables);

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
        static void marginalize(State *state, Type *marg);


        /**
         * @brief Clones "variable to clone" and places it at end of covariance
         *
         * @param state Pointer to state
         * @param variable_to_clone Pointer to variable that will be cloned
         */
        static Type* clone(State *state, Type *variable_to_clone);


        /**
         * @brief Initializes new variable into covariance.
         *
         * Uses Givens to separate into updating and initializing systems (therefore system must be fed as isotropic).
         * If you are not isotropic first whiten your system (TODO: we should add a helper function to do this).
         * If your H_L Jacobian is already directly invertable, the just call the initialize_invertible() instead of this function.
         *
         * @param state Pointer to state
         * @param new_variable Pointer to variable to be initialized
         * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
         * @param H_R Jacobian of initializing measurements wrt variables in H_order
         * @param H_L Jacobian of initializing measurements wrt new variable
         * @param R Covariance of initializing measurements (isotropic)
         * @param res Residual of initializing measurements
         */
        static void initialize(State *state, Type *new_variable, const std::vector<Type *> &H_order, Eigen::MatrixXd &H_R,
                               Eigen::MatrixXd &H_L, Eigen::MatrixXd &R, Eigen::VectorXd &res);


        /**
         * @brief Initializes new variable into covariance (H_L must be invertible)
         *
         * @param state Pointer to state
         * @param new_variable Pointer to variable to be initialized
         * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
         * @param H_R Jacobian of initializing measurements wrt variables in H_order
         * @param H_L Jacobian of initializing measurements wrt new variable (needs to be invertible)
         * @param R Covariance of initializing measurements
         * @param res Residual of initializing measurements
         */
        static void initialize_invertible(State *state, Type *new_variable, const std::vector<Type *> &H_order, const Eigen::MatrixXd &H_R,
                                          const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res);


        /**
         * @brief Augment the state with a stochastic copy of the current IMU pose
         *
         * After propagation, normally we augment the state with an new clone that is at the new update timestep.
         * This augmentation clones the IMU pose and adds it to our state's clone map.
         * If we are doing time offset calibration we also make our cloning a function of the time offset.
         * Time offset logic is based on Mingyang Li and Anastasios I. Mourikis paper: http://journals.sagepub.com/doi/pdf/10.1177/0278364913515286
         *
         * @param state Pointer to state
         * @param last_w The estimated angular velocity at cloning time (used to estimate imu-cam time offset)
         */
        static void augment_clone(State *state, Eigen::Matrix<double, 3, 1> last_w);


        /**
         * @brief Remove the oldest clone, if we have more then the max clone count!!
         *
         * This will marginalize the clone from our covariance, and remove it from our state.
         * This is mainly a helper function that we can call after each update.
         * It will marginalize the clone specified by State::margtimestep() which should return a clone timestamp.
         *
         * @param state Pointer to state
         */
        static void marginalize_old_clone(State *state) {
            if ((int) state->n_clones() > state->options().max_clone_size) {
                double margTime = state->margtimestep();
                StateHelper::marginalize(state, state->get_clone(margTime));
                // Note that the marginalizer should have already deleted the clone
                // Thus we just need to remove the pointer to it from our state
                state->erase_clone(margTime);
            }
        }


    private:

        /**
         * All function in this class should be static.
         * Thus an instance of this class cannot be created.
         */
        StateHelper() {}


    };

}

#endif //OV_MSCKF_STATE_HELPER_H
