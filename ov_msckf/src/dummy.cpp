

/**
 * @namespace ov_msckf
 *
 * This is an implementation of a [Multi-State Constraint Kalman Filter (MSCKF)](https://ieeexplore.ieee.org/document/4209642) which leverages inertial and visual feature information.
 * We want to stress that this is not a "vanilla" implementation of the filter and instead has many more features and improvements over the original.
 * In additional we have a modular type system which allows us to initialize and marginalizing variables out of state with ease.
 * The key features of the system are the following:
 *
 * - Sliding stochastic imu clones
 * - First estimate Jacobians
 * - Camera intrinsics and extrinsic online calibration
 * - Time offset between camera and imu calibration
 * - Standard MSCKF features with nullspace projection
 * - 3d slam feature support (with five different representations)
 * - Generic simulation of trajectories through and environment
 *
 * We suggest those that are interested to first checkout the State and Propagator which should provide a nice introduction to the code.
 * Both the slam and msckf features leverage the same Jacobian code, and thus we also recommend looking at the UpdaterHelper class for details on that.
 *
 */
namespace ov_msckf { }





