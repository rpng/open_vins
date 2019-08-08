

/**
 * @namespace ov_core
 *
 * This has the core algorithms that all projects within the Open VINS ecosystem leverage.
 * The purpose is to allow for the reuse of code that could be shared between different localization systems (i.e. msckf-based, batch-based, etc.).
 * These algorithms are the foundation which is necessary before we can even write an estimator that can perform localization.
 * The key components of the ov_core codebase are the following:
 *
 * - Closed-form preintegration @cite Eckenhoff2019IJRR
 * - 3d feature initialization
 * - Inertial state initialization
 * - Visual-inertial simulator and SE(3) b-spline
 * - KLT, descriptor, aruco, and simulation feature trackers
 * - Groundtruth dataset reader
 * - Quaternion and other manifold math operations
 *
 * Please take a look at classes that we offer for the user to leverage as each has its own documentation.
 * If you are looking for the estimator please take a look at the ov_msckf project which leverages these algorithms.
 *
 */
namespace ov_core { }





