

/**
 * @namespace ov_eval
 *
 * This project contains the key evaluation and research scripts for localization methods.
 * These come from the necessity of trying to quantify the accuracy of the estimated trajectory while
 * also allowing for the comparision to other methods.
 * The key methods that we have included are:
 *
 * - Absolute trajectory error
 * - Relative pose error (for varying segment lengths)
 * - Pose to text file recorder
 *
 * The absolute and relative error scripts have been implemented in C++ to allow for fast computation on multiple runs.
 * We recommend that people look at the [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) toolbox provided by Zhang and Scaramuzza.
 * For a background we recommend reading their [A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf) @cite Zhang2018IROS
 * and its use in [A Benchmark Comparison of Monocular Visual-Inertial Odometry Algorithms for Flying Robots](http://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf) @cite Delmerico2018ICRA.
 *
 */
namespace ov_eval { }





