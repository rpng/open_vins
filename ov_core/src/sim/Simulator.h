#ifndef OV_CORE_SIMULATOR_H
#define OV_CORE_SIMULATOR_H


#include <fstream>
#include <sstream>
#include <random>
#include <string>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>



/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {



    /**
     * @brief B-Spline which performs interpolation over SE(3) manifold.
     *
     * This class implements the b-spline functionality that allows for interpolation over the \f$\mathbb{SE}(3)\f$ manifold.
     * This is based off of the derivations from [Continuous-Time Visual-Inertial Odometry for Event Cameras](https://ieeexplore.ieee.org/abstract/document/8432102/)
     * and [A Spline-Based Trajectory Representation for Sensor Fusion and Rolling Shutter Cameras](https://link.springer.com/article/10.1007/s11263-015-0811-3)
     * with some additional derivations being avalible in [these notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf).
     * The use of b-splines for \f$\mathbb{SE}(3)\f$ interpolation has the following properties:
     *
     * 1. Local control, allowing the system to function online as well as in batch
     * 2. \f$C^2\f$-continuity to enable inertial predictions and calculations
     * 3. Good approximation of minimal torque trajectories
     * 4. A parameterization of rigid-body motion devoid of singularities
     *
     * The key idea is to convert a set of trajectory points into a continuous-time *uniform cubic cumulative* b-spline.
     * As compared to standard b-spline representations, the cumulative form ensures local continuity which is needed for on-manifold interpolation.
     * We leverage the cubic b-spline to ensure \f$C^2\f$-continuity to ensure that we can calculate accelerations at any point along the trajectory.
     *
     */
    class Simulator {

    public:


        /**
         * @brief Default constructor
         */
        Simulator(std::string path_traj) {
            load_data(path_traj);
        }





    protected:


        /**
         * @brief This will load the trajectory into memory.
         * @param path_traj Path to the trajectory file that we want to read in.
         */
        void load_data(std::string path_traj);


        /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
        std::vector<Eigen::Matrix<double,8,1>,Eigen::aligned_allocator<Eigen::Matrix<double,8,1>>> traj_data;


    };


}

#endif //OV_CORE_SIMULATOR_H
