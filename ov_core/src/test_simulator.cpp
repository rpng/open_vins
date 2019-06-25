#include <cmath>
#include <vector>
#include <deque>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>


#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "sim/Simulator.h"


using namespace ov_core;



// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_tracking");
    ros::NodeHandle nh("~");


    // Create the simulator
    std::string trajpath = "/home/patrick/workspace/catkin_ws_ov/src/rpg_trajectory_evaluation/results/laptop/vio_mono/laptop_vio_mono_MH_01/stamped_traj_estimate.txt";
    Simulator sim(trajpath);








    // Done!
    return EXIT_SUCCESS;


}




