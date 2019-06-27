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
    Simulator sim(nh);

    // Continue to simulate until we have processed all the measurements
    while(ros::ok() && sim.ok()) {

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim.get_next_imu(time_imu, wm, am);
        if(hasimu) {
            cout << "new imu measurement = " << std::setprecision(15) << time_imu << std::setprecision(3) << " | w = " << wm.norm() << " | a = " << am.norm() << endl;
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<size_t> featids;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> featuvs;
        bool hascam = sim.get_next_cam(time_cam, camids, featids, featuvs);
        if(hascam) {
            cout << "new cam measurement = " << std::setprecision(15) << time_cam << std::setprecision(3) << " | " << featids.size() << " cameras" << endl;
        }

    }


    // Done!
    return EXIT_SUCCESS;


}




