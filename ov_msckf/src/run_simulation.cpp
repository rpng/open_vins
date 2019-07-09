

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include "sim/Simulator.h"
#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"


using namespace ov_msckf;


Simulator* sim;
VioManager* sys;
RosVisualizer* viz;


// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "test_simulation");
    ros::NodeHandle nh("~");

    // Create our VIO system
    sim = new Simulator(nh);
    sys = new VioManager(nh);
    viz = new RosVisualizer(nh, sys, sim);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Get initial state
    Eigen::Matrix<double, 17, 1> imustate;
    bool success = sim->get_state(sim->current_timestamp(),imustate);
    if(!success) {
        ROS_ERROR("[SIM]: Could not initialize the filter to the first state");
        ROS_ERROR("[SIM]: Did the simulator load properly???");
        std::exit(EXIT_FAILURE);
    }

    // Initialize our filter with the groundtruth
    sys->initialize_with_gt(imustate);
    viz->visualize();


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Buffer our camera image
    double buffer_timecam = -1;
    std::vector<int> buffer_camids;
    std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> buffer_feats;

    // Step through the rosbag
    while(ros::ok() && sim->ok()) {

        // IMU: get the next simulated IMU measurement if we have it
        double time_imu;
        Eigen::Vector3d wm, am;
        bool hasimu = sim->get_next_imu(time_imu, wm, am);
        if(hasimu) {
            sys->feed_measurement_imu(time_imu, wm, am);
        }

        // CAM: get the next simulated camera uv measurements if we have them
        double time_cam;
        std::vector<int> camids;
        std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> feats;
        bool hascam = sim->get_next_cam(time_cam, camids, feats);
        if(hascam) {
            if(buffer_timecam != -1) {
                sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
                viz->visualize();
            }
            buffer_timecam = time_cam;
            buffer_camids = camids;
            buffer_feats = feats;
        }

    }


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sim;
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;

}


















