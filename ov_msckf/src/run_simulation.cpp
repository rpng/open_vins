

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
#include <boost/filesystem.hpp>

#include "sim/Simulator.h"
#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"


using namespace ov_msckf;


Simulator* sim;
VioManager* sys;
RosVisualizer* viz;


//helper function that saves calibration estimates and gt to file
std::ofstream of_calib_gt, of_calib_est, of_calib_std;
void save_calibration_to_file_for_paper();



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
                //save_calibration_to_file_for_paper();
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





void save_calibration_to_file_for_paper() {

    // If the file is not open, then open the file
    if(!of_calib_gt.is_open()) {

        // files we will open
        std::string filepath_gt = "/home/patrick/github/pubs_data/pgeneva/2019_openvins/sim_calibration/singlerun/calibration_groundtruth.txt";
        std::string filepath_est = "/home/patrick/github/pubs_data/pgeneva/2019_openvins/sim_calibration/singlerun/calibration_estimate.txt";
        std::string filepath_std = "/home/patrick/github/pubs_data/pgeneva/2019_openvins/sim_calibration/singlerun/calibration_deviation.txt";

        // If it exists, then delete it
        if(boost::filesystem::exists(filepath_gt))
            boost::filesystem::remove(filepath_gt);
        if(boost::filesystem::exists(filepath_est))
            boost::filesystem::remove(filepath_est);
        if(boost::filesystem::exists(filepath_std))
            boost::filesystem::remove(filepath_std);

        // Open the files
        of_calib_gt.open(filepath_gt.c_str());
        of_calib_est.open(filepath_est.c_str());
        of_calib_std.open(filepath_std.c_str());
        of_calib_gt << "# timestamp(s) cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;
        of_calib_est << "# timestamp(s) cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;
        of_calib_std << "# timestamp(s) cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans .... etc" << std::endl;

    }

    // Get the current groundtruth calibration values
    of_calib_gt.precision(5);
    of_calib_gt.setf(std::ios::fixed, std::ios::floatfield);
    of_calib_gt << sys->get_state()->timestamp() << " ";
    of_calib_gt.precision(7);
    of_calib_gt << sim->get_true_imucamdt() << " ";
    of_calib_gt.precision(0);
    of_calib_gt << sim->get_num_cameras() << " ";
    of_calib_gt.precision(6);

    // Get the current estimate calibration values
    of_calib_est.precision(5);
    of_calib_est.setf(std::ios::fixed, std::ios::floatfield);
    of_calib_est << sys->get_state()->timestamp() << " ";
    of_calib_est.precision(7);
    of_calib_est << sys->get_state()->calib_dt_CAMtoIMU()->value()(0) << " ";
    of_calib_est.precision(0);
    of_calib_est << sim->get_num_cameras() << " ";
    of_calib_est.precision(6);

    // Get the current std values
    of_calib_std.precision(5);
    of_calib_std.setf(std::ios::fixed, std::ios::floatfield);
    of_calib_std << sys->get_state()->timestamp() << " ";
    of_calib_std << std::sqrt(sys->get_state()->Cov()(sys->get_state()->calib_dt_CAMtoIMU()->id(),sys->get_state()->calib_dt_CAMtoIMU()->id())) << " ";
    of_calib_std.precision(0);
    of_calib_std << sim->get_num_cameras() << " ";
    of_calib_std.precision(6);

    // Write the camera values to file
    for(int i=0; i<sim->get_num_cameras(); i++) {

        // GT: intrinsics
        of_calib_gt << sim->get_true_intrinsics().at(i)(0) << " " << sim->get_true_intrinsics().at(i)(1) << " " << sim->get_true_intrinsics().at(i)(2) << " " << sim->get_true_intrinsics().at(i)(3) << " ";
        of_calib_gt << sim->get_true_intrinsics().at(i)(4) << " " << sim->get_true_intrinsics().at(i)(5) << " " << sim->get_true_intrinsics().at(i)(6) << " " << sim->get_true_intrinsics().at(i)(7) << " ";
        // GT: rotation and position
        of_calib_gt << sim->get_true_extrinsics().at(i)(0) << " " << sim->get_true_extrinsics().at(i)(1) << " " << sim->get_true_extrinsics().at(i)(2) << " " << sim->get_true_extrinsics().at(i)(3) << " ";
        of_calib_gt << sim->get_true_extrinsics().at(i)(4) << " " << sim->get_true_extrinsics().at(i)(5) << " " << sim->get_true_extrinsics().at(i)(6) << " ";

        // EST: intrinsics
        of_calib_est << sys->get_state()->get_intrinsics_CAM(i)->value()(0) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(1) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(2) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(3) << " ";
        of_calib_est << sys->get_state()->get_intrinsics_CAM(i)->value()(4) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(5) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(6) << " " << sys->get_state()->get_intrinsics_CAM(i)->value()(7) << " ";
        // EST: rotation and position
        of_calib_est << sys->get_state()->get_calib_IMUtoCAM(i)->value()(0) << " " << sys->get_state()->get_calib_IMUtoCAM(i)->value()(1) << " " << sys->get_state()->get_calib_IMUtoCAM(i)->value()(2) << " " << sys->get_state()->get_calib_IMUtoCAM(i)->value()(3) << " ";
        of_calib_est << sys->get_state()->get_calib_IMUtoCAM(i)->value()(4) << " " << sys->get_state()->get_calib_IMUtoCAM(i)->value()(5) << " " << sys->get_state()->get_calib_IMUtoCAM(i)->value()(6) << " ";

        // Covariance
        int index_in = sys->get_state()->get_intrinsics_CAM(i)->id();
        of_calib_std << std::sqrt(sys->get_state()->Cov()(index_in+0,index_in+0)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+1,index_in+1)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+2,index_in+2)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+3,index_in+3)) << " ";
        of_calib_std << std::sqrt(sys->get_state()->Cov()(index_in+4,index_in+4)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+5,index_in+5)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+6,index_in+6)) << " " << std::sqrt(sys->get_state()->Cov()(index_in+7,index_in+7)) << " ";
        int index_ex = sys->get_state()->get_calib_IMUtoCAM(i)->id();
        of_calib_std << std::sqrt(sys->get_state()->Cov()(index_ex+0,index_ex+0)) << " " << std::sqrt(sys->get_state()->Cov()(index_ex+1,index_ex+1)) << " " << std::sqrt(sys->get_state()->Cov()(index_ex+2,index_ex+2)) << " ";
        of_calib_std << std::sqrt(sys->get_state()->Cov()(index_ex+3,index_ex+3)) << " " << std::sqrt(sys->get_state()->Cov()(index_ex+4,index_ex+4)) << " " << std::sqrt(sys->get_state()->Cov()(index_ex+5,index_ex+5)) << " ";

    }


    // New line
    of_calib_gt << endl;
    of_calib_est << endl;
    of_calib_std << endl;



}













