

#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


#include "alignment/Trajectory.h"
#include "utils/Loader.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "error_comparison");

    // Ensure we have a path
    if(argc < 4) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval error_comparison <align_mode> <folder_groundtruth> <folder_algorithms>");
        std::exit(EXIT_FAILURE);
    }

    // List the groundtruth files in this folder
    std::string path_gts(argv[2]);
    std::vector<boost::filesystem::path> path_groundtruths;
    for(const auto& p: boost::filesystem::recursive_directory_iterator(path_gts)) {
        if(p.path().extension() == ".txt") {
            path_groundtruths.push_back(p.path());
        }
    }
    std::sort(path_groundtruths.begin(), path_groundtruths.end());

    // Try to load our paths
    for(size_t i=0; i<path_groundtruths.size(); i++) {
        // Load it!
        std::vector<double> times;
        std::vector<Eigen::Matrix<double,7,1>> poses;
        std::vector<Eigen::Matrix3d> cov_ori, cov_pos;
        ov_eval::Loader::load_data(path_groundtruths.at(i).string(), times, poses, cov_ori, cov_pos);
        // Print its length and stats
        double length = ov_eval::Loader::get_total_length(poses);
        ROS_INFO("[COMP]: %d poses in %s => length of %.2f meters",(int)times.size(),path_groundtruths.at(i).filename().c_str(),length);
    }

    // Get the algorithms we will process
    // Also create empty statistic objects for each of our datasets
    std::string path_algos(argv[3]);
    std::vector<boost::filesystem::path> path_algorithms;
    for(const auto& entry : boost::filesystem::directory_iterator(path_algos)) {
        if(boost::filesystem::is_directory(entry)) {
            path_algorithms.push_back(entry.path());

        }
    }
    std::sort(path_algorithms.begin(), path_algorithms.end());


    //===============================================================================
    //===============================================================================
    //===============================================================================


    // Relative pose error segment lengths
    //std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
    std::vector<double> segments = {7.0, 14.0, 21.0, 28.0, 35.0};

    // The overall RPE error calculation for each algorithm type
    std::map<std::string,std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>>> algo_rpe;
    for(const auto& p : path_algorithms) {
        std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> temp;
        for(const auto& len : segments) {
            temp.insert({len,{ov_eval::Statistics(),ov_eval::Statistics()}});
        }
        algo_rpe.insert({p.stem().string(),temp});
    }


    //===============================================================================
    //===============================================================================
    //===============================================================================



    // Loop through each algorithm type
    for(size_t i=0; i<path_algorithms.size(); i++) {

        // Debug print
        ROS_INFO("======================================");
        ROS_INFO("[COMP]: processing %s algorithm", path_algorithms.at(i).stem().c_str());

        // Get the list of datasets this algorithm records
        std::map<std::string,boost::filesystem::path> path_algo_datasets;
        for(auto& entry : boost::filesystem::directory_iterator(path_algorithms.at(i))) {
            if(boost::filesystem::is_directory(entry)) {
                path_algo_datasets.insert({entry.path().stem().string(),entry.path()});
            }
        }

        // Loop through our list of groundtruth datasets, and see if we have it
        for(size_t j=0; j<path_groundtruths.size(); j++) {

            // Check if we have runs for this dataset
            if(path_algo_datasets.find(path_groundtruths.at(j).stem().string())==path_algo_datasets.end()) {
                ROS_ERROR("[COMP]: %s dataset does not have any runs for %s!!!!!",path_algorithms.at(i).stem().c_str(),path_groundtruths.at(j).stem().c_str());
                continue;
            }

            // Debug print
            ROS_INFO("[COMP]: processing %s algorithm => %s dataset", path_algorithms.at(i).stem().c_str(),path_groundtruths.at(j).stem().c_str());

            // Errors for this specific dataset (i.e. our averages over the total runs)
            ov_eval::Statistics ate_dataset_ori;
            ov_eval::Statistics ate_dataset_pos;
            std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> rpe_dataset;
            for(const auto& len : segments) {
                rpe_dataset.insert({len,{ov_eval::Statistics(),ov_eval::Statistics()}});
            }

            // Loop though the different runs for this dataset
            for(auto& entry : boost::filesystem::directory_iterator(path_algo_datasets.at(path_groundtruths.at(j).stem().string()))) {
                if(entry.path().extension() == ".txt") {

                    // Our paths
                    std::string dataset = path_groundtruths.at(j).stem().string();
                    std::string path_gttxt = path_groundtruths.at(j).string();
                    std::string path_esttxt = entry.path().string();

                    // Create our trajectory object
                    ov_eval::Trajectory traj(path_esttxt, path_gttxt, argv[1]);

                    // Calculate ATE error for this dataset
                    ov_eval::Statistics error_ori, error_pos;
                    traj.calculate_ate(error_ori, error_pos);
                    ate_dataset_ori.values.push_back(error_ori.rmse);
                    ate_dataset_pos.values.push_back(error_pos.rmse);

                    // Calculate RPE error for this dataset
                    std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> error_rpe;
                    traj.calculate_rpe(segments, error_rpe);
                    for(const auto& elm : error_rpe) {
                        rpe_dataset.at(elm.first).first.values.insert(rpe_dataset.at(elm.first).first.values.end(),elm.second.first.values.begin(),elm.second.first.values.end());
                        rpe_dataset.at(elm.first).first.timestamps.insert(rpe_dataset.at(elm.first).first.timestamps.end(),elm.second.first.timestamps.begin(),elm.second.first.timestamps.end());
                        rpe_dataset.at(elm.first).second.values.insert(rpe_dataset.at(elm.first).second.values.end(),elm.second.second.values.begin(),elm.second.second.values.end());
                        rpe_dataset.at(elm.first).second.timestamps.insert(rpe_dataset.at(elm.first).second.timestamps.end(),elm.second.second.timestamps.begin(),elm.second.second.timestamps.end());
                    }

                }
            }

            // Compute our mean ATE score
            ate_dataset_ori.calculate();
            ate_dataset_pos.calculate();

            // Print stats for this specific dataset
            ROS_INFO("\tATE: mean_ori = %.3f | mean_pos = %.3f",ate_dataset_ori.mean,ate_dataset_pos.mean);
            ROS_INFO("\tATE: std_ori  = %.3f | std_pos  = %.3f",ate_dataset_ori.std,ate_dataset_pos.std);
            for(auto &seg : rpe_dataset) {
                seg.second.first.calculate();
                seg.second.second.calculate();
                ROS_INFO("\tRPE: seg %d - mean_ori = %.3f | mean_pos = %.3f (%d samples)",(int)seg.first,seg.second.first.mean,seg.second.second.mean,(int)seg.second.second.values.size());
                //ROS_INFO("RPE: seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
            }

            // Update the global RPE error stats
            std::string algo = path_algorithms.at(i).stem().string();
            for(const auto& elm : rpe_dataset) {
                algo_rpe.at(algo).at(elm.first).first.values.insert(algo_rpe.at(algo).at(elm.first).first.values.end(),elm.second.first.values.begin(),elm.second.first.values.end());
                algo_rpe.at(algo).at(elm.first).first.timestamps.insert(algo_rpe.at(algo).at(elm.first).first.timestamps.end(),elm.second.first.timestamps.begin(),elm.second.first.timestamps.end());
                algo_rpe.at(algo).at(elm.first).second.values.insert(algo_rpe.at(algo).at(elm.first).second.values.end(),elm.second.second.values.begin(),elm.second.second.values.end());
                algo_rpe.at(algo).at(elm.first).second.timestamps.insert(algo_rpe.at(algo).at(elm.first).second.timestamps.end(),elm.second.second.timestamps.begin(),elm.second.second.timestamps.end());
            }

        }

    }



    //===============================================================================
    //===============================================================================
    //===============================================================================


    // Finally print the RPE for all the runs
    ROS_INFO(" ");
    for(auto &algo : algo_rpe) {
        ROS_INFO("============================================");
        ROS_INFO("algorithm %s",algo.first.c_str());
        for(auto &seg : algo.second) {
            seg.second.first.calculate();
            seg.second.second.calculate();
            ROS_INFO("\tRPE: seg %d - mean_ori = %.4f | mean_pos = %.4f (%d samples)",(int)seg.first,seg.second.first.mean,seg.second.second.mean,(int)seg.second.second.values.size());
            //ROS_INFO("RPE: seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
        }
    }
    ROS_INFO("============================================");


    // Done!
    return EXIT_SUCCESS;

}


