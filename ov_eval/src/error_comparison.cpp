/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "calc/ResultTrajectory.h"
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

    // ATE summery information
    std::map<std::string,std::vector<std::pair<ov_eval::Statistics,ov_eval::Statistics>>> algo_ate;
    for(const auto& p : path_algorithms) {
        std::vector<std::pair<ov_eval::Statistics,ov_eval::Statistics>> temp;
        for(size_t i=0; i<path_groundtruths.size(); i++) {
            temp.push_back({ov_eval::Statistics(),ov_eval::Statistics()});
        }
        algo_ate.insert({p.stem().string(),temp});
    }

    // Relative pose error segment lengths
    std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0, 48.0};
    //std::vector<double> segments = {7.0, 14.0, 21.0, 28.0, 35.0};
    //std::vector<double> segments = {10.0, 25.0, 50.0, 75.0, 120.0};
    //std::vector<double> segments = {5.0, 15.0, 30.0, 45.0, 60.0};

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
                    ov_eval::ResultTrajectory traj(path_esttxt, path_gttxt, argv[1]);

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
                //ROS_INFO("\tRPE: seg %d - mean_ori = %.3f | mean_pos = %.3f (%d samples)",(int)seg.first,seg.second.first.mean,seg.second.second.mean,(int)seg.second.second.values.size());
                ROS_INFO("\tRPE: seg %d - median_ori = %.4f | median_pos = %.4f (%d samples)",(int)seg.first,seg.second.first.median,seg.second.second.median,(int)seg.second.second.values.size());
                //ROS_INFO("RPE: seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
            }

            // Update the global ATE error stats
            std::string algo = path_algorithms.at(i).stem().string();
            algo_ate.at(algo).at(j).first = ate_dataset_ori;
            algo_ate.at(algo).at(j).second = ate_dataset_pos;

            // Update the global RPE error stats
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

    // Finally print the ATE for all the runs
    ROS_INFO("============================================");
    ROS_INFO("ATE LATEX TABLE");
    ROS_INFO("============================================");
    for(size_t i=0; i<path_groundtruths.size(); i++) {
        std::string gtname = path_groundtruths.at(i).stem().string();
        boost::replace_all(gtname,"_","\\_");
        cout << " & \\textbf{" << gtname << "}";
    }
    cout << " & \\textbf{Average} \\\\\\hline" << endl;
    for(auto &algo : algo_ate) {
        std::string algoname = algo.first;
        boost::replace_all(algoname,"_","\\_");
        cout << algoname;
        double sum_ori = 0.0;
        double sum_pos = 0.0;
        int sum_ct = 0;
        for(auto &seg : algo.second) {
            if(seg.first.values.empty() || seg.second.values.empty()) {
                cout << std::fixed << std::setprecision(3) << " & - / -";
            } else {
                cout << std::fixed << std::setprecision(3) << " & " << seg.first.rmse << " / " << seg.second.rmse;
                sum_ori += seg.first.rmse;
                sum_pos += seg.second.rmse;
                sum_ct++;
            }
        }
        cout << std::fixed << std::setprecision(3) << " & " << sum_ori/sum_ct << " / " << sum_pos/sum_ct;
        cout << " \\\\" << endl;
    }
    ROS_INFO("============================================");


    // Finally print the RPE for all the runs
    ROS_INFO("============================================");
    ROS_INFO("RPE LATEX TABLE");
    ROS_INFO("============================================");
    for(const auto& len : segments) {
        cout << std::fixed << std::setprecision(0) << " & \\textbf{" << len << "m}";
    }
    cout << " \\\\\\hline" << endl;
    for(auto &algo : algo_rpe) {
        std::string algoname = algo.first;
        boost::replace_all(algoname,"_","\\_");
        cout << algoname;
        for(auto &seg : algo.second) {
            seg.second.first.calculate();
            seg.second.second.calculate();
            cout << std::fixed << std::setprecision(3) << " & " << seg.second.first.median << " / " << seg.second.second.median;
        }
        cout << " \\\\" << endl;
    }
    ROS_INFO("============================================");


#ifdef HAVE_PYTHONLIBS

    // Plot line colors
    std::vector<std::string> colors = {"blue","red","black","green","cyan","magenta"};
    std::vector<std::string> linestyle = {"-","--","-."};
    assert(algo_rpe.size() <= colors.size()*linestyle.size());

    // Parameters
    std::map<std::string, std::string> params_rpe;
    params_rpe.insert({"notch","false"});
    params_rpe.insert({"sym",""});

    // Plot this figure
    matplotlibcpp::figure_size(1000, 700);
    matplotlibcpp::subplot(2,1,1);
    matplotlibcpp::title("Relative Orientation Error");

    // Plot each RPE next to each other
    double width = 1.0/(algo_rpe.size()+1);
    double spacing = width/(algo_rpe.size()+1);
    std::vector<double> xticks;
    std::vector<std::string> labels;
    int ct_algo = 0;
    double ct_pos = 0;
    for(auto &algo : algo_rpe) {
        // Start based on what algorithm we are doing
        xticks.clear();
        labels.clear();
        ct_pos = 1+ct_algo*(width+spacing);
        // Loop through each length type
        for(auto &seg : algo.second) {
            xticks.push_back(ct_pos-(algo_rpe.size()*(width+spacing)-width)/2);
            labels.push_back(std::to_string((int)seg.first));
            matplotlibcpp::boxplot(seg.second.first.values, ct_pos, width, colors.at(ct_algo%colors.size()), linestyle.at(ct_algo/colors.size()), params_rpe);
            ct_pos += 1+3*width;
        }
        // Move forward
        ct_algo++;
    }

    // Add "fake" plots for our legend
    ct_algo = 0;
    for(const auto &algo : algo_rpe) {
        std::map<std::string, std::string> params_empty;
        params_empty.insert({"label", algo.first});
        params_empty.insert({"linestyle", linestyle.at(ct_algo/colors.size())});
        params_empty.insert({"color", colors.at(ct_algo%colors.size())});
        std::vector<double> vec_empty;
        matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
        ct_algo++;
    }

    // Plot each RPE next to each other
    matplotlibcpp::xlim(0.5,ct_pos-0.5-3*width);
    matplotlibcpp::xticks(xticks,labels);
    matplotlibcpp::ylabel("orientation error (deg)");
    matplotlibcpp::legend();
    matplotlibcpp::subplot(2,1,2);
    ct_algo = 0;
    ct_pos = 0;
    for(auto &algo : algo_rpe) {
        // Start based on what algorithm we are doing
        ct_pos = 1+ct_algo*(width+spacing);
        // Loop through each length type
        for(auto &seg : algo.second) {
            matplotlibcpp::boxplot(seg.second.second.values, ct_pos, width, colors.at(ct_algo%colors.size()), linestyle.at(ct_algo/colors.size()), params_rpe);
            ct_pos += 1+3*width;
        }
        // Move forward
        ct_algo++;
    }

    // Add "fake" plots for our legend
    ct_algo = 0;
    for(const auto &algo : algo_rpe) {
        std::map<std::string, std::string> params_empty;
        params_empty.insert({"label", algo.first});
        params_empty.insert({"linestyle", linestyle.at(ct_algo/colors.size())});
        params_empty.insert({"color", colors.at(ct_algo%colors.size())});
        std::vector<double> vec_empty;
        matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
        ct_algo++;
    }

    // Display to the user
    matplotlibcpp::xlim(0.5,ct_pos-0.5-3*width);
    matplotlibcpp::xticks(xticks,labels);
    matplotlibcpp::title("Relative Position Error");
    matplotlibcpp::ylabel("translational error (m)");
    matplotlibcpp::xlabel("sub-segment lengths (m)");
    matplotlibcpp::show(true);

    // Wait till the user kills this node
    //ros::spin();

#endif


    // Done!
    return EXIT_SUCCESS;

}


