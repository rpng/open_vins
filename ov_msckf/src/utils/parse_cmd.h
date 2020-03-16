/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
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
#ifndef OV_MSCKF_PARSE_CMDLINE_H
#define OV_MSCKF_PARSE_CMDLINE_H


#include "core/VioManagerOptions.h"
#include "utils/CLI11.hpp"


namespace ov_msckf {



    /**
     * @brief This function will parse the command line arugments using [CLI11](https://github.com/CLIUtils/CLI11).
     * This is only used if you are not building with ROS, and thus isn't the primary supported way to pass arguments.
     * We recommend building with ROS as compared using this parser.
     * @param argc Number of parameters
     * @param argv Pointer to string passed as options
     * @return A fully loaded VioManagerOptions object
     */
    VioManagerOptions parse_command_line_arguments(int argc, char** argv) {

        // Our vio manager options with defaults
        VioManagerOptions params;

        // Create our command line parser
        CLI::App app1{"parser_cmd_01"};
        app1.allow_extras();

        app1.add_option("--max_cameras", params.max_cameras, "");
        app1.add_option("--use_stereo", params.use_stereo, "");
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        app1.add_option("--gravity", gravity, "");


        app1.add_option("--gyroscope_noise_density", params.gyroscope_noise_density, "");
        app1.add_option("--accelerometer_noise_density", params.accelerometer_noise_density, "");
        app1.add_option("--gyroscope_random_walk", params.gyroscope_random_walk, "");
        app1.add_option("--accelerometer_random_walk", params.accelerometer_random_walk, "");
        app1.add_option("--up_msckf_sigma_px", params.up_msckf_sigma_px, "");

        app1.add_option("--calib_camimu_dt", params.calib_camimu_dt, "");


        // Finally actually parse the command line and load it
        try {
            app1.parse(argc, argv);
        } catch (const CLI::ParseError &e) {
            std::exit(app1.exit(e));
        }

        // Parse gravity
        assert(gravity.size()==3);
        params.gravity << gravity.at(0), gravity.at(1), gravity.at(2);

        //======================================================================
        //======================================================================
        //======================================================================


        // Create our command line parser for the cameras
        // NOTE: we need to first parse how many cameras we have before we can parse this
        CLI::App app2{"parser_cmd_02"};
        app2.allow_extras();

        std::vector<int> p_fish;
        std::vector<std::vector<double>> p_intrinsic;
        std::vector<std::vector<double>> p_extrinsic;
        std::vector<std::vector<int>> p_wh;

        // Set the defaults
        for(int i=0; i<params.max_cameras; i++) {
            p_fish.push_back(false);
            p_intrinsic.push_back({458.654,457.296,367.215,248.375,-0.28340811,0.07395907,0.00019359,1.76187114e-05});
            p_extrinsic.push_back({0,0,0,1,0,0,0});
            p_wh.push_back({752,480});
            app2.add_option("--cam"+std::to_string(i)+"_fisheye", p_fish.at(i));
            app2.add_option("--cam"+std::to_string(i)+"_intrinsic", p_intrinsic.at(i), "");
            app2.add_option("--cam"+std::to_string(i)+"_extrinsic", p_extrinsic.at(i), "");
            app2.add_option("--cam"+std::to_string(i)+"_wh", p_wh.at(i), "");
        }

        // Finally actually parse the command line and load it
        try {
            app2.parse(argc, argv);
        } catch (const CLI::ParseError &e) {
            std::exit(app2.exit(e));
        }

        // Finally load it into our params
        for(int i=0; i<params.max_cameras; i++) {

            // Convert to Eigen
            assert(p_intrinsic.at(i).size()==8);
            Eigen::Matrix<double,8,1> intrinsics;
            intrinsics << p_intrinsic.at(i).at(0),p_intrinsic.at(i).at(1),p_intrinsic.at(i).at(2),p_intrinsic.at(i).at(3),
                    p_intrinsic.at(i).at(4),p_intrinsic.at(i).at(5),p_intrinsic.at(i).at(6),p_intrinsic.at(i).at(7);
            assert(p_extrinsic.at(i).size()==7);
            Eigen::Matrix<double,7,1> extrinsics;
            extrinsics << p_extrinsic.at(i).at(0),p_extrinsic.at(i).at(1),p_extrinsic.at(i).at(2),p_extrinsic.at(i).at(3),
                    p_extrinsic.at(i).at(4),p_extrinsic.at(i).at(5),p_extrinsic.at(i).at(6);
            assert(p_wh.at(i).size()==2);

            // Insert
            params.camera_fisheye.insert({i, p_fish.at(i)});
            params.camera_intrinsics.insert({i, intrinsics});
            params.camera_extrinsics.insert({i, extrinsics});
            params.camera_wh.insert({i, {p_wh.at(i).at(0),p_wh.at(i).at(1)}});
            cout << "inserting cam " << i << endl;

        }

        // Success, lets returned the parsed options
        return params;

    }



}


#endif //OV_MSCKF_PARSE_CMDLINE_H