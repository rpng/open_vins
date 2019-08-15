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
#include "Feature.h"


using namespace ov_core;




void Feature::clean_old_measurements(std::vector<double> valid_times) {


    // Loop through each of the cameras we have
    for(auto const &pair : timestamps) {

        // Assert that we have all the parts of a measurement
        assert(timestamps[pair.first].size() == uvs[pair.first].size());
        assert(timestamps[pair.first].size() == uvs_norm[pair.first].size());

        // Our iterators
        auto it1 = timestamps[pair.first].begin();
        auto it2 = uvs[pair.first].begin();
        auto it3 = uvs_norm[pair.first].begin();

        // Loop through measurement times, remove ones that are not in our timestamps
        while (it1 != timestamps[pair.first].end()) {
            if (std::find(valid_times.begin(),valid_times.end(),*it1) == valid_times.end()) {
                it1 = timestamps[pair.first].erase(it1);
                it2 = uvs[pair.first].erase(it2);
                it3 = uvs_norm[pair.first].erase(it3);
            } else {
                ++it1;
                ++it2;
                ++it3;
            }
        }
    }

}




