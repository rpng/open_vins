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




