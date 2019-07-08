#ifndef OV_EVAL_STATISTICS_H
#define OV_EVAL_STATISTICS_H


#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>


namespace ov_eval {


    /**
     * @brief Stat object for a given set scalar time series values.
     */
    struct Statistics {

    public:

        /// Root mean squared for the given values
        double rmse = 0.0;

        /// Mean of the given values
        double mean = 0.0;

        /// Median of the given values
        double median = 0.0;

        /// Standard deviation of given values
        double std = 0.0;

        /// Max of the given values
        double max = 0.0;

        /// Min of the given values
        double min = 0.0;

        /// Timestamp when these values occured at
        std::vector<double> timestamps;

        /// Values (e.g. error or nees at a given time)
        std::vector<double> values;

        /// Bound of these values (e.g. our expected covariance bound)
        std::vector<double> values_bound;


        /// Will calculate all values from our vectors of information
        void calculate() {

            // Sort the data for easy finding of values
            std::vector<double> values_sorted = values;
            std::sort(values_sorted.begin(), values_sorted.end());

            // Return if we don't have any data
            if(values_sorted.size() < 2)
                return;

            // Now that its been sorted, can easily grab min and max
            min = values_sorted.at(0);
            max = values_sorted.at(values_sorted.size()-1);

            // Compute median
            // ODD:  grab middle from the sorted vector
            // EVEN: average the middle two numbers
            if (values_sorted.size() % 2 == 1) {
                median = values_sorted.at(values_sorted.size() / 2);
            } else {
                median = 0.5 * (values_sorted.at(values_sorted.size() / 2) + values_sorted.at(values_sorted.size() / 2 + 1));
            }

            // Compute mean and rmse
            mean = 0;
            for (size_t i = 0; i < values_sorted.size(); i++) {
                assert(!std::isnan(values_sorted.at(i)));
                mean += values_sorted.at(i);
                rmse += values_sorted.at(i) * values_sorted.at(i);
            }
            mean /= values_sorted.size();
            rmse = std::sqrt(rmse / values_sorted.size());

            // Using mean, compute standard deviation
            std = 0;
            for (size_t i = 0; i < values_sorted.size(); i++) {
                std += std::pow(values_sorted.at(i) - mean, 2);
            }
            std = std::sqrt(std / (values_sorted.size() - 1));

        }

        /// Will clear any old values
        void clear() {
            timestamps.clear();
            values.clear();
            values_bound.clear();
        }

    };



}

#endif //OV_EVAL_STATISTICS_H
