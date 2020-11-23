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
#ifndef OV_CORE_GRIDER_DOG_H
#define OV_CORE_GRIDER_DOG_H


#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ov_core {

    /**
     * @brief Does Difference of Gaussian (DoG) in a grid pattern.
     *
     * This does "Difference of Gaussian" detection in a grid pattern to try to get good features.
     * We then pick the top features in each grid, and return the top features collected over the entire image.
     * This class hasn't been tested that much, as we normally use the Grider_FAST class instead.
     *
     * @m_class{m-block m-danger}
     *
     * @par Improve and Test This!
     *      If you are interested in using this grider, then please give it a try and test if for us.
     *      Please open a pull request with any impromements, or an issue if you have had any success.
     *      We still recommend the Grider_FAST class since that is what we normally use.
     *
     */
    class Grider_DOG {

    public:


        /**
         * @brief Compare keypoints based on their response value.
         * @param first First keypoint
         * @param second Second keypoint
         *
         * We want to have the keypoints with the highest values!
         * See: https://stackoverflow.com/a/10910921
         */
        static bool compare_response(cv::KeyPoint first, cv::KeyPoint second) {
            return first.response > second.response;
        }


        /**
         * @brief For a given small image region this will do the Difference of Gaussian (DOG) detection
         *
         * Will return the vector of keypoints with the averaged response for that given UV.
         * See: https://github.com/jrdi/opencv-examples/blob/master/dog/main.cpp
         */
        static void detect(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, int ksize,
                           float sigma_small, float sigma_big, float threshold) {

            // Calculate the two kernels we will use
            cv::Mat gauBig = cv::getGaussianKernel(ksize, sigma_big, CV_32F);
            cv::Mat gauSmall = cv::getGaussianKernel(ksize, sigma_small, CV_32F);
            cv::Mat DoG = gauSmall - gauBig;

            // Apply the kernel to our image
            cv::Mat img_filtered;
            cv::sepFilter2D(img, img_filtered, CV_32F, DoG.t(), DoG);
            img_filtered = cv::abs(img_filtered);

            // Assert we are a grey scaled image
            assert(img_filtered.channels() == 1);

            // Loop through and record all points above our threshold
            for (int j = 0; j < img_filtered.rows; j++) {
                for (int i = 0; i < img_filtered.cols; i++) {
                    // Calculate the response at this u,v coordinate
                    float response = img_filtered.at<float>(j, i);
                    // Check to see if it is able our specified threshold
                    if (response >= threshold) {
                        cv::KeyPoint pt;
                        pt.pt.x = i;
                        pt.pt.y = j;
                        pt.response = response;
                        pts.push_back(pt);
                    }
                }
            }

        }


        /**
         * @brief This function will perform grid extraction using Difference of Gaussian (DOG)
         * @param img Image we will do FAST extraction on
         * @param pts vector of extracted points we will return
         * @param num_features max number of features we want to extract
         * @param grid_x size of grid in the x-direction / u-direction
         * @param grid_y size of grid in the y-direction / v-direction
         * @param ksize kernel size
         * @param sigma_small small gaussian sigma
         * @param sigma_big big gaussian sigma
         * @param threshold response threshold
         *
         * Given a specified grid size, this will try to extract fast features from each grid.
         * It will then return the best from each grid in the return vector.
         */
        static void perform_griding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, int num_features,
                                    int grid_x, int grid_y, int ksize, float sigma_small, float sigma_big,
                                    float threshold) {

            // Calculate the size our extraction boxes should be
            int size_x = img.cols / grid_x;
            int size_y = img.rows / grid_y;

            // Make sure our sizes are not zero
            assert(size_x > 0);
            assert(size_y > 0);

            // We want to have equally distributed features
            auto num_features_grid = (int) (num_features / (grid_x * grid_y)) + 1;

            // Lets loop through each grid and extract features
            for (int x = 0; x < img.cols; x += size_x) {
                for (int y = 0; y < img.rows; y += size_y) {

                    // Skip if we are out of bounds
                    if (x + size_x > img.cols || y + size_y > img.rows)
                        continue;

                    // Calculate where we should be extracting from
                    cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                    // Extract DOG features for this part of the image
                    std::vector<cv::KeyPoint> pts_new;
                    Grider_DOG::detect(img(img_roi), pts_new, ksize, sigma_small, sigma_big, threshold);

                    // Now lets get the top number from this
                    std::sort(pts_new.begin(), pts_new.end(), Grider_DOG::compare_response);

                    // Debug print out the response values
                    //for (auto pt : pts_new) {
                    //    std::cout << pt.response << std::endl;
                    //}
                    //std::cout << "======================" << std::endl;

                    // Append the "best" ones to our vector
                    // Note that we need to "correct" the point u,v since we extracted it in a ROI
                    // So we should append the location of that ROI in the image
                    for (size_t i = 0; i < (size_t) num_features_grid && i < pts_new.size(); i++) {
                        cv::KeyPoint pt_cor = pts_new.at(i);
                        pt_cor.pt.x += x;
                        pt_cor.pt.y += y;
                        pts.push_back(pt_cor);
                    }


                }
            }


        }


    };

}

#endif /* OV_CORE_GRIDER_DOG_H */