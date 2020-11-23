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
#ifndef OV_CORE_GRIDER_FAST_H
#define OV_CORE_GRIDER_FAST_H


#include <vector>
#include <iostream>
#include <functional>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace ov_core {

    /**
     * This is a utility class required to build with older version of opencv
     * On newer versions this doesn't seem to be needed, but here we just use it to ensure we can work for more opencv version.
     * https://answers.opencv.org/question/65800/how-to-use-lambda-as-a-parameter-to-parallel_for_/?answer=130691#post-id-130691
     */
    class LambdaBody : public cv::ParallelLoopBody {    
    public:
        explicit LambdaBody(const std::function<void(const cv::Range &)> &body) {
            _body = body;
        }
        void operator() (const cv::Range & range) const override {
            _body(range);
        }    
    private:
        std::function<void(const cv::Range &)> _body;
    };

    /**
     * @brief Extracts FAST features in a grid pattern.
     *
     * As compared to just extracting fast features over the entire image,
     * we want to have as uniform of extractions as possible over the image plane.
     * Thus we split the image into a bunch of small grids, and extract points in each.
     * We then pick enough top points in each grid so that we have the total number of desired points.
     */
    class Grider_FAST {

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
         * @brief This function will perform grid extraction using FAST.
         * @param img Image we will do FAST extraction on
         * @param pts vector of extracted points we will return
         * @param multithread if we should try to process with multiple threads or single threaded
         * @param num_features max number of features we want to extract
         * @param grid_x size of grid in the x-direction / u-direction
         * @param grid_y size of grid in the y-direction / v-direction
         * @param threshold FAST threshold paramter (10 is a good value normally)
         * @param nonmaxSuppression if FAST should perform non-max suppression (true normally)
         *
         * Given a specified grid size, this will try to extract fast features from each grid.
         * It will then return the best from each grid in the return vector.
         */
        static void perform_griding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, bool multithread, int num_features,
                                    int grid_x, int grid_y, int threshold, bool nonmaxSuppression) {

            // Calculate the size our extraction boxes should be
            int size_x = img.cols / grid_x;
            int size_y = img.rows / grid_y;

            // Make sure our sizes are not zero
            assert(size_x > 0);
            assert(size_y > 0);

            // We want to have equally distributed features
            auto num_features_grid = (int) (num_features / (grid_x * grid_y)) + 1;

            // Either we will parellize with opencv parallel for loop
            // Or we will run through a standard double for loop
            if (multithread) {
                // Parallelize our 2d grid extraction!!
                int ct_cols = std::floor(img.cols/size_x);
                int ct_rows = std::floor(img.rows/size_y);
                std::vector<std::vector<cv::KeyPoint>> collection(ct_cols*ct_rows);
                parallel_for_(cv::Range(0, ct_cols*ct_rows), LambdaBody([&](const cv::Range& range) {
                    for (int r = range.start; r < range.end; r++) {
                        // Calculate what cell xy value we are in
                        int x = r%ct_cols*size_x;
                        int y = r/ct_cols*size_y;

                        // Skip if we are out of bounds
                        if (x + size_x > img.cols || y + size_y > img.rows)
                            continue;

                        // Calculate where we should be extracting from
                        cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                        // Extract FAST features for this part of the image
                        std::vector<cv::KeyPoint> pts_new;
                        cv::FAST(img(img_roi), pts_new, threshold, nonmaxSuppression);

                        // Now lets get the top number from this
                        std::sort(pts_new.begin(), pts_new.end(), Grider_FAST::compare_response);

                        // Append the "best" ones to our vector
                        // Note that we need to "correct" the point u,v since we extracted it in a ROI
                        // So we should append the location of that ROI in the image
                        for(size_t i=0; i<(size_t)num_features_grid && i<pts_new.size(); i++) {
                            cv::KeyPoint pt_cor = pts_new.at(i);
                            pt_cor.pt.x += (float)x;
                            pt_cor.pt.y += (float)y;
                            collection.at(r).push_back(pt_cor);
                        }
                    }
                }));

                // Combine all the collections into our single vector
                for(size_t r=0; r<collection.size(); r++) {
                    pts.insert(pts.end(),collection.at(r).begin(),collection.at(r).end());
                }

            } else {
                // Lets loop through each grid and extract features
                for (int x = 0; x < img.cols; x += size_x) {
                    for (int y = 0; y < img.rows; y += size_y) {
                        // Skip if we are out of bounds
                        if (x + size_x > img.cols || y + size_y > img.rows)
                            continue;

                        // Calculate where we should be extracting from
                        cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                        // Extract FAST features for this part of the image
                        std::vector<cv::KeyPoint> pts_new;
                        cv::FAST(img(img_roi), pts_new, threshold, nonmaxSuppression);

                        // Now lets get the top number from this
                        std::sort(pts_new.begin(), pts_new.end(), Grider_FAST::compare_response);

                        // Append the "best" ones to our vector
                        // Note that we need to "correct" the point u,v since we extracted it in a ROI
                        // So we should append the location of that ROI in the image
                        for(size_t i=0; i<(size_t)num_features_grid && i<pts_new.size(); i++) {
                            cv::KeyPoint pt_cor = pts_new.at(i);
                            pt_cor.pt.x += (float)x;
                            pt_cor.pt.y += (float)y;
                            pts.push_back(pt_cor);
                        }
                    }
                }

            }

            // Return if no points
            if(pts.empty())
                return;

            // Sub-pixel refinement parameters
            cv::Size win_size = cv::Size( 5, 5 );
            cv::Size zero_zone = cv::Size( -1, -1 );
            cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.001);

            // Get vector of points
            std::vector<cv::Point2f> pts_refined;
            for(size_t i=0; i<pts.size(); i++) {
                pts_refined.push_back(pts.at(i).pt);
            }

            // Finally get sub-pixel for all extracted features
            cv::cornerSubPix(img, pts_refined, win_size, zero_zone, term_crit);

            // Save the refined points!
            for(size_t i=0; i<pts.size(); i++) {
                pts.at(i).pt = pts_refined.at(i);
            }


        }

    };

}


#endif /* OV_CORE_GRIDER_FAST_H */
