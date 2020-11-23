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
#ifndef OV_CORE_TRACK_BASE_H
#define OV_CORE_TRACK_BASE_H


#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>

#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Grider_FAST.h"
#include "Grider_DOG.h"
#include "feat/FeatureDatabase.h"
#include "utils/colors.h"


namespace ov_core {


    /**
     * @brief Visual feature tracking base class
     *
     * This is the base class for all our visual trackers.
     * The goal here is to provide a common interface so all underlying trackers can simply hide away all the complexities.
     * We have something called the "feature database" which has all the tracking information inside of it.
     * The user can ask this database for features which can then be used in an MSCKF or batch-based setting.
     * The feature tracks store both the raw (distorted) and undistorted/normalized values.
     * Right now we just support two camera models, see: undistort_point_brown() and undistort_point_fisheye().
     *
     * @m_class{m-note m-warning}
     *
     * @par A Note on Multi-Threading Support
     * There is some support for asynchronous multi-threaded feature tracking of independent cameras.
     * The key assumption during implementation is that the user will not try to track on the same camera in parallel, and instead call on different cameras.
     * For example, if I have two cameras, I can either sequentially call the feed function, or I spin each of these into separate threads and wait for their return.
     * The @ref currid is atomic to allow for multiple threads to access it without issue and ensure that all features have unique id values.
     * We also have mutex for access for the calibration and previous images and tracks (used during visualization).
     * It should be noted that if a thread calls visualization, it might hang or the feed thread might, due to acquiring the mutex for that specific camera id / feed.
     *
     * This base class also handles most of the heavy lifting with the visualization, but the sub-classes can override
     * this and do their own logic if they want (i.e. the TrackAruco has its own logic for visualization).
     * This visualization needs access to the prior images and their tracks, thus must synchronise in the case of multi-threading.
     * This shouldn't impact performance, but high frequency visualization calls can negatively effect the performance.
     */
    class TrackBase {

    public:

        /**
         * @brief Public default constructor
         */
        TrackBase() : database(new FeatureDatabase()), num_features(200), use_multi_threading(false), currid(0) { }

        /**
         * @brief Public constructor with configuration variables
         * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
         * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
         * @param multithread if we should try to process with multiple threads or single threaded
         */
        TrackBase(int numfeats, int numaruco, bool multithread) :
            database(new FeatureDatabase()), num_features(numfeats), use_multi_threading(multithread) {
            // Our current feature ID should be larger then the number of aruco tags we have
            currid = (size_t) numaruco + 1;
        }

        virtual ~TrackBase() { }


        /**
         * @brief Given a the camera intrinsic values, this will set what we should normalize points with.
         * This will also update the feature database with corrected normalized values.
         * Normally this would only be needed if we are optimizing our camera parameters, and thus should re-normalize.
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye Map of camera_id => bool if we should do radtan or fisheye distortion model
         * @param correct_active If we should re-undistort active features in our database
         */
        void set_calibration(std::map<size_t,Eigen::VectorXd> camera_calib,
                             std::map<size_t, bool> camera_fisheye, bool correct_active=false) {

            // Assert vectors are equal
            assert(camera_calib.size()==camera_fisheye.size());

            // Initialize our mutex and camera intrinsic values if we are just starting up
            // The number of cameras should not change over time, thus we just need to check if our initial data is empty
            if(mtx_feeds.empty() || camera_k_OPENCV.empty() || camera_d_OPENCV.empty() || this->camera_fisheye.empty()) {
                // Create our mutex array based on the number of cameras we have
                // See https://stackoverflow.com/a/24170141/7718197
                std::vector<std::mutex> list(camera_calib.size());
                mtx_feeds.swap(list);
                // Overwrite our fisheye calibration
                this->camera_fisheye = camera_fisheye;
                // Convert values to the OpenCV format
                for (auto const &cam : camera_calib) {
                    // Assert we are of size eight
                    assert(cam.second.rows()==8);
                    // Camera matrix
                    cv::Matx33d tempK;
                    tempK(0, 0) = cam.second(0);
                    tempK(0, 1) = 0;
                    tempK(0, 2) = cam.second(2);
                    tempK(1, 0) = 0;
                    tempK(1, 1) = cam.second(1);
                    tempK(1, 2) = cam.second(3);
                    tempK(2, 0) = 0;
                    tempK(2, 1) = 0;
                    tempK(2, 2) = 1;
                    camera_k_OPENCV.insert({cam.first, tempK});
                    // Distortion parameters
                    cv::Vec4d tempD;
                    tempD(0) = cam.second(4);
                    tempD(1) = cam.second(5);
                    tempD(2) = cam.second(6);
                    tempD(3) = cam.second(7);
                    camera_d_OPENCV.insert({cam.first, tempD});
                }
                return;
            }

            // assert that the number of cameras can not change
            assert(camera_k_OPENCV.size()==camera_calib.size());
            assert(camera_k_OPENCV.size()==camera_fisheye.size());
            assert(camera_k_OPENCV.size()==camera_d_OPENCV.size());

            // Convert values to the OpenCV format
            for (auto const &cam : camera_calib) {
                // Lock this image feed
                std::unique_lock<std::mutex> lck(mtx_feeds.at(cam.first));
                // Fisheye value
                this->camera_fisheye.at(cam.first) = camera_fisheye.at(cam.first);
                // Assert we are of size eight
                assert(cam.second.rows()==8);
                // Camera matrix
                cv::Matx33d tempK;
                tempK(0, 0) = cam.second(0);
                tempK(0, 1) = 0;
                tempK(0, 2) = cam.second(2);
                tempK(1, 0) = 0;
                tempK(1, 1) = cam.second(1);
                tempK(1, 2) = cam.second(3);
                tempK(2, 0) = 0;
                tempK(2, 1) = 0;
                tempK(2, 2) = 1;
                camera_k_OPENCV.at(cam.first) = tempK;
                // Distortion parameters
                cv::Vec4d tempD;
                tempD(0) = cam.second(4);
                tempD(1) = cam.second(5);
                tempD(2) = cam.second(6);
                tempD(3) = cam.second(7);
                camera_d_OPENCV.at(cam.first) = tempD;
            }

            // If we are calibrating camera intrinsics our normalize coordinates will be stale
            // This is because we appended them to the database with the current best guess *at that timestep*
            // Thus here since we have a change in calibration, re-normalize all the features we have
            if(correct_active) {

                // Get all features in this database
                std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup = database->get_internal_data();

                // Loop through and correct each one
                for(const auto& pair_feat : features_idlookup) {
                    // Get our feature
                    std::shared_ptr<Feature> feat = pair_feat.second;
                    // Loop through each camera for this feature
                    for (auto const& meas_pair : feat->timestamps) {
                        size_t camid = meas_pair.first;
                        std::unique_lock<std::mutex> lck(mtx_feeds.at(camid));
                        for(size_t m=0; m<feat->uvs.at(camid).size(); m++) {
                            cv::Point2f pt(feat->uvs.at(camid).at(m)(0), feat->uvs.at(camid).at(m)(1));
                            cv::Point2f pt_n = undistort_point(pt,camid);
                            feat->uvs_norm.at(camid).at(m)(0) = pt_n.x;
                            feat->uvs_norm.at(camid).at(m)(1) = pt_n.y;
                        }
                    }
                }

            }


        }

        /**
         * @brief Process a new monocular image
         * @param timestamp timestamp the new image occurred at
         * @param img new cv:Mat grayscale image
         * @param cam_id the camera id that this new image corresponds too
         */
        virtual void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) = 0;

        /**
         * @brief Process new stereo pair of images
         * @param timestamp timestamp this pair occured at (stereo is synchronised)
         * @param img_left first grayscaled image
         * @param img_right second grayscaled image
         * @param cam_id_left first image camera id
         * @param cam_id_right second image camera id
         */
        virtual void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) = 0;

        /**
         * @brief Shows features extracted in the last image
         * @param img_out image to which we will overlayed features on
         * @param r1,g1,b1 first color to draw in
         * @param r2,g2,b2 second color to draw in
         */
        virtual void display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2);

        /**
         * @brief Shows a "trail" for each feature (i.e. its history)
         * @param img_out image to which we will overlayed features on
         * @param r1,g1,b1 first color to draw in
         * @param r2,g2,b2 second color to draw in
         * @param highlighted unique ids which we wish to highlight
         */
        virtual void display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::vector<size_t> highlighted={});

        /**
         * @brief Get the feature database with all the track information
         * @return FeatureDatabase pointer that one can query for features
         */
        std::shared_ptr<FeatureDatabase> get_feature_database() {
            return database;
        }

        /**
         * @brief Changes the ID of an actively tracked feature to another one
         * @param id_old Old id we want to change
         * @param id_new Id we want to change the old id to
         */
        void change_feat_id(size_t id_old, size_t id_new) {

            // If found in db then replace
            if(database->get_internal_data().find(id_old)!=database->get_internal_data().end()) {
                std::shared_ptr<Feature> feat = database->get_internal_data().at(id_old);
                database->get_internal_data().erase(id_old);
                feat->featid = id_new;
                database->get_internal_data().insert({id_new, feat});
            }

            // Update current track IDs
            for(auto &cam_ids_pair : ids_last) {
                for(size_t i=0; i<cam_ids_pair.second.size(); i++) {
                    if(cam_ids_pair.second.at(i)==id_old) {
                        ids_last.at(cam_ids_pair.first).at(i) = id_new;
                    }
                }
            }

        }

        /**
         * @brief Main function that will undistort/normalize a point.
         * @param pt_in uv 2x1 point that we will undistort
         * @param cam_id id of which camera this point is in
         * @return undistorted 2x1 point
         *
         * Given a uv point, this will undistort it based on the camera matrices.
         * This will call on the model needed, depending on what type of camera it is!
         * So if we have fisheye for camera_1 is true, we will undistort with the fisheye model.
         * In Kalibr's terms, the non-fisheye is `pinhole-radtan` while the fisheye is the `pinhole-equi` model.
         */
        cv::Point2f undistort_point(cv::Point2f pt_in, size_t cam_id) {
            // Determine what camera parameters we should use
            cv::Matx33d camK = this->camera_k_OPENCV.at(cam_id);
            cv::Vec4d camD = this->camera_d_OPENCV.at(cam_id);
            // Call on the fisheye if we should!
            if (this->camera_fisheye.at(cam_id)) {
                return undistort_point_fisheye(pt_in, camK, camD);
            }
            return undistort_point_brown(pt_in, camK, camD);
        }

    protected:

        /**
         * @brief Undistort function RADTAN/BROWN.
         *
         * Given a uv point, this will undistort it based on the camera matrices.
         * To equate this to Kalibr's models, this is what you would use for `pinhole-radtan`.
         */
        static cv::Point2f undistort_point_brown(cv::Point2f pt_in, cv::Matx33d &camK, cv::Vec4d &camD) {
            // Convert to opencv format
            cv::Mat mat(1, 2, CV_32F);
            mat.at<float>(0, 0) = pt_in.x;
            mat.at<float>(0, 1) = pt_in.y;
            mat = mat.reshape(2); // Nx1, 2-channel
            // Undistort it!
            cv::undistortPoints(mat, mat, camK, camD);
            // Construct our return vector
            cv::Point2f pt_out;
            mat = mat.reshape(1); // Nx2, 1-channel
            pt_out.x = mat.at<float>(0, 0);
            pt_out.y = mat.at<float>(0, 1);
            return pt_out;
        }

        /**
         * @brief Undistort function FISHEYE/EQUIDISTANT.
         *
         * Given a uv point, this will undistort it based on the camera matrices.
         * To equate this to Kalibr's models, this is what you would use for `pinhole-equi`.
         */
        static cv::Point2f undistort_point_fisheye(cv::Point2f pt_in, cv::Matx33d &camK, cv::Vec4d &camD) {
            // Convert point to opencv format
            cv::Mat mat(1, 2, CV_32F);
            mat.at<float>(0, 0) = pt_in.x;
            mat.at<float>(0, 1) = pt_in.y;
            mat = mat.reshape(2); // Nx1, 2-channel
            // Undistort it!
            cv::fisheye::undistortPoints(mat, mat, camK, camD);
            // Construct our return vector
            cv::Point2f pt_out;
            mat = mat.reshape(1); // Nx2, 1-channel
            pt_out.x = mat.at<float>(0, 0);
            pt_out.y = mat.at<float>(0, 1);
            return pt_out;
        }

        /// Database with all our current features
        std::shared_ptr<FeatureDatabase> database;

        /// If we are a fisheye model or not
        std::map<size_t, bool> camera_fisheye;

        /// Camera intrinsics in OpenCV format
        std::map<size_t, cv::Matx33d> camera_k_OPENCV;

        /// Camera distortion in OpenCV format
        std::map<size_t, cv::Vec4d> camera_d_OPENCV;

        /// Number of features we should try to track frame to frame
        int num_features;

        /// Boolean for if we should try to multi-thread our frontends
        bool use_multi_threading = false;

        /// Mutexs for our last set of image storage (img_last, pts_last, and ids_last)
        std::vector<std::mutex> mtx_feeds;

        /// Last set of images (use map so all trackers render in the same order)
        std::map<size_t, cv::Mat> img_last;

        /// Last set of tracked points
        std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

        /// Set of IDs of each current feature in the database
        std::unordered_map<size_t, std::vector<size_t>> ids_last;

        /// Master ID for this tracker (atomic to allow for multi-threading)
        std::atomic<size_t> currid;


    };

}

#endif /* OV_CORE_TRACK_BASE_H */
