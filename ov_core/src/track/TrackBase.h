#ifndef OV_CORE_TRACK_BASE_H
#define OV_CORE_TRACK_BASE_H


#include <iostream>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Grider_FAST.h"
#include "Grider_DOG.h"
#include "FeatureDatabase.h"

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
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
     * This base class also handles most of the heavy lifting with the visualalization, but the sub-classes can override
     * this and do their own logic if they want (i.e. the auroctag tracker has its own logic for visualization).
     */
    class TrackBase {

    public:

        /**
         * @brief Public default constructor
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         */
        TrackBase(std::unordered_map<size_t, Eigen::Matrix<double,8,1>> camera_calib,
                  std::unordered_map<size_t, bool> camera_fisheye) :
                database(new FeatureDatabase()), num_features(200), currid(0) {
            // Set calibration params
            set_calibration(camera_calib, camera_fisheye);
        }

        /**
         * @brief Public constructor with configuration variables
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
         * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
         */
        TrackBase(std::unordered_map<size_t, Eigen::Matrix<double,8,1>> camera_calib,
                  std::unordered_map<size_t, bool> camera_fisheye,
                  int numfeats, int numaruco) :
                database(new FeatureDatabase()), num_features(numfeats) {
            // Our current feature ID should be larger then the number of aruco tags we have
            currid = (size_t) numaruco + 1;
            // Set calibration params
            set_calibration(camera_calib, camera_fisheye);
        }


        /**
         * @brief Given a the camera intrinsic values, this will set what we should normalize points with.
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye Map of camera_id => bool if we should do radtan or fisheye distortion model
         */
        void set_calibration(std::unordered_map<size_t, Eigen::Matrix<double,8,1>> camera_calib,
                             std::unordered_map<size_t, bool> camera_fisheye) {

            // Clear old maps
            camera_k_OPENCV.clear();
            camera_d_OPENCV.clear();

            // Overwrite our fisheye calibration
            this->camera_fisheye = camera_fisheye;

            // Convert values to the OpenCV format
            for (auto const &cam : camera_calib) {

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
        virtual void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left,
                                 size_t cam_id_right) = 0;

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
         */
        virtual void display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2);

        /**
         * @brief Get the feature database with all the track information
         * @return FeatureDatabase pointer that one can query for features
         */
        FeatureDatabase *get_feature_database() {
            return database;
        }

    protected:

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
            cv::Matx33d camK = this->camera_k_OPENCV[cam_id];
            cv::Vec4d camD = this->camera_d_OPENCV[cam_id];
            // Call on the fisheye if we should!
            if (this->camera_fisheye[cam_id]) {
                return undistort_point_fisheye(pt_in, camK, camD);
            }
            return undistort_point_brown(pt_in, camK, camD);
        }

        /**
         * @brief Undistort function RADTAN/BROWN.
         *
         * Given a uv point, this will undistort it based on the camera matrices.
         * To equate this to Kalibr's models, this is what you would use for `pinhole-radtan`.
         */
        cv::Point2f undistort_point_brown(cv::Point2f pt_in, cv::Matx33d &camK, cv::Vec4d &camD) {
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
        cv::Point2f undistort_point_fisheye(cv::Point2f pt_in, cv::Matx33d &camK, cv::Vec4d &camD) {
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

        // Database with all our current features
        FeatureDatabase *database;

        // If we are a fisheye model or not
        std::unordered_map<size_t, bool> camera_fisheye;

        // Camera intrinsics in OpenCV format
        std::unordered_map<size_t, cv::Matx33d> camera_k_OPENCV;
        std::unordered_map<size_t, cv::Vec4d> camera_d_OPENCV;

        // number of features we should try to track frame to frame
        int num_features;

        // Last set of images
        std::unordered_map<size_t, cv::Mat> img_last;

        // Last set of tracked points
        std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

        // Set of IDs of each current feature in the database
        size_t currid = 0;
        std::unordered_map<size_t, std::vector<size_t>> ids_last;


    };

}

#endif /* OV_CORE_TRACK_BASE_H */