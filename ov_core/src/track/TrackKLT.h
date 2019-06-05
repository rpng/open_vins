#ifndef OV_CORE_TRACK_KLT_H
#define OV_CORE_TRACK_KLT_H


#include "TrackBase.h"


/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {


    /**
     * @brief KLT tracking of features.
     *
     * This is the implementation of a KLT visual frontend for tracking sparse features.
     * We can track either monocular cameras across time (temporally) along with
     * stereo cameras which we also track across time (temporally) but track from left to right
     * to find the stereo correspondence information also.
     * This uses the [calcOpticalFlowPyrLK](https://github.com/opencv/opencv/blob/master/modules/video/src/lkpyramid.cpp) OpenCV function to do the KLT tracking.
     */
    class TrackKLT : public TrackBase {

    public:

        /**
         * @brief Public default constructor
         * @param camera_k map of camera_id => 3x3 camera intrinic matrix
         * @param camera_d  map of camera_id => 4x1 camera distortion parameters
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         */
        TrackKLT(std::unordered_map<size_t, Eigen::Matrix3d> camera_k,
                 std::unordered_map<size_t, Eigen::Matrix<double, 4, 1>> camera_d,
                 std::unordered_map<size_t, bool> camera_fisheye) :
                TrackBase(camera_k, camera_d, camera_fisheye), threshold(10), grid_x(8), grid_y(5), min_px_dist(30) {}

        /**
         * @brief Public constructor with configuration variables
         * @param camera_k map of camera_id => 3x3 camera intrinic matrix
         * @param camera_d  map of camera_id => 4x1 camera distortion parameters
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
         * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
         * @param fast_threshold FAST detection threshold
         * @param gridx size of grid in the x-direction / u-direction
         * @param gridy size of grid in the y-direction / v-direction
         * @param minpxdist features need to be at least this number pixels away from each other
         */
        explicit TrackKLT(std::unordered_map<size_t, Eigen::Matrix3d> camera_k,
                          std::unordered_map<size_t, Eigen::Matrix<double, 4, 1>> camera_d,
                          std::unordered_map<size_t, bool> camera_fisheye,
                          int numfeats, int numaruco, int fast_threshold, int gridx, int gridy, int minpxdist) :
                TrackBase(camera_k, camera_d, camera_fisheye, numfeats, numaruco), threshold(fast_threshold),
                grid_x(gridx), grid_y(gridy), min_px_dist(minpxdist) {}


        /**
         * @brief Process a new monocular image
         * @param timestamp timestamp the new image occurred at
         * @param img new cv:Mat grayscale image
         * @param cam_id the camera id that this new image corresponds too
         */
        void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) override;

        /**
         * @brief Process new stereo pair of images
         * @param timestamp timestamp this pair occured at (stereo is synchronised)
         * @param img_left first grayscaled image
         * @param img_right second grayscaled image
         * @param cam_id_left first image camera id
         * @param cam_id_right second image camera id
         */
        void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) override;


    protected:

        /**
         * @brief Detects new features in the current image
         * @param img0 image we will detect features on
         * @param pts0 vector of currently extracted keypoints in this image
         * @param ids0 vector of feature ids for each currently extracted keypoint
         *
         * Given an image and its currently extracted features, this will try to add new features if needed.
         * Will try to always have the "max_features" being tracked through KLT at each timestep.
         * Passed images should already be grayscaled.
         */
        void perform_detection_monocular(const cv::Mat &img0, std::vector<cv::KeyPoint> &pts0, std::vector<size_t> &ids0);

        /**
         * @brief Detects new features in the current stereo pair
         * @param img0 left image we will detect features on
         * @param img1 right image we will detect features on
         * @param pts0 left vector of currently extracted keypoints
         * @param pts1 right vector of currently extracted keypoints
         * @param ids0 left vector of feature ids for each currently extracted keypoint
         * @param ids1 right vector of feature ids for each currently extracted keypoint
         *
         * This does the same logic as the perform_detection_monocular() function, but we also enforce stereo contraints.
         * So we detect features in the left image, and then KLT track them onto the right image.
         * If we have valid tracks, then we have both the keypoint on the left and its matching point in the right image.
         * Will try to always have the "max_features" being tracked through KLT at each timestep.
         */
        void perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::KeyPoint> &pts0,
                                      std::vector<cv::KeyPoint> &pts1, std::vector<size_t> &ids0, std::vector<size_t> &ids1);

        /**
         * @brief KLT track between two images, and do RANSAC afterwards
         * @param img0 starting image
         * @param img1 image we want to track too
         * @param pts0 starting points
         * @param pts1 points we have tracked
         * @param mask_out what points had valid tracks
         *
         * This will track features from the first image into the second image.
         * The two point vectors will be of equal size, but the mask_out variable will specify which points are good or bad.
         * If the second vector is non-empty, it will be used as an initial guess of where the keypoints are in the second image.
         */
        void perform_matching(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::KeyPoint> &pts0,
                              std::vector<cv::KeyPoint> &pts1, std::vector<uchar> &mask_out);

        // Timing variables
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

        // Parameters for our FAST grid detector
        int threshold;
        int grid_x;
        int grid_y;

        // Minimum pixel distance to be "far away enough" to be a different extracted feature
        int min_px_dist;

    };


}


#endif /* OV_CORE_TRACK_KLT_H */