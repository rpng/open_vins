#ifndef OV_CORE_TRACK_DESC_H
#define OV_CORE_TRACK_DESC_H


#include <opencv2/xfeatures2d.hpp>

#include "TrackBase.h"

/**
 * @namespace ov_core
 * @brief Core algorithms for Open VINS
 */
namespace ov_core {


    /**
     * @brief Descriptor-based visual tracking
     *
     * Here we use descriptor matching to track features from one frame to the next.
     * We track both temporally, and across stereo pairs to get stereo constraints.
     * Right now we use ORB descriptors as we have found it is the fastest when computing descriptors.
     */
    class TrackDescriptor : public TrackBase {

    public:

        /**
         * @brief Public default constructor
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         */
        TrackDescriptor(std::unordered_map<size_t, Eigen::Matrix<double,8,1>> camera_calib,
                        std::unordered_map<size_t, bool> camera_fisheye) :
                        TrackBase(camera_calib, camera_fisheye), threshold(10), grid_x(8), grid_y(5), knn_ratio(0.75) {}

        /**
         * @brief Public constructor with configuration variables
         * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
         * @param camera_fisheye map of camera_id => bool if we should do radtan or fisheye distortion model
         * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
         * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
         * @param fast_threshold FAST detection threshold
         * @param gridx size of grid in the x-direction / u-direction
         * @param gridy size of grid in the y-direction / v-direction
         * @param knnratio matching ratio needed (smaller value forces top two descriptors during match to be more different)
         */
        explicit TrackDescriptor(std::unordered_map<size_t, Eigen::Matrix<double,8,1>> camera_calib,
                                 std::unordered_map<size_t, bool> camera_fisheye,
                                 int numfeats, int numaruco, int fast_threshold, int gridx, int gridy, double knnratio) :
                                 TrackBase(camera_calib, camera_fisheye, numfeats, numaruco), threshold(fast_threshold),
                                 grid_x(gridx), grid_y(gridy), knn_ratio(knnratio) {}

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
        void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left,
                         size_t cam_id_right) override;


    protected:

        /**
         * @brief Detects new features in the current image
         * @param img0 image we will detect features on
         * @param pts0 vector of extracted keypoints
         * @param desc0 vector of the extracted descriptors
         * @param ids0 vector of all new IDs
         *
         * Given a set of images, and their currently extracted features, this will try to add new features.
         * We return all extracted descriptors here since we DO NOT need to do stereo tracking left to right.
         * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
         * See robust_match() for the matching.
         */
        void perform_detection_monocular(const cv::Mat &img0, std::vector<cv::KeyPoint> &pts0, cv::Mat &desc0,
                                         std::vector<size_t> &ids0);

        /**
         * @brief Detects new features in the current stereo pair
         * @param img0 left image we will detect features on
         * @param img1 right image we will detect features on
         * @param pts0 left vector of new keypoints
         * @param pts1 right vector of new keypoints
         * @param desc0 left vector of extracted descriptors
         * @param desc1 left vector of extracted descriptors
         * @param cam_id0 id of the first camera
         * @param cam_id1 id of the second camera
         * @param ids0 left vector of all new IDs
         * @param ids1 right vector of all new IDs
         *
         * This does the same logic as the perform_detection_monocular() function, but we also enforce stereo contraints.
         * We also do STEREO matching from the left to right, and only return good matches that are found in both the left and right.
         * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
         * See robust_match() for the matching.
         */
        void perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::KeyPoint> &pts0,
                                      std::vector<cv::KeyPoint> &pts1,
                                      cv::Mat &desc0, cv::Mat &desc1,
                                      size_t cam_id0, size_t cam_id1,
                                      std::vector<size_t> &ids0, std::vector<size_t> &ids1);

        /**
         * @brief Find matches between two keypoint+descriptor sets.
         * @param pts0 first vector of keypoints
         * @param pts1 second vector of keypoints
         * @param desc0 first vector of descriptors
         * @param desc1 second vector of decriptors
         * @param id0 id of the first camera
         * @param id1 id of the second camera
         * @param matches vector of matches that we have found
         *
         * This will perform a "robust match" between the two sets of points (slow but has great results).
         * First we do a simple KNN match from 1to2 and 2to1, which is followed by a ratio check and symmetry check.
         * Original code is from the "RobustMatcher" in the opencv examples, and seems to give very good results in the matches.
         * https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
         */
        void robust_match(std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> pts1,
                          cv::Mat &desc0, cv::Mat &desc1, size_t id0, size_t id1, std::vector<cv::DMatch> &matches);

        // Helper functions for the robust_match function
        // Original code is from the "RobustMatcher" in the opencv examples
        // https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
        void robust_ratio_test(std::vector<std::vector<cv::DMatch> > &matches);

        void robust_symmetry_test(std::vector<std::vector<cv::DMatch> > &matches1,
                                  std::vector<std::vector<cv::DMatch> > &matches2,
                                  std::vector<cv::DMatch> &good_matches);

        // Timing variables
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

        // Our orb extractor
        cv::Ptr<cv::ORB> orb0 = cv::ORB::create();
        cv::Ptr<cv::ORB> orb1 = cv::ORB::create();

        // Our descriptor matcher
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        // Parameters for our FAST grid detector
        int threshold;
        int grid_x;
        int grid_y;

        // The ratio between two kNN matches, if that ratio is larger then this threshold
        // then the two features are too close, so should be considered ambiguous/bad match
        double knn_ratio;

        // Descriptor matrices
        std::unordered_map<size_t, cv::Mat> desc_last;


    };


}


#endif /* OV_CORE_TRACK_DESC_H */