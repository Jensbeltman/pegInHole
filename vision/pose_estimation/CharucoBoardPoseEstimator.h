//
// Created by jens on 4/24/20.
//

#ifndef POSE_ESTIMATION_CHARUCOBOARDPOSEESTIMATOR_H
#define POSE_ESTIMATION_CHARUCOBOARDPOSEESTIMATOR_H

#include "opencv2/core.hpp"
#include "opencv2/aruco/charuco.hpp"


using namespace std;

class CharucoBoardPoseEstimator {
public:
    CharucoBoardPoseEstimator(string calibration_file_path,string charuco_board_file_path);

    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_ptr;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_ptr;
    cv::Scalar marker_draw_color = cv::Scalar(255, 0, 0);
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;

    string window_name = "Charucoboard pose estimator";

    bool poseEstimate(cv::Mat image,cv::Vec3d &rvec, cv::Vec3d &tvec, bool imshow = false);

};


#endif //POSE_ESTIMATION_CHARUCOBOARDPOSEESTIMATOR_H
