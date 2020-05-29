//
// Created by jens on 4/24/20.
//

#include "CharucoBoardPoseEstimator.h"
#include "utility_opencv_aruco.h"
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "utility_opencv.h"


// Modified version of https://docs.opencv.org/4.3.0/df/d4a/tutorial_charuco_detection
using namespace std;
using namespace utility_opencv;

CharucoBoardPoseEstimator::CharucoBoardPoseEstimator(string calibration_file_path, string charuco_board_file_path) {
    camera_matrix = cv::Mat(cv::Size(3, 3), CV_64FC1);
    distortion_coefficients = cv::Mat(cv::Size(5, 1), CV_64FC1);
    (cv::Size(5, 1), CV_64FC1);
    bool readOk = readCameraParameters(calibration_file_path, camera_matrix, distortion_coefficients);
    if (!readOk) { std::cerr << "Invalid camera file" << std::endl; }

    charuco_board_ptr = charucoBoardFromFile(charuco_board_file_path);
    detector_parameters_ptr = cv::aruco::DetectorParameters::create();
}
bool CharucoBoardPoseEstimator::poseEstimate(cv::Mat image, cv::Vec3d &rvec, cv::Vec3d &tvec, bool imshow) {
    bool valid =  false;
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(image, charuco_board_ptr->dictionary, markerCorners, markerIds, detector_parameters_ptr);
    // if at least one marker detected
    if (markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charuco_board_ptr, charucoCorners,
                                             charucoIds, camera_matrix, distortion_coefficients);

        if (charucoIds.size() > 0) {
            if (imshow)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, marker_draw_color);
            cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco_board_ptr, camera_matrix,
                                                distortion_coefficients, rvec, tvec);
            valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco_board_ptr,
                                                             camera_matrix, distortion_coefficients, rvec,
                                                             tvec);
            // if charuco pose is valid
            if (valid){
                if (imshow){
                    cv::aruco::drawAxis(imageCopy, camera_matrix, distortion_coefficients, rvec, tvec, 0.1f);
                    cv::imshow(window_name, imageCopy);
                    cv::waitKey();
                }
            }
            else {
                std::cout << "Pose was not found to be valid"<<std::endl;
            }
        }
        else{
            std::cout << charucoIds.size() << " no charuco board markers where found !!!" << endl;
        }
    }


    return valid;
}






