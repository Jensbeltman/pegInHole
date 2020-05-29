#ifndef POSE_ESTIMATION_UTILITY_ARUCO_HPP
#define POSE_ESTIMATION_UTILITY_ARUCO_HPP

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>

namespace utility_opencv {
    inline namespace aruco {

        cv::Ptr<cv::aruco::Dictionary> charucoDicitonaryFromString(std::string dictionary_string);

//    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::CharucoBoard> charucoBoardFromFile(std::string filename) {
            cv::FileStorage fs(filename, cv::FileStorage::READ);
            if (!fs.isOpened())
                throw ("Error when trying to load  " + filename + " as charuco board");

            int squaresX, squaresY;
            double squareLength, markerLength;
            std::string dictionary_string;

            fs["squaresX"] >> squaresX;// = 14;
            fs["squaresY"] >> squaresY;// = 20;
            fs["squareLength"] >> squareLength;// = 0.02;
            fs["markerLength"] >> markerLength;// = 0.016;
            fs["dictionary"] >> dictionary_string;// = DICT_5X5_1000;

            cout << "Loaded CharucoBoard Parameters: " << endl;
            cout << "\t squaresX = " << squaresX << endl;
            cout << "\t squaresY = " << squaresY << endl;
            cout << "\t squareLength = " << squareLength << endl;
            cout << "\t markerLength = " << markerLength << endl;
            cout << "\t dictionary_string = " << dictionary_string << endl;
            cout << endl;


            return cv::aruco::CharucoBoard::create(squaresX, squaresY, markerLength, squareLength,
                                                   charucoDicitonaryFromString(dictionary_string));
        }

        cv::Ptr<cv::aruco::Dictionary> charucoDicitonaryFromString(std::string dictionary_string) {
            if (dictionary_string == "DICT_4X4_50")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            else if (dictionary_string == "DICT_4X4_100")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
            else if (dictionary_string == "DICT_4X4_250")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
            else if (dictionary_string == "DICT_4X4_1000")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
            else if (dictionary_string == "DICT_5X5_50")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
            else if (dictionary_string == "DICT_5X5_100")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
            else if (dictionary_string == "DICT_5X5_250")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
            else if (dictionary_string == "DICT_5X5_1000")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
            else if (dictionary_string == "DICT_6X6_50")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
            else if (dictionary_string == "DICT_6X6_100")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
            else if (dictionary_string == "DICT_6X6_250")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            else if (dictionary_string == "DICT_6X6_1000")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
            else if (dictionary_string == "DICT_7X7_50")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
            else if (dictionary_string == "DICT_7X7_100")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
            else if (dictionary_string == "DICT_7X7_250")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
            else if (dictionary_string == "DICT_7X7_1000")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
            else if (dictionary_string == "DICT_ARUCO_ORIGINAL")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            else if (dictionary_string == "DICT_APRILTAG_16h5")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
            else if (dictionary_string == "DICT_APRILTAG_25h9")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
            else if (dictionary_string == "DICT_APRILTAG_36h10")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
            else if (dictionary_string == "DICT_APRILTAG_36h11")
                return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        }

    }
}
#endif //POSE_ESTIMATION_UTILITY_ARUCO_HPP
