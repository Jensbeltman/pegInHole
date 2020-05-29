#ifndef POSE_ESTIMATION_UTILITY_OPENCV_H
#define POSE_ESTIMATION_UTILITY_OPENCV_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#include <iostream>
#include <string>

namespace utility_opencv {
        using namespace std;

        static void load_all_images(vector<cv::Mat> &data, string image_folder_path, string type = "jpg") {
            cv::String path(image_folder_path + "*." + type); //select only jpg
            vector<cv::String> fn;
            cv::glob(path, fn, true); // recurse
            int total_cols = 0;
            int total_rows = 0;
            for (size_t k = 0; k < fn.size(); ++k) {
                cv::Mat im = cv::imread(fn[k]);
                if (im.empty()) continue; //only proceed if sucsessful
                // you probably want to do some preprocessing
                data.push_back(im);
                total_rows += im.rows;
                total_cols += im.cols;
            }

            cout << "Loaded " << fn.size() << " images with average width(" << total_cols / fn.size()
                 << ") and average height(" << total_rows / fn.size() << ")" << endl;
        }


        static void load_all_meshes(vector<cv::viz::Mesh> &data, string mesh_folder_path, string type = "ply") {
            cv::String path(mesh_folder_path + "*." + type); //select only jpg
            vector<cv::String> fn;
            cv::glob(path, fn, true); // recurse
            int total_points = 0;

            for (size_t k = 0; k < fn.size(); ++k) {
                cv::viz::Mesh mesh = cv::viz::Mesh::load(fn[k]);
                if (mesh.polygons.empty()) continue; //only proceed if sucsessful
                // you probably want to do some preprocessing
                data.push_back(mesh);
                total_points += mesh.cloud.cols;
            }

            cout << "Loaded " << fn.size() << " mesh with and average of " << total_points / fn.size()
                 << " points from " << mesh_folder_path << endl;
        }


// Modified version of https://docs.opencv.org/4.3.0/df/d4a/tutorial_charuco_detection
                    static bool readCameraParameters(std::string filename, cv::Mat &camera_matrix, cv::Mat &distortion_coefficients) {
            cv::FileStorage fs(filename, 0);
            cv::FileNode camera_matrix_fn = fs["camera_matrix"];
            cv::FileNode distortion_coefficients_fn = fs["distortion_coefficients"];

            if (camera_matrix_fn.isSeq()) {
                for (int i = 0; i < camera_matrix_fn.size(); i++) {
                    cv::FileNode row = camera_matrix_fn[i]; // we only want the content of val1
                    for (int j = 0; j < row.size(); j += 1) { // read 3 consecutive values
                        camera_matrix.at<double>(i, j) = row[j].real();
                    }
                }
            } else {
                cout << "camera_matrix needs to be a sequence" << endl;
                return false;
            }
            if (distortion_coefficients_fn.isSeq()) {
                cout << distortion_coefficients_fn.size() << endl;
                for (int i = 0; i < distortion_coefficients_fn.size(); i++) {
                    distortion_coefficients.at<double>(0, i) = distortion_coefficients_fn[i].operator double();
                }
            } else {
                cout << "distortion_coefficients needs to be a sequence" << endl;
                return false;
            }

            cout << "Read camera parameters:" << endl;
            cout << "camera_matrix:\n" << camera_matrix << endl;
            cout << "distortion_coefficients: " << distortion_coefficients << endl;
            cout << endl;

            return true;

        }

        static void inverse_transformation(cv::Vec3d &tvec, cv::Vec3d &rvec) {
            cv::Mat rmat(3, 3, CV_64FC1);
            cv::Rodrigues(rvec, rmat);
            cv::transpose(rmat, rmat);
            cv::Rodrigues(rmat, rvec);
            tvec *= -1;
        }
}

#endif