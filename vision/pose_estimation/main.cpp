#include <iostream>
#include <opencv2/opencv.hpp>
#include "utility_opencv.h"
#include "utility_pcl.h"

#include "CharucoBoardPoseEstimator.h"
#include "poseEstimationPipeline.h"
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <thread>
#include "pcl/common/common.h"
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>


typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

typedef pcl::PointXYZLNormal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;

typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> PointCloudNormals;

using namespace std;
using namespace pcl;
using namespace utility_pcl;
using namespace utility_opencv;

void makeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
              double standard_deviation);

int main(int argc, char *argv[]) {

    string camera_calibration_json;
    string charuco_board_json;
    string color_data_path;
    string depth_data_path;
    string model_data_path;

    if (argc != 6)
        cout << "usage: " << argv[0] << "<filename,camera_calibration_json>" << "<charuco_board_json>"
             << "<color_data_path>" << "<depth_data_path>" << "<model_data_path>" << endl;
    else {
        camera_calibration_json = argv[1];
        charuco_board_json = argv[2];
        color_data_path = argv[3];
        depth_data_path = argv[4];
        model_data_path = argv[5];
    }

    CharucoBoardPoseEstimator charucoBoardPoseEstimator(camera_calibration_json, charuco_board_json);
    vector<cv::Mat> color_data;
    vector<PointCloud<PointXYZ>::Ptr> depth_data;
    vector<PointCloudRGB::Ptr> depth_data_rgb;

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_data_points;
    vector<PointCloudNormals::Ptr> model_data_normals;
    vector<cv::viz::Mesh> model_mesh_data;

    load_all_images(color_data, color_data_path);
    load_all_point_clouds<PointCloudRGB::Ptr, PointRGB>(depth_data_rgb, depth_data_path);
    load_all_point_clouds<PointCloud<PointXYZ>::Ptr, PointXYZ>(depth_data, depth_data_path);

    load_all_point_clouds<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointXYZ>(model_data_points, model_data_path,"pcd",true);
    load_all_point_clouds<PointCloudNormals::Ptr, Normal>(model_data_normals, model_data_path);

    load_all_meshes(model_mesh_data, model_data_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr perfect(new pcl::PointCloud<pcl::PointXYZ>);
//
    pcl::copyPointCloud(*model_data_points[0],*perfect);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero();
    transformation(0,2) = -1;
    transformation(1,1) = 1;
    transformation(2,0) = 1;
    transformation(3,3) = 1;
    transformation(2,3) = -0.2;
    cout<<transformation<<endl;


    transformPointCloud(*perfect,*perfect,transformation);

//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addCoordinateSystem (0.05);
//    viewer->initCameraParameters ();
//
//    viewer->addPointCloud<pcl::PointXYZ>(perfect,"scene");
//    viewer->spin();

//    viewer->resetCamera();
//    viewer->spin();

    int modelNr = 1;
    int numberOfViewers=0;
//    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_data_points_subset;
//    vector<pcl::PointCloud<pcl::Normal>::Ptr> model_data_normals_subset;
//    //model_data_points_subset.push_back(model_data_points[0]);
//    model_data_points_subset.push_back(model_data_points[1]);
//    model_data_normals_subset.push_back(model_data_normals[1]);

   // poseEstimationPipeline.globalPoseEstimation(depth_data[0]);

/*    for(pcl::PointCloud<pcl::PointXYZ>::Ptr pcp:depth_data){
        PoseEstimationPipeline poseEstimationPipeline(model_data_points,model_data_normals);
        poseEstimationPipeline.globalPoseEstimation(pcp);

    }*/
//    makeNoise(perfect,perfect,0.001);
//    poseEstimationPipeline.globalPoseEstimation(perfect);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(model_data_points[modelNr], 50, 50, 200);
//    viewer->addPointCloud<pcl::PointXYZ> (model_data_points[modelNr], white, "model");
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(poseEstimationPipeline.model_keypoints[modelNr], 0, 200, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (poseEstimationPipeline.model_keypoints[modelNr], green, "keypoints");


    for (cv::Mat image : color_data) {
        cv::Vec3d rvec, tvec;
        charucoBoardPoseEstimator.poseEstimate(image, rvec, tvec, true);
        cout << "tvec:" << tvec << "\trvec:" << rvec << endl;
        cv::waitKey(100);
    }

    return 0;
}

void makeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
         double standard_deviation)
{
    pcl::console::TicToc tt;
    tt.tic ();

    cout<<"Adding Gaussian noise with mean 0.0 and standard deviation"<< standard_deviation;

    output->points.resize (input->points.size ());
    output->header = input->header;
    output->width = input->width;
    output->height = input->height;


    boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
    boost::normal_distribution<> nd (0, standard_deviation);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);


    for (size_t point_i = 0; point_i < input->points.size (); ++point_i)
    {
        output->points[point_i].x = input->points[point_i].x + static_cast<float> (var_nor ());
        output->points[point_i].y = input->points[point_i].y + static_cast<float> (var_nor ());
        output->points[point_i].z = input->points[point_i].z + static_cast<float> (var_nor ());
    }
}

