#ifndef POSE_ESTIMATION_POSEESTIMATIONPIPELINE_H
#define POSE_ESTIMATION_POSEESTIMATIONPIPELINE_H

#include "pcl/common/common.h"
#include "pcl/registration/registration.h"
#include <pcl/features/shot.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <vector>
#include <string>
#include <iostream>


using Matrix4 = Eigen::Matrix4f;
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using PointWithNormalsT = pcl::PointXYZLNormal;
using PointCloudWithNormalsT = pcl::PointCloud<PointWithNormalsT>;
using DescriptorT = pcl::SHOT352;
using DescriptorEstimatorT = pcl::SHOTEstimation<PointT, NormalT, DescriptorT>;
//using DescriptorT = pcl::FPFHSignature33;
using DescriptorCloudT = pcl::PointCloud<DescriptorT>;
using ColorHandlerT = pcl::visualization::PointCloudColorHandlerCustom<PointT>;


class PoseEstimationPipeline {
public:
    PoseEstimationPipeline();
    PoseEstimationPipeline(std::vector<PointCloudT::Ptr> mod_point_clouds,std::vector<NormalCloudT::Ptr> mod_normal_clouds);
    DescriptorCloudT::Ptr generate_descriptors(PointCloudT::Ptr &cloud, NormalCloudT::Ptr &normal_cloud, pcl::IndicesConstPtr keypoints_idx);
    void generate_model_descriptors();
    Matrix4 globalPoseEstimation(PointCloudT::Ptr &scene);
    Matrix4 localPoseEstimation(PointCloudT::Ptr &scene,PointCloudT::Ptr model);

    const pcl::IndicesConstPtr uniformIdxSampling(double leaf_size, PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out);
    const pcl::IndicesConstPtr uniformIdxSampling(double leaf_size, PointCloudT::Ptr &cloud_in);
    void plane_removal(PointCloudT::Ptr &scene,double radius);
    void generate_keypoints(PointCloudT::Ptr &cloud,PointCloudT::Ptr &keypoints,pcl::IndicesConstPtr &keypoints_idx);
    pcl::CorrespondencesPtr findCorrespondences(DescriptorCloudT::Ptr scene_descriptors,pcl::IndicesConstPtr scene_descriptors_idx,int m);
    pcl::visualization::PCLVisualizer::Ptr getViewer();
    void drawCorrespondences(PointCloudT::Ptr &source,PointCloudT::Ptr &target,NormalCloudT::Ptr &source_normal,NormalCloudT::Ptr &target_normal,pcl::Correspondences & correspondences, pcl::visualization::PCLVisualizer::Ptr &corrviewer);
    void addColorCloud(PointCloudT::Ptr &cloud,std::string id,pcl::visualization::PCLVisualizer::Ptr &vis,int r = 255, int g = 255, int b = 255);
    void addSpheres(PointCloudT::Ptr &cloud,std::string id,pcl::visualization::PCLVisualizer::Ptr &vis,int r , int g , int b );
    void saveCloud(PointCloudT::Ptr &cloud,int r, int g, int b);
    void saveCloud(PointCloudWithNormalsT ::Ptr &cloud,int r, int g, int b);

    // global pipeline constants
    const float descriptorSearchRadius = 0.05;
    const float normalEstimationRadius = 0.004;
    const float keyPointRes = 0.004;
    const float sceneRes = 0.002;
    const float planeRemovalInlierRadius = 0.005;
    const float cluster_tolerance = 0.005;
    const int min_cluster_size = 10000/8;
    const int max_cluster_size = 1000000;
    const int max_it = 5000;
    const float dist_epsilon = 0.0025;
    pcl::EuclideanClusterExtraction<PointT> euclid;
    pcl::ExtractIndices<PointT> extract;

    int number_of_models = 0;
    int numberOfViewers = 0;
    std::vector<PointCloudT::Ptr> model_point_clouds;
    std::vector<PointCloudT::Ptr> model_downsampled_point_clouds;
    std::vector<NormalCloudT::Ptr> model_normal_clouds;
    std::vector<PointCloudWithNormalsT::Ptr> model_point_clouds_with_normals;
    std::vector<PointCloudT::Ptr> model_keypoints;
    std::vector<pcl::IndicesConstPtr> model_keypoints_idx;
    std::vector<DescriptorCloudT::Ptr> model_keypoint_descriptors;

    DescriptorEstimatorT descriptor_estimator;
    pcl::NormalEstimation<PointT, NormalT> normalEstimation;

    pcl::visualization::PCLVisualizer::Ptr viewer ;
    std::vector<pcl::KdTreeFLANN<DescriptorT>::Ptr> model_descriptor_kdtrees;
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    int vis_clouds_idx=0;
    std::vector<std::pair<PointCloudT::Ptr,std::array<int,3>>> vis_hist;
    bool first_vis = true;


private:
    double computeCloudResolution(PointCloudT::Ptr &cloud);


};


#endif //POSE_ESTIMATION_POSEESTIMATIONPIPELINE_H
