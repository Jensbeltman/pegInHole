#ifndef POSE_ESTIMATION_GLOBAL_POSE_ESTIMATION_HPP
#define POSE_ESTIMATION_GLOBAL_POSE_ESTIMATION_HPP

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <ctime>
#include "utility_pcl.h"

using namespace std;
using namespace pcl;
using namespace utility_pcl;

Eigen::Matrix4d global_pose_estimation(pcl::PointCloud<PointXYZ>::Ptr &scene, PointCloud<PointXYZ>::Ptr &object, pcl::PointCloud<pcl::Normal>::Ptr &object_normals, pcl::PointCloud<PointXYZ>::Ptr &aligned_object) {
// -------- Normal estimation -------------------
// Create an empty kdtree representation, to be used by the normal estimator
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    search::KdTree<PointXYZ>::Ptr kdt_scene(new search::KdTree<PointXYZ>());
    search::KdTree<PointXYZ>::Ptr kdt_object(new search::KdTree<PointXYZ>());
// Containers to contain the results of the normals estimator
    PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>);

// Create the normal estimation class, and pass the input dataset to it
    NormalEstimation<PointXYZ, Normal> normalEstimation;
    normalEstimation.setKSearch(10); // Make the estimator use the k NN's

    normalEstimation.setInputCloud(scene);
    normalEstimation.setSearchMethod(kdt_scene);
    normalEstimation.compute(*scene_normals);
    cout << "Generated " << scene_normals->size() << " scene normals" << endl;


// -------- SpinImage computation -------------------
// Container for spin image results
    PointCloud<Histogram<153> >::Ptr scene_spin_images(new PointCloud<Histogram<153> >);
    PointCloud<Histogram<153> >::Ptr object_spin_images(new PointCloud<Histogram<153> >);
//spin image estimatOR
    SpinImageEstimation<PointXYZ, Normal, Histogram<153>> spinImageEstimation;

    spinImageEstimation.setRadiusSearch(0.05);

//Scene spin image estimation
    spinImageEstimation.setInputCloud(scene);
    spinImageEstimation.setInputNormals(scene_normals);
    spinImageEstimation.setSearchMethod(kdt_scene);
    spinImageEstimation.compute(*scene_spin_images);
    std::cout << "Scene features found: " << scene_spin_images->points.size() << std::endl;

// Object spin image estimation
    spinImageEstimation.setInputCloud(object);
    spinImageEstimation.setInputNormals(object_normals);
    spinImageEstimation.setSearchMethod(kdt_object);
    spinImageEstimation.compute(*object_spin_images);
    std::cout << "Object features found: " << object_spin_images->points.size() << std::endl;

// -------- Feature matching -------------------
    vector<pair<int, int>> match_idxs = findFeatureMatches(*object_spin_images, *scene_spin_images);
    cout << "Generated " << match_idxs.size() << " features matches based on l2 norm" << endl;


// ----- Downsampling object for RANSAC-------------------------
    const double leafeSize = 0.02;
    PointCloud<PointXYZ>::Ptr downsampled_object(new PointCloud<PointXYZ>);

    VoxelGrid<PointXYZ> voxelGrid;
    voxelGrid.setLeafSize(leafeSize, leafeSize, leafeSize);

    voxelGrid.setInputCloud(object);
    voxelGrid.filter(*downsampled_object);

    common::UniformGenerator<int> uniformGenerator(0, match_idxs.size());

    cout << "Sampled object with 1 cm resulted in " << downsampled_object->size() << " points" << endl;
    cout << "Compared to befores " << object->size() << " points" << endl;

// -------- RANSAC-------------------
    const int max_it = 10000;
    vector<int> source_idxs;
    vector<int> target_idxs;
    vector<int> nn_idx;
    vector<float> nn_dist;
    nn_idx.resize(1);
    nn_dist.resize(1);
    source_idxs.resize(3);
    target_idxs.resize(3);


    registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Ptr transformationEstimationSvd(
            new registration::TransformationEstimationSVD<PointXYZ, PointXYZ>);
    search::KdTree<PointXYZ>::Ptr scene_kd_tree(new search::KdTree<PointXYZ>);
    scene_kd_tree->setInputCloud(scene);

    registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Matrix4 transformation;
    registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Matrix4 best_transformation;


    const float dist_sqrt_epsilon = pow(0.005, 2);
    int inliers = 0;
    int max_inliers = 0;

    std::clock_t c_start = std::clock();
    for (int i = 0; i < max_it; i++) {
        inliers = 0;
// Generating a set of 3 uniformed sampled feature matches
        for (int m = 0; m < 3; m++) {
            pair<int, int> rand_match = match_idxs[uniformGenerator.run()];
            source_idxs[m] = rand_match.first;
            target_idxs[m] = rand_match.second;
        }

// Geneating and performing transformation based on matches
        transformationEstimationSvd->estimateRigidTransformation(*object, source_idxs, *scene, target_idxs,
                                                                 transformation);
        transformPointCloud(*downsampled_object, *aligned_object, transformation);

// Evaluating tranformation based on inliers
        for (auto point : *aligned_object) {
            scene_kd_tree->nearestKSearch(point, 1, nn_idx, nn_dist);
            if (nn_dist[0] < dist_sqrt_epsilon) {
                inliers++;
            }
        }
        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_transformation = transformation;
        }
    }

// ---------- Timing and evaluation ---------------------------
    std::clock_t c_end = std::clock();
    double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
    std::cout << "CPU time used: " << time_elapsed_ms / 1000.0 << " s\n";
    cout << "Best transformation with " << inliers << " is:\n" << transformation;
    transformPointCloud(*object, *aligned_object, best_transformation);

    return best_transformation.cast<double>();
}

#endif //POSE_ESTIMATION_GLOBAL_POSE_ESTIMATION_HPP
