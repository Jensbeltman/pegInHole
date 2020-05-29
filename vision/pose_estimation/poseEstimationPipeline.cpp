#include "poseEstimationPipeline.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/registration.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/random.h>
#include <thread>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include<vector> // for vector
#include<algorithm> // for copy() and assign()
class SampleConsensusPrerejective;

using namespace std;
using namespace pcl;

PoseEstimationPipeline::PoseEstimationPipeline() {
    viewer = getViewer();
    viewer->registerKeyboardCallback(&PoseEstimationPipeline::keyboardEventOccurred, *this);
}

PoseEstimationPipeline::PoseEstimationPipeline(std::vector<PointCloudT::Ptr> mod_point_clouds, std::vector<NormalCloudT::Ptr> mod_normal_clouds) {
    viewer = getViewer();
    viewer->registerKeyboardCallback(&PoseEstimationPipeline::keyboardEventOccurred, *this);

   // descriptor_estimator.setNumberOfThreads(4);
    model_point_clouds = mod_point_clouds;
    model_normal_clouds = mod_normal_clouds;
    number_of_models = model_point_clouds.size();
    for (int i = 0;i<number_of_models;i++){
       // pcl::IndicesConstPtr is = uniformIdxSampling(sceneRes,model,model);
        int orig_model_points = model_point_clouds[i]->points.size();

        PointCloudT::Ptr model_downsampled(new PointCloudT);
        uniformIdxSampling(sceneRes*3,model_point_clouds[i],model_downsampled);
        model_downsampled_point_clouds.push_back(model_downsampled);

        PointCloudWithNormalsT::Ptr point_cloud_with_normals(new PointCloudWithNormalsT);
        pcl::concatenateFields(*(model_point_clouds[i]), *(model_normal_clouds[i]), *point_cloud_with_normals);
        model_point_clouds_with_normals.push_back(point_cloud_with_normals);
        cout<<"Model "<<i<<" downsampled from "<< orig_model_points << " to "<< model_downsampled->points.size() << "for inlier counting"<<endl;
    }
    generate_model_descriptors();
}

DescriptorCloudT::Ptr PoseEstimationPipeline::generate_descriptors(PointCloudT::Ptr &point_cloud,NormalCloudT::Ptr &normal_cloud, pcl::IndicesConstPtr keypoints_idx){
    descriptor_estimator.setInputCloud(point_cloud);
    descriptor_estimator.setInputNormals(normal_cloud);
    descriptor_estimator.setIndices(keypoints_idx);
    descriptor_estimator.setRadiusSearch(descriptorSearchRadius);

    DescriptorCloudT::Ptr descriptors(new DescriptorCloudT());
    descriptor_estimator.compute(*descriptors);
    return descriptors;
}

void PoseEstimationPipeline::generate_keypoints(PointCloudT::Ptr& cloud,PointCloudT::Ptr &keypoints,pcl::IndicesConstPtr &keypoints_idx){
    float cloud_resolution = 0.001;
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT> ());
    ISSKeypoint3D<PointT, PointT> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (3 * cloud_resolution);
    iss_detector.setNonMaxRadius (1 * cloud_resolution);

    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (4);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute (*keypoints);
    pcl::PointIndicesConstPtr pointIndices = iss_detector.getKeypointsIndices();
    pcl::IndicesConstPtr tmp_Indices(new const pcl::Indices(pointIndices->indices));
    keypoints_idx = tmp_Indices;
}

void PoseEstimationPipeline::generate_model_descriptors() {
    for (int i = 0;i<number_of_models;i++){
        PointCloudT::Ptr keypoints(new PointCloudT);
//        pcl::IndicesConstPtr keypoints_idx(new const pcl::Indices);
//        generate_keypoints(model_point_clouds[i],keypoints,keypoints_idx);
        pcl::IndicesConstPtr keypoints_idx =  uniformIdxSampling(keyPointRes,model_point_clouds[i],keypoints);
        model_keypoints.push_back(keypoints);
        model_keypoints_idx.push_back(keypoints_idx);
        model_keypoint_descriptors.push_back( generate_descriptors(model_point_clouds[i], model_normal_clouds[i], model_keypoints_idx[i]) );
        pcl::KdTreeFLANN<DescriptorT>::Ptr kdtree(new pcl::KdTreeFLANN<DescriptorT>);

        model_descriptor_kdtrees.push_back(kdtree);
        model_descriptor_kdtrees.back()->setInputCloud(model_keypoint_descriptors.back());
        cout<<"Generated "<<keypoints->size()<<" keypoint for model "<<i<<endl;
    }
    cout<<"Model desciptors generated"<<endl;
}

Matrix4 PoseEstimationPipeline::globalPoseEstimation(PointCloudT::Ptr &scene_point_cloud) {
    int orig_scene_points = scene_point_cloud->points.size();
    saveCloud(scene_point_cloud,255,0,0);
    cout<<"Pose estimating scene with "<<orig_scene_points<<endl;
    // ---- Scene filtering ----
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(Eigen::Vector4f(-0.5,-0.5,-0.7,0));
    cropBox.setMax(Eigen::Vector4f(0.5,0.5,0,0));
    cropBox.setInputCloud(scene_point_cloud);
    cropBox.filter(*scene_point_cloud);
    cout<<"CropBox on scene reduced scene points to  "<<scene_point_cloud->points.size()<<endl;

    uniformIdxSampling(0.002,scene_point_cloud,scene_point_cloud);

    saveCloud(scene_point_cloud,255,255,0);
  //  cout<<"Uniform sampling with leafe size "<< sceneRes << " on scene reduced scene points to  "<<scene_point_cloud->points.size()<<endl;

    plane_removal(scene_point_cloud,planeRemovalInlierRadius);
    saveCloud(scene_point_cloud,0,255,0);
//    pcl::StatisticalOutlierRemoval<PointT> statisticalOutlierRemoval;
//    statisticalOutlierRemoval.setInputCloud (scene_point_cloud);
//    statisticalOutlierRemoval.setMeanK (25);
//    statisticalOutlierRemoval.setStddevMulThresh (0.05);
//    statisticalOutlierRemoval.filter (*scene_point_cloud);


    cout<<"Scene descriptors calculated"<<endl;

    // ---- Scene clustering ----
    vector<pcl::PointIndices> cluster_indices;
    euclid.setInputCloud(scene_point_cloud);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(cluster_indices);

    cout << cluster_indices.size() << endl;


    vector<PointCloudT::Ptr> cluster_point_clouds;
    vector<PointCloudT::Ptr> cluster_key_points;
    vector<NormalCloudT::Ptr> cluster_normal_clouds;
    vector<pcl::IndicesConstPtr> cluster_keypoints_idx;
    vector<DescriptorCloudT::Ptr> cluster_keypoint_descriptors;


    for (int c =0; c < cluster_indices.size(); c++){
        PointCloudT::Ptr point_cloud(new PointCloudT);
        NormalCloudT::Ptr normal_cloud(new NormalCloudT);
        uniformIdxSampling(0.002,point_cloud,point_cloud);

        pcl::PointIndicesConstPtr indices(new const pcl::PointIndices(cluster_indices[c]));
        extract.setInputCloud(scene_point_cloud);
        extract.setIndices(indices);
        extract.filter(*point_cloud);
        cout<<"Cluster "<<c<<" has "<< point_cloud->width*point_cloud->height <<" points "<<endl;

        normalEstimation.setInputCloud(point_cloud);
        normalEstimation.setRadiusSearch(normalEstimationRadius);
        pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.useSensorOriginAsViewPoint();
        normalEstimation.compute(*normal_cloud);
        cout<<"Cluster "<<c<<" normal estimation done"<<endl;

        PointCloudT::Ptr keypoints(new PointCloudT);


        pcl::IndicesConstPtr keypoints_idx = uniformIdxSampling(keyPointRes,point_cloud,keypoints);
       // generate_keypoints(point_cloud,keypoints,keypoints_idx);

        // descriptor estimation object.
        DescriptorCloudT::Ptr descriptors = generate_descriptors(point_cloud,normal_cloud,keypoints_idx);

        cluster_point_clouds.push_back(point_cloud);
        cluster_normal_clouds.push_back(normal_cloud);
        cluster_keypoint_descriptors.push_back(descriptors);
        cluster_keypoints_idx.push_back(keypoints_idx);
        cluster_key_points.push_back(keypoints);

        cout<<"Generated "<< keypoints->points.size() <<" keypoints in cluster "<<c<<endl;

        addColorCloud(point_cloud,"cluster"+to_string(c),viewer,0,255,0);
        //viewer->addPointCloudNormals<PointT,NormalT>(point_cloud,normal_cloud,10,0.003 ,"clusterNormals"+to_string(c));
    }

    pcl::console::print_highlight ("Starting alignment...\n");

    Matrix4 transformation;
    registration::TransformationEstimationSVD<PointT, PointT>::Ptr transformationEstimationSvd(new registration::TransformationEstimationSVD<PointT, PointT>);

    vector<int> nn_idx;
    vector<float> nn_dist;
    nn_idx.resize(1,0);
    nn_dist.resize(1,0);

    const int nrClusters = cluster_point_clouds.size();
    const int nrModels = model_point_clouds.size();
    vector<vector<double>> max_inlier_percent;
    vector<vector<Matrix4>> best_transformation;
    vector<vector<pcl::CorrespondencesPtr>> best_correspondence_triplet;
    max_inlier_percent.resize(nrClusters);
    best_transformation.resize(nrClusters);
    best_correspondence_triplet.resize(nrClusters);
    for (int i = 0;i<nrClusters;i++){
        max_inlier_percent[i].resize(nrModels);
        best_transformation[i].resize(nrModels);
    }

    // ----- RANSAC ------
    PointCloudT::Ptr aligned_model(new PointCloudT);
    PointCloudWithNormalsT::Ptr aligned_model_with_normals(new PointCloudWithNormalsT);

    int inliers = 0;
    int max_inliers = 0;
    int rnd=0;
    for (int c = 0; c<cluster_point_clouds.size();c++){
        int cluster_points = cluster_point_clouds[c]->points.size(); // Used for inlier percent
        for (int m = 0; m<nrModels;m++){
            std::clock_t c_start_corr = std::clock();
            pcl::CorrespondencesPtr correspondences = findCorrespondences(cluster_keypoint_descriptors[c],cluster_keypoints_idx[c],m);

            cout << "Found " << correspondences->size() << " between cluster " << c << " and model " << m << " of "<< cluster_keypoint_descriptors[c]->width << "/" << model_keypoint_descriptors[m]->width << endl;
//            pcl::visualization::PCLVisualizer::Ptr corrviewer = getViewer();
//
//            drawCorrespondences(cluster_point_clouds[c],model_point_clouds[m],cluster_normal_clouds[c],model_normal_clouds[m],*correspondences,corrviewer);
//            viewer->spinOnce();

            if (correspondences->size()>=3) {
                std::clock_t c_start_sac = std::clock();
                cout<<"Running correspondance rejector"<<endl;
                pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
                rejector.setInputSource(model_point_clouds[m]);
                rejector.setInputTarget(cluster_point_clouds[c]);
                rejector.setInputCorrespondences(correspondences);
                rejector.setMaximumIterations(max_it);
                rejector.setInlierThreshold(dist_epsilon);
                rejector.getCorrespondences(*correspondences);
                best_transformation[c][m] = rejector.getBestTransformation();
                std::cout << best_transformation[c][m] << std::endl;


                pcl::transformPointCloud(*(model_point_clouds_with_normals[m]),*aligned_model_with_normals, best_transformation[c][m]);
                saveCloud(aligned_model_with_normals,0,0,255);

                PointCloudWithNormalsT::Ptr cluster_cloud_with_normals(new PointCloudWithNormalsT);
                pcl::concatenateFields(*(cluster_point_clouds[c]), *(cluster_normal_clouds[c]), *cluster_cloud_with_normals);
//// ---------- Timing and evaluation ---------------------------
                std::clock_t c_start_icp = std::clock();
                pcl::IterativeClosestPointWithNormals<PointWithNormalsT, PointWithNormalsT> icp;
                icp.setInputSource(aligned_model_with_normals);
                icp.setInputTarget(cluster_cloud_with_normals);
                icp.setMaximumIterations (max_it);
                icp.setTransformationEpsilon (1e-12);
                icp.setMaxCorrespondenceDistance (dist_epsilon);
                icp.setUseSymmetricObjective (true) ;
                icp.setEuclideanFitnessEpsilon (1);
                icp.setRANSACOutlierRejectionThreshold (0.1);
                icp.align(*aligned_model_with_normals);


                saveCloud(aligned_model_with_normals,0,0,255);
                if (icp.hasConverged())
                {
                    std::cout << "ICP converged." << std::endl;
//                    << "The score is " << icp.getFitnessScore() << std::endl;
//                    std::cout << "Transformation matrix:" << std::endl;
//                    std::cout << icp.getFinalTransformation() << std::endl;
                }
                else std::cout << "ICP did not converge." << std::endl;


                std::clock_t c_end = std::clock();
                double time_elapsed_ms_total = 1000.0 * (c_end - c_start_corr) / CLOCKS_PER_SEC;
                double time_elapsed_ms_corr = 1000.0 * (c_start_sac - c_start_corr) / CLOCKS_PER_SEC;
                double time_elapsed_ms_sac = 1000.0 * (c_start_icp - c_start_sac) / CLOCKS_PER_SEC;
                double time_elapsed_ms_icp = 1000.0 * (c_end - c_start_icp) / CLOCKS_PER_SEC;
                std::cout << "CPU time used: " <<"\nCorrMatch: " <<time_elapsed_ms_corr / 1000.0 <<"\nSAC: " <<time_elapsed_ms_sac / 1000.0 <<"\nICP: " <<time_elapsed_ms_icp / 1000.0 <<"\nTotal: " <<time_elapsed_ms_total / 1000.0 << " s\n";

            }
        }
    }

    viewer->setCameraPosition(0, 0, 0, 0, 0, -0.25, 0,1,0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return best_transformation[0][0];
}


const pcl::IndicesConstPtr PoseEstimationPipeline::uniformIdxSampling(double leaf_size, PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out) {
    int initial_size = cloud_in->width * cloud_in->height;

    pcl::UniformSampling<PointT> uniformSampler(true);

    uniformSampler.setInputCloud(cloud_in);
    uniformSampler.setRadiusSearch(leaf_size);
    uniformSampler.filter(*cloud_out);

    // Removed indices, not guaranteed to be sorted
    pcl::PointIndices removed_indices;
    uniformSampler.getRemovedIndices(removed_indices);
    std::sort(removed_indices.indices.begin(), removed_indices.indices.end());

    const pcl::IndicesConstPtr input_indices = uniformSampler.getIndices();
    pcl::IndicesPtr retained_indices(new pcl::Indices);
    // Compute retained indices as a set difference between all and removed
    std::set_difference(input_indices->begin(),
                        input_indices->end(),
                        removed_indices.indices.begin(),
                        removed_indices.indices.end(),
                        std::inserter(*retained_indices, retained_indices->begin()));

//    cout<<"Point cloud downsampled from "<<initial_size<<" to "<< retained_indices->size()<<endl;

    return retained_indices;
}

const pcl::IndicesConstPtr PoseEstimationPipeline::uniformIdxSampling(double leaf_size, PointCloudT::Ptr &cloud_in) {
    PointCloudT::Ptr temp(new PointCloudT);
    return uniformIdxSampling(leaf_size,cloud_in,temp);
}

bool descriptor_is_finite(pcl::SHOT352 &desc){
    return std::isfinite(desc.descriptor[0]);
}
bool descriptor_is_finite(pcl::FPFHSignature33 &desc){
    return std::isfinite(desc.histogram[0]);
}

pcl::CorrespondencesPtr PoseEstimationPipeline::findCorrespondences(DescriptorCloudT::Ptr scene_descriptors,
                                                                    pcl::IndicesConstPtr scene_descriptors_idx,
                                                                    int m) {

    // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
    pcl::KdTreeFLANN<DescriptorT>::Ptr matching =  model_descriptor_kdtrees[m];
    // A Correspondence object stores the indices of the query and the match,
    // and the distance/weight.
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

    // Check every descriptor computed for the scene.
    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neighbors(1);
        std::vector<float> squaredDistances(1);
        // Ignore NaNs.
        if (descriptor_is_finite(scene_descriptors->at(i)))
        {
            // Find the nearest neighbor (in descriptor space)...
            int neighborCount = matching->nearestKSearch(*scene_descriptors,i, 1, neighbors, squaredDistances);
            if (neighborCount > 0)
            {
                pcl::Correspondence correspondence((*model_keypoints_idx[m])[neighbors[0]], (*scene_descriptors_idx)[static_cast<int>(i)], squaredDistances[0]);
                correspondences->push_back(correspondence);
            }
        }
    }
    //std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;

    return correspondences;
}

void PoseEstimationPipeline::plane_removal(PointCloudT::Ptr &scene,double radius) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (radius);

    seg.setInputCloud (scene);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(scene);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*scene);
}


void PoseEstimationPipeline::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{

    if (vis_hist.size() != 0) {
        if (first_vis){
            cout << vis_hist.size() << " visclouds" << endl;
            std::array<int,3> color = vis_hist[vis_clouds_idx].second;
            addColorCloud(vis_hist[vis_clouds_idx].first,"cloud",viewer,color[0],color[1],color[2]);
            first_vis = false;

        }
        else if (event.getKeySym() == "n" && event.keyDown()) {
            if (vis_clouds_idx != vis_hist.size() - 1) {
                vis_clouds_idx++;
                viewer->removePointCloud("cloud");
                std::array<int,3> color = vis_hist[vis_clouds_idx].second;
                addColorCloud(vis_hist[vis_clouds_idx].first,"cloud",viewer,color[0],color[1],color[2]);

            }
        }
        else if (event.getKeySym() == "b" && event.keyDown()) {
            if (vis_clouds_idx != 0) {
                vis_clouds_idx--;
                viewer->removePointCloud("cloud");
                std::array<int,3> color = vis_hist[vis_clouds_idx].second;
                addColorCloud(vis_hist[vis_clouds_idx].first,"cloud",viewer,color[0],color[1],color[2]);
            }
        }
    }
}


pcl::visualization::PCLVisualizer::Ptr PoseEstimationPipeline::getViewer(){
    pcl::visualization::PCLVisualizer::Ptr new_viewer(new pcl::visualization::PCLVisualizer("Viewer"+std::to_string(numberOfViewers)));
    numberOfViewers++;
    new_viewer->setBackgroundColor (0, 0, 0);
    new_viewer->addCoordinateSystem (0.05);
    new_viewer->initCameraParameters ();
    return new_viewer;
}

void PoseEstimationPipeline::drawCorrespondences(PointCloudT::Ptr &source,PointCloudT::Ptr &target, NormalCloudT::Ptr &source_normal,NormalCloudT::Ptr &target_normal, pcl::Correspondences & correspondences, pcl::visualization::PCLVisualizer::Ptr &corrviewer){

    corrviewer->removeAllPointClouds();
    corrviewer->removeAllShapes();
//I am using this boolean to alter the color of the spheres
    bool alter=false;
    for (size_t i =0; i <correspondences.size(); i++)
    {

        const PointT & p_src = source->points.at(correspondences[i].index_match);
        const PointT & p_tgt = target->points.at(correspondences[i].index_query);

        // Generate a unique string for each line
        std::stringstream ss ("line");
        ss << i;

        std::stringstream sss ("spheresource");
        sss << i;

        std::stringstream ssss ("spheretarget");
        float sphere_radius = 0.0005;
        ssss << i;

            //this is for red lines and spheres
            corrviewer->addSphere<PointT>(p_src,sphere_radius,255,0,0,sss.str());
            corrviewer->addSphere<PointT>(p_tgt,sphere_radius,255,0,0,ssss.str());
            corrviewer->addLine<PointT> (p_src, p_tgt, 0, 0, 255, ss.str ());

    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> white( 50, 50, 200);

    corrviewer->addPointCloud<PointT>(source,white,"source");
    corrviewer->addPointCloud<PointT>(target,white,"target");

    corrviewer->addPointCloudNormals<PointT,NormalT>(source,source_normal,5,0.002,"source_normal");
    corrviewer->addPointCloudNormals<PointT,NormalT>(target,target_normal,5,0.002,"target_normal");
    corrviewer->resetCamera();
}

void PoseEstimationPipeline::addColorCloud(PointCloudT::Ptr &cloud,string id,pcl::visualization::PCLVisualizer::Ptr &vis,int r , int g , int b ){
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, r, g, b);
    vis->addPointCloud<PointT>(cloud,color,id);
}
void PoseEstimationPipeline::addSpheres(PointCloudT::Ptr &cloud,string id,pcl::visualization::PCLVisualizer::Ptr &vis,int r , int g , int b ){
    float sphere_radius = 0.0005;

    for (size_t i =0; i <cloud->points.size(); i++)
    {
        std::stringstream ss ("spheresource");
        ss << id << i;
        const PointT & p = cloud->points.at(i);
        vis->addSphere<PointT>(p,sphere_radius,r,g,b,ss.str());
    }
}


void PoseEstimationPipeline::saveCloud(PointCloudT ::Ptr &cloud, int r=255, int g=255, int b=255) {
    PointCloudT::Ptr temp(new PointCloudT);
    copyPointCloud(*cloud,*temp);
    vis_hist.push_back(std::make_pair(temp,std::array<int,3>{r,g,b}));
}
void PoseEstimationPipeline::saveCloud(PointCloudWithNormalsT ::Ptr &cloud, int r=255, int g=255, int b=255) {
    PointCloudT::Ptr temp(new PointCloudT);
    copyPointCloud(*cloud,*temp);
    vis_hist.push_back(std::make_pair(temp,std::array<int,3>{r,g,b}));
}
