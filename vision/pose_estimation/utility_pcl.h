//
// Created by jens on 4/30/20.
//

#ifndef POSE_ESTIMATION_UTILITY_PCL_H
#define POSE_ESTIMATION_UTILITY_PCL_H
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/norms.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>


namespace utility_pcl {
    using namespace std;
    using namespace pcl;

    template<int N=153>
    static vector<std::pair<int,int>> findFeatureMatches(pcl::PointCloud<pcl::Histogram<N>> &object_features, pcl::PointCloud<pcl::Histogram<N>> &scene_features){
        vector<std::pair<int,int>> matches;
        const int feature_dim = N;
        float best_dist;
        int object_idx;
        int scene_idx;
        for (unsigned int o = 0; o<object_features.size();o++){
            best_dist = 1000000;
            for (unsigned int s = 0; s<scene_features.size();s++)
            {
                float dist = pcl::L2_Norm(object_features.at(o).histogram,scene_features.at(s).histogram,feature_dim);
                if (dist<best_dist){
                    best_dist=dist;
                    object_idx=o;
                    scene_idx=s;
                }
            }
            matches.push_back(std::pair<int,int>(object_idx,scene_idx));
        }

        return matches;
    }


    template <class PointCloudPtr,class PointType>
    static void load_all_point_clouds(vector<PointCloudPtr> &data, string pcd_folder_path,string type = "pcd",bool verbose = false){
        cv::String path(pcd_folder_path+"*."+type); //select only jpg
        vector<cv::String> fn;
        cv::glob(path,fn,true); // recurse
        int total_points = 0;
        for (size_t k=0; k<fn.size(); ++k)
        {
            PointCloudPtr cloud (new pcl::PointCloud<PointType>);
            if (pcl::io::loadPCDFile<PointType> (fn[k], *cloud) == -1) //* load the file
            {
                cout<<"Couldn't read file " << fn[k] << endl;
            }
            total_points += cloud->width * cloud->height;

            data.push_back(cloud);
            if (verbose){
                cout<<"Loaded point cloud with "<<cloud->width * cloud->height<<" points from "<<fn[k]<<endl;
            }
        }
        cout<<"Loaded "<<fn.size()<<" point clouds with and average of "<<total_points/fn.size()<<" points from "<<pcd_folder_path<<endl;
    }


    static void extract_normals(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &point_with_normals,pcl::PointCloud<pcl::Normal>::Ptr &normals ){
        normals->points.resize(point_with_normals->size());
        for (size_t i = 0; i < point_with_normals->points.size(); i++) {
            point_with_normals->points[i].x = normals->points[i].normal_x;
            point_with_normals->points[i].y = normals->points[i].normal_y;
            point_with_normals->points[i].z = normals->points[i].normal_z;
        }
}

}
#endif //POSE_ESTIMATION_UTILITY_PCL_H
