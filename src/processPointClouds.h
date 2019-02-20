// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream> 
#include <string>  
#include <sstream>
#include <ctime>
#include <chrono>
#include "render/box.h"

using namespace pcl;
using namespace std;

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename PointCloud<PointT>::Ptr cloud);

    typename PointCloud<PointT>::Ptr FilterCloud(typename PointCloud<PointT>::Ptr cloud, float filterRes);

    std::pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> SegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, float downSampleRatio);

    vector<typename PointCloud<PointT>::Ptr> Clustering(typename PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename PointCloud<PointT>::Ptr loadPcd(std::string file);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */