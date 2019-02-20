// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

using namespace pcl;
using namespace std;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename PointCloud<PointT>::Ptr cloud)
{
  cout << cloud->points.size() << endl;
}

template<typename PointT>
typename PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename PointCloud<PointT>::Ptr cloud, float filterRes)
{

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // Create the filtering object: downsample the dataset using a leaf size of .2m
  VoxelGrid<PointT> vg;
  typename PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
  //std::cout << typeid(vg).name() << endl;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  cout << "filtering took " << elapsedTime.count() << " milliseconds" << endl;

  return cloudFiltered;

}

template<typename PointT>
std::pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, float downSampleRatio)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

	// Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename PointCloud<PointT>::Ptr obstCloud (new PointCloud<PointT> ());
  //  Obstacle cloud starts out equal to input cloud
	*obstCloud = *cloud;

  typename PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

  // Create the segmentation object for the planar model and set all the parameters
  //
  //
  SACSegmentation<PointT> seg;
  PointIndices::Ptr inliers {new pcl::PointIndices};
  ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  //
  //

  int numPoints = obstCloud->points.size();
  
  while (obstCloud->points.size() > downSampleRatio * numPoints)
  {
     // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(obstCloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Place plane inliers indices into plane cloud
    for(int index : inliers->indices)
      planeCloud->points.push_back(cloud->points[index]);
  	
    // Extract the planar inliers from the obstacle cloud
    ExtractIndices<PointT> extract;
    extract.setInputCloud (obstCloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << endl;

  pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
  return segResult;
}

template<typename PointT>
vector<typename PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  // Creating the KdTree object for the search method of the extraction
  typename search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  
  vector<PointIndices> clusterIndices;
  EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); 
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  vector<typename PointCloud<PointT>::Ptr> clusters;

  for(PointIndices getIndices: clusterIndices)
  {
      typename PointCloud<PointT>::Ptr cloudCluster (new PointCloud<PointT>);

      for(int index : getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]); 

      cloudCluster->width = cloudCluster->points.size ();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);

  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << endl;

  return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename PointCloud<PointT>::Ptr cluster)
{
  // source of code http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

	// find bounding box for one of the clusters
  Eigen::Quaternionf bboxQuaternion;
  Eigen::Vector3f bboxTransform;
  PointT minPoint, maxPoint;
  
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
  
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  
  // Final transform
  Eigen::Quaternionf Quaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI

  Box box;
  box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  box.bboxQuaternion = Quaternion;
  box.cube_length = maxPoint.x - minPoint.x;
  box.cube_width = maxPoint.y - minPoint.y;
  box.cube_height = maxPoint.z - minPoint.z;

  return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}