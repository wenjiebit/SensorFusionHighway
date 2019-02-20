/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  std::vector<Car> cars;

  Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  cars.push_back(egoCar);
  
  Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  cars.push_back(car1);
  
  Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
  cars.push_back(car2);
 
  Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  cars.push_back(car3);
  

  bool renderScene = true;
  // render environment
  if(renderScene)
  {
  	renderHighway(viewer);
  	egoCar.render(viewer);
  	car1.render(viewer);
  	car2.render(viewer);
  	car3.render(viewer);
  }

  // create lidar
  Lidar* lidar = new Lidar(cars,0);

  // scan the enviroment using ray casting to create a new pcd 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();

  // create point cloud processor
  ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

  // render rays
  //renderRays(viewer,lidar->position,pointCloud);

  //renderPointCloud(viewer,pointCloud,"planeCloud");

  // save pcd files
  //pointProcessor->savePcd(pointCloud,"../src/sensors/data/pcd/simpleHighway.pcd");
  // load pcd files
  pcl::PointCloud<PointXYZ>::Ptr pointCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/simpleHighway.pcd");
  
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pointCloud, 100, 0.2, 0.3);

  vector<PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for(PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
  {
  	cout << "cluster size ";
  	pointProcessor->numPoints(cluster);
  	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
  	Box box = pointProcessor->BoundingBox(cluster);
  	renderBox(viewer,box,clusterId);
  	++clusterId;

  }
  renderPointCloud(viewer,segmentCloud.second,"planeCloud");
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	// ---------------------------------------------------------
 	// -----Open 3D viewer and display pcd from city block -----
 	// ---------------------------------------------------------
 	
 	// create point cloud processor
  	ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

  	pcl::PointCloud<PointXYZI>::Ptr pointCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  	pointCloud = pointProcessor->FilterCloud(pointCloud,0.2);

  	renderPointCloud(viewer,pointCloud,"pointCloud");

  	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pointCloud, 25, 0.4, 0.7);

  	//renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  	//renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  	vector<PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.5, 20, 400);

  	int clusterId = 0;
  	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  	for(PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  	{
  		//cout << "cluster size ";
  		//pointProcessor->numPoints(cluster);
  		//renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
  		Box box = pointProcessor->BoundingBox(cluster);
  		renderBox(viewer,box,clusterId);
  		++clusterId;
	
  	}


}


int main (int argc, char** argv)
{

	std::cout << "starting enviroment" << std::endl;

  	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
  	viewer->addCoordinateSystem (1.0);
	
  	// set camera position and angle
  	viewer->initCameraParameters();
  	// distance away in meters
  	int distance = 16;
  	viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);

  	//simpleHighway(viewer);
  	cityBlock(viewer);

  	while (!viewer->wasStopped ())
  	{
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}
}