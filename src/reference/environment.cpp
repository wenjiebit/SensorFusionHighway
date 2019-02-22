/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"



void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZ>* pointProcessor)
{
	// ----------------------------------------------------
  	// -----Open 3D viewer and display simple highway -----
  	// ----------------------------------------------------
	
	// RENDER OPTIONS
  	bool    render_scene = false;
  	bool 		 do_scan = false;
  	bool  	 render_rays = false;
  	bool 	render_input = true;
  	bool     render_obst = false;
 	bool    render_plane = true;
  	bool render_clusters = true;
  	bool 	  render_box = true;
  	bool 	 	save_pcd = false;

  	std::vector<Car> cars;
	
  	Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  	cars.push_back(egoCar);
  	
  	Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  	cars.push_back(car1);
  	
  	Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
  	cars.push_back(car2);
 	
  	Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  	cars.push_back(car3);
  	
  	// render environment
  	if(render_scene)
  	{
  		renderHighway(viewer);
  		egoCar.render(viewer);
  		car1.render(viewer);
  		car2.render(viewer);
  		car3.render(viewer);
  	}
	
  	// Create lidar
  	Lidar* lidar = new Lidar(cars,0);
	
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

  	// Scan the enviroment using ray casting to create a new pcd or load previous pcd
  	if(do_scan)
  		inputCloud = lidar->scan(); 
	else
		inputCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/simpleHighway.pcd");

  	if(render_rays)
  		renderRays(viewer,lidar->position, inputCloud);
	
  	if(render_input)
  		renderPointCloud(viewer, inputCloud,"inputCloud");
	
  	if(save_pcd)
  		pointProcessor->savePcd(inputCloud,"../src/sensors/data/pcd/simpleHighway.pcd");
  	
  	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

  	if(render_obst)
  		renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  	if(render_plane)
  		renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
	
  	int clusterId = 0;
  	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	
  	for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
  	{
  		if(render_clusters)
  		{
  			std::cout << "cluster size ";
  			pointProcessor->numPoints(cluster);
  			renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
  		}
  		if(render_box)
  		{
  			Box box = pointProcessor->BoundingBox(cluster);
  			renderBox(viewer,box,clusterId);
  		}
  		++clusterId;
	
  	}
  	renderPointCloud(viewer,segmentCloud.second,"planeCloud");
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
	// ---------------------------------------------------------
 	// -----Open 3D viewer and display pcd from city block -----
 	// ---------------------------------------------------------

	// RENDER OPTIONS
 	bool    render_input = false;
 	bool     render_obst = false;
 	bool    render_plane = true;
 	bool render_clusters = true;
 	bool 	  render_box = true;

 	if(render_input)
 		renderPointCloud(viewer,inputCloud,"inputCloud");

  	inputCloud = pointProcessor->FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30,  7,  1, 1));

  	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 80, 0.2);

  	if(render_obst)
  		renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  	if(render_plane)
  		renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
  	
  	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.4, 10, 700);

  	int clusterId = 0;
  	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  	{
  		if(render_clusters)
  		{
  			std::cout << "cluster size ";
  			pointProcessor->numPoints(cluster);
  			renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
  		}
  		if(render_box)
  		{
  			Box box = pointProcessor->BoundingBox(cluster);
  			renderBox(viewer,box,clusterId);
  		}
  		++clusterId;
	
  	}
  	
}

enum Environment
{
	SimpleHighway, CityBlock
};

int main (int argc, char** argv)
{

	std::cout << "starting enviroment" << std::endl;

	// CHANGE ARGUMENT HERE TO SWITCH RUN ENVIRONMENT {SimpleHighway, CityBlock}
  	Environment run = CityBlock;

  	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
  	
  	// set camera position and angle
  	viewer->initCameraParameters();
  	// distance away in meters
  	int distance = 16;
  	// CAN CHANGE ARGUMENT HERE TO SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
  	CameraAngle setAngle = FPS;
  	
  	switch(setAngle)
  	{
  		case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
  		case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
  		case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
  		case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  	}
  	if(setAngle!=FPS)
  		viewer->addCoordinateSystem (1.0);
  	

  	// Create point cloud processor
  	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
  	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

  	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  	auto streamIterator = stream.begin();

  	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

  	if(run==SimpleHighway)
  	{
  		simpleHighway(viewer, pointProcessor);
  	}

  	while (!viewer->wasStopped ())
  	{
  		if(run==CityBlock)
  		{
  			// Clear viewer
  			viewer->removeAllPointClouds();
  	  		viewer->removeAllShapes();
	
  	  		// Load pcd and run obstacle detection process
  	  		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
  			cityBlock(viewer, pointProcessorI, inputCloudI);
  	  		
  	  		streamIterator++;
  	  		if(streamIterator == stream.end())
  	  			streamIterator = stream.begin();
		}
    	viewer->spinOnce ();
  	}
  	
}