/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	// units in meters
	double roadLength = 50.0;
	double roadWidth = 12.0;
	double roadHeight = 0.2;

	viewer->addCube(-roadLength/2, roadLength/2, -roadWidth/2, roadWidth/2, -roadHeight, 0, .2, .2, .2, "highwayPavement"); 
	viewer->addLine(pcl::PointXYZ(-roadLength/2,-roadWidth/6, 0.01),pcl::PointXYZ(roadLength/2, -roadWidth/6, 0.01),1,1,0,"line1");
	viewer->addLine(pcl::PointXYZ(-roadLength/2, roadWidth/6, 0.01),pcl::PointXYZ(roadLength/2, roadWidth/6, 0.01),1,1,0,"line2");
}

int countRays = 0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const std::vector<Vect3>& points)
{
	for(Vect3 point : points)
	{
		viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), pcl::PointXYZ(point.x,point.y,point.z),1,0,0,"ray"+std::to_string(countRays));
		countRays++;
	}
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	while(countRays)
	{
		countRays--;
		viewer->removeShape("ray"+std::to_string(countRays));
	}
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const std::vector<Vect3>& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	for(Vect3 point : points)
	{
		pcl::PointXYZ cloudPoint;
      	cloudPoint.x = point.x;
      	cloudPoint.y = point.y;
      	cloudPoint.z = point.z;
      	point_cloud_ptr->points.push_back(cloudPoint);
	}

	viewer->addPointCloud<pcl::PointXYZ> (point_cloud_ptr, "point cloud");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point cloud");
}


	