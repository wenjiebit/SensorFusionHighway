/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	// units in meters
	double roadWidth = 12.0;
	double roadLength = 50.0;
	double roadHeight = 0.2;

	viewer->addCube(-roadLength/2, roadLength/2, -roadHeight, 0, -roadWidth/2, roadWidth/2, .2, .2, .2, "highwayPavement"); 
	viewer->addLine(pcl::PointXYZ(-roadLength/2,0.01,-roadWidth/6),pcl::PointXYZ(roadLength/2,0.01,-roadWidth/6),1,1,0,"line1");
	viewer->addLine(pcl::PointXYZ(-roadLength/2,0.01,roadWidth/6),pcl::PointXYZ(roadLength/2,0.01,roadWidth/6),1,1,0,"line2");
}