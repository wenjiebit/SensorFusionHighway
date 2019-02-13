/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include "render.h"

pcl::visualization::PCLVisualizer::Ptr simpleHighway ()
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  // render environment
  renderHighway(viewer);

  Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  egoCar.render(viewer);

  Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  car1.render(viewer);

  Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
  car2.render(viewer);

  Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  car3.render(viewer);
  
  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;
  viewer->setCameraPosition(-distance, distance, distance, 0, 0, 0);

  return (viewer);
}


int main (int argc, char** argv)
{


  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = simpleHighway();
  

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}