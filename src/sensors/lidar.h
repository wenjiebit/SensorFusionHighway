#ifndef LIDAR_H
#define LIDAR_H
#include "../render.h"

const double pi = 3.1415;

struct Ray
{
	
	Vect3 origin;
	double resolution;
	Vect3 direction;
	Vect3 castPosition;
	double castDistance;

	// parameters:
	// setOrigin: the starting position from where the ray is cast
	// horizontalAngle: the angle of direction the ray travels on the xy plane
	// verticalAngle: the angle of direction between xy plane and ray 
	// 				  for example 0 radians is along xy plane and pi/2 radians is stright up
	// resoultion: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more expensive

	Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
		: origin(setOrigin), resolution(setResolution), direction(resolution*cos(verticalAngle)*cos(horizontalAngle), resolution*cos(verticalAngle)*sin(horizontalAngle),resolution*sin(verticalAngle)),
		  castPosition(origin), castDistance(0)
	{}

	void rayCast(const std::vector<Car>& cars, double minDistance, double maxDistance, pcl::PointCloud<pcl::PointXYZ>& cloud, double slopeAngle)
	{
		// reset ray
		castPosition = origin;
		castDistance = 0;

		bool collision = false;

		while(!collision && castDistance < maxDistance)
		{

			castPosition = castPosition + direction;
			castDistance += resolution;

			// check if there is any collisions with ground slope
			collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

			// check if there is any collisions with cars
			if(!collision && castDistance < maxDistance)
			{
				for(Car car : cars)
				{
					collision |= car.checkCollision(castPosition);
					if(collision)
						break;
				}
			}
		}

		if((castDistance >= minDistance)&&(castDistance<=maxDistance))
			cloud.points.push_back(pcl::PointXYZ(castPosition.x, castPosition.y, castPosition.z));
			
	}

};

struct Lidar
{

	std::vector<Ray> rays;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	std::vector<Car> cars;
	Vect3 position;
	double groundSlope;
	double minDistance;
	double maxDistance;
	double resoultion;

	Lidar(std::vector<Car> setCars, double setGroundSlope)
		: position(0,0,2.6)
	{
		minDistance = 0;
		maxDistance = 50;
		resoultion = 0.2;
		cars = setCars;
		groundSlope = setGroundSlope;

		int numLayers = 8;
		// the steepest vertical angle
		double steepestAngle =  30.0*(-pi/180);
		double angleRange = 26.0*(pi/180);

		double angleIncrement = angleRange/numLayers;

		for(double angleVertical = steepestAngle; angleVertical < steepestAngle+angleRange; angleVertical+=angleIncrement)
		{
			for(double angle = 0; angle <= 2*pi; angle+=pi/64)
			{
				Ray ray(position,angle,angleVertical,resoultion);
				rays.push_back(ray);
			}
		}

	}

	pcl::PointCloud<pcl::PointXYZ> scan()
	{
		cloud.points.clear();
		for(Ray ray : rays)
			ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope);

		cloud.width = cloud.points.size();
		cloud.height = 1; // one dimensional unorganized point cloud dataset
		return cloud;
	}

	void savePcd(pcl::PointCloud<pcl::PointXYZ> cloud, std::string file)
	{
		pcl::io::savePCDFileASCII (file, cloud);
  		std::cerr << "Saved " << cloud.points.size () << " data points to "+file << std::endl;
	}

	pcl::PointCloud<pcl::PointXYZ> loadPcd(std::string file)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  		{
    		PCL_ERROR ("Couldn't read file \n");
  		}
  		std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  		return *cloud;
	}


};

#endif