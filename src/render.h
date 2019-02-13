/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include <pcl/visualization/pcl_visualizer.h>

struct Color
{

	int r, g, b;

	Color(int setR, int setG, int setB)
		: r(setR), g(setG), b(setB)
	{}
};

struct Vect3
{

	double x, y, z;

	Vect3(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ)
	{}
};

struct Car
{

	// units in meters
  	Vect3 position, dimensions;
  	
  	std::string name;
  	Color color;

  	Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
    	: position(setPosition), dimensions(setDimensions), color(setColor), name(setName)
  	{}

  	void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		// render bottom of car
		viewer->addCube(position.x-dimensions.x/2, position.x+dimensions.x/2, position.z, position.z+dimensions.z*2/3, position.y-dimensions.y/2, position.y+dimensions.y/2, color.r, color.g, color.b, name); 
		// render top of car
		viewer->addCube(position.x-dimensions.x/4, position.x+dimensions.x/4, position.z+dimensions.z*2/3, position.z+dimensions.z, position.y-dimensions.y/2, position.y+dimensions.y/2, color.r, color.g, color.b, name+"Top"); 
	}

};

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);
