#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct Box
{
	
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};
#endif