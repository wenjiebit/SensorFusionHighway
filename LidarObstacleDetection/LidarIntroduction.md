# Introduction to Lidar

Before diving into lidar, you will first need to test your enviroment. You will be working out of the `src/environment.cpp file`. If you cloned the repo and did all the proper installation for PCL, test the environment by compiling it. 

## Compiling 

Create a new from the root of the repo named **build**\: `mkdir build`

Then go into the build directory\: `cd build`

Run cmake pointing to the CMakeLists.txt in the root\: `cmake ..`

If everything went well you should see something like

```javascript
-- Configuring done
-- Generating done
-- Build files have been written to: /your directory/simple_highway/build
```

If you got **errors** then most likely PCL is not set up properly. See the Installation page from README.md for help on linux, or try googling the specific error message to see how to resolove it.

If cmake was successful, then from inside build run make\: `make`

If make built target environment 100%, it will have generated an executable called `environment`. This is defined from the CMakeLists.txt.

## Running the Environment

Once you have built an executable file, you can lunch it by doing `./environment`

Now you should see a window poping up that looks like this.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/environment.png" width="700" height="400" />

Here you have a simple highway simulator environment with the ego car in green in the center lane (thats your car), and the other traffic cars in blue. Everything is rendered using PCL with simple boxes, lines, and colors. 

You can move around your environment with the mouse. Try holding the left mouse button to oribit around the scene. You can also pan around the scene by holding the middle mouse button and moving. To zoom, use the middle scroll mouse botton or the right mouse button while moving.

Congratulation! Your environement is now running and you are ready to start looking at what happens when you introduce lidar!

## The PCL Library

In this course, you will be using the open source library [PCL](https://github.com/PointCloudLibrary) to work with point cloud data in C++. PCL is a widely used library that provides a variety of tools for 2D and 3D point cloud processing and visualization.

## Creating a Lidar Object

### The PCL Viewer
The `environment.cpp` file contains the `main` function for your program. In the `main`, you will create a `PCLVisualizer` object called `viewer`\: 
```
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
```
You can think of the viewer as the scene for your point cloud visualization, and the next couple of lines in `main` are used to set the camera properties for the scene using the `initCamera` function.


### Creating the Lidar
The first thing you are going to do is create a lidar object. This is defined by the `src/sensors/lidar.h` header file and is included at the top of `environment.cpp`. In `environment.cpp` inside the function `simpleHighway` function, which takes a reference argument for pcl viewer, you can create a new lidar for your car by adding the code:
```
Lidar* lidar = new Lidar(cars, 0);
``` 
This will create a new lidar object allocated on the heap since you your using the `new` key word, notice that also your lidar is a pointer object `Lidar*`, so to call its functions you do `->`. Allocating on the heap gives us more memory to represent your objects, stack memory allocation only provides about 2 MB, while on the heap it can be closer to GBs. Although you get more memory on the heap, it's trade off is speed, since it takes more time to locate objects on the heap, while on the stack it's extreamly fast to locate objects. 

The arguments that your lidar accepts are necessary for it to do ray casting. The rays need to know whats defined as obstacles in your scene, which in your case is other cars and the ground plane. The `0` argument indicates that your ground slope is at 0 degress, flat slope. 

To go further with your newly created lidar object, lets check out `src/sensors/lidar.h` to see how everything is defined. you can see the ray object being defined in the lidar header file, lidar will use these rays to sense its sourrounding by doing ray casting. The `scan()` function from the lidar struct will be what's doing the ray casting. Also if you are wondering why lidar is a struct instead of a class, in C++ the two types are actually very similar with the only difference being that by defualt everything is public in a struct and everything is private in a class. Its also in C that structs are very different from classes since they don't contain functions. Now after that C++ aside, let's call the lidar scan function and see how lidar rays look. Back in your environment file right after where you created your lidar call the scan function and render its rays, the code for this should look like this.

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderRays(viewer,lidar->position, inputCloud);
```

The lidar scan function produces a pcl PointCloud object, the object uses templates because you can have many different types of point clouds, some that are 3D, some that are 2D, some that include color and intensity. Here you are working with plain 3D point clouds so you are using PointXYZ. The Ptr type indicates that are object is actually a pointer, its a 32 bit integar that contains the memory address of your point cloud object. Many functions in pcl use point cloud pointers as arguments so it's convient to return it in this form. The renderRays function is defined in `src/render`, it contains functions that allow us to render points, and shapes to the pcl viewer. you will be using it to render your lidar rays, rendered as line segments in the viewer. The arguments for the renderRays function is viewer, which gets passed in by reference so any changes to it in the function directly affect it outside the function scope, the lidar position also gets passed in, and the point cloud that your scan function generated. The result should look like when complied and ran.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/rays.png" width="700" height="400" />

We can orbit and move around the scene and see the different angles that the rays are being cast. Now with this lidar you are not going to be able to do much, the resoultion is not high enough and you can see from the scene that only one of your rays right now is even hitting a car. So The next task for you will be to increase your lidar's resoultion so you can clearly see the other cars around us. To do this follow the instructions from the TODO statements in `lidar.h`. The changes include increasing the minimum distance, you dont want to include points that are hitting the roof of your car, as well increasing both the horizontal and vertical angle resolution, and finally adding noise. The noise you will be adding is actually quite high, but it will yield more interesting point data in your case. Also feel completely free to experiment and play around with these lidar hyper parameters!  

Now that you can see what are lidar rays look like what about the actual point cloud data that you will be using and processing? you can also view the point cloud data using the renderPointCloud function in render. you can also choice to turn off the rendering for the highway scene so you can see what the pcd looks like by it's self.

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderPointCloud(viewer, inputCloud,"inputCloud");
```
<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcd1.png" width="700" height="400" />

The result above is without noise and lidar minDistance set to zero, with a high lidar minDistance you can remove the points above that are hitting the roof of your car, since these won't help us detect other cars. Also some noise varience helps to create more interesting looking pcds, as well you want your point processing functions to be robust on data containing noise.

Awesome, you have created a lidar sensor in your scene and have generated high resoultion point cloud data with it. The data that you generated is sufficent to detecting the sourrounding cars around us. Next you will look at creating a point processor object to detect where the traffic cars are at.

Check out the next section, Point Cloud Segmentation.

https://github.com/awbrown90/SensorFusionHighway/blob/master/LidarObstacleDetection/PointCloudSegmentation.md
