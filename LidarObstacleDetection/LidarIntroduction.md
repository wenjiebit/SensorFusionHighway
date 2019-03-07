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
The first thing you are going to do is create a lidar object. The lidar class is defined by the `src/sensors/lidar.h` header file which is included at the top of `environment.cpp`. Also in `environment.cpp` is the `simpleHighway` function, which takes a reference argument for the PCL visualizer `viewer` that was discussed previously. You can create a new lidar for your car in the `simpleHighway` function by adding the code:
```
Lidar* lidar = new Lidar(cars, 0);
``` 
Notice that also your lidar is a pointer object `Lidar*`, so to call the object methods, you should use the arrow operator: `->`. Also note that the code will create a new lidar object allocated on the heap since you are using the `new` key word.  Allocating memory for the lidar on the heap allows for more memory to represent the object; stack memory allocation only provides about 2 MB, while on the heap it can be closer to GBs. The `Lidar` object will be storing point clouds, so the additional memory will be useful in this case. Finally, note that although more memory is available on the heap, there is a trade off in speed. It takes more time to locate objects on the heap, while on the stack it's extremely fast to locate objects.

The arguments that that the `Lidar` constructor accepts are necessary for it to do ray casting, since the rays need to know what objects are defined as obstacles in the scene. In this case the obstacles are other cars and the ground plane. The `0` argument indicates that the ground slope is at 0 degress - a flat slope. 


## Using the Lidar Object

To go further with your newly created `Lidar` object, check out `src/sensors/lidar.h` to see how everything is defined. In this header file, you can see the ray object being defined. Lidar will use these rays to sense its sourrounding by doing ray casting. The `scan()` function from the lidar struct will be what is doing the ray casting. If you are wondering why lidar is a struct instead of a class, in C++ the two types are actually very similar with the only difference being that attributes and methods are public by defualt, whereas everything is private by default in a class. If you are coming from a C background, you'll recall that in C code structs are very different from classes since they don't contain functions. 

Now let's call the lidar scan function and see how lidar rays look. Back in your environment file, right after the call to the `Lidar` constructor, you can use the scan function and then render the lidar rays. The code for this should look like this:

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderRays(viewer,lidar->position, inputCloud);
```

The lidar scan function produces a pcl PointCloud object with `pcl::PointXYZ` points. The object uses a template because there are many different types of point clouds: some that are 3D, some that are 2D, some that include color and intensity. Here you are working with plain 3D point clouds so `PointXYZ` is used. However, later in the course you will have points with an intensity component as well, using the same code. 

The Ptr type indicates that are object is actually a pointer - a 32 bit integer that contains the memory address of your point cloud object. Many functions in pcl use point cloud pointers as arguments, so it's convient to return the `inputCloud` in this form. The renderRays function is defined in `src/render`. It contains functions that allow us to render points and shapes to the pcl viewer. You will be using it to render your lidar rays as line segments in the viewer. The arguments for the `renderRays` function is viewer, which gets passed in by reference. This means that any changes to the viewer in the body of the `renderRays` function directly affect the viewer outside the function scope. The lidar position also gets passed in, as well as the point cloud that your scan function generated. The result should look like the following when complied and run:

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/rays.png" width="700" height="400" />

## Adjusting Lidar Parameters

You can orbit and move around the scene to see the different rays that are being cast. The current lidar settings will limit what you can do, however. The resoultion is low, and as you can see from the scene, only one of the rays is touching a car. So The next task for you will be to increase your lidar's resoultion so you can clearly see the other cars around. To do this, follow the instructions from the TODO statements in `lidar.h`. The changes include increasing the minimum distance so you don't include contact points from the roof of your car, increasing both the horizontal and vertical angle resolution, and finally, adding noise. The noise you will be adding is actually quite high, but it will yield more interesting and realistic point data in the scene. Also feel completely free to experiment and play around with these lidar hyper parameters!

## Examining the Point Cloud

Now that you can see what are lidar rays look like, what about the actual point cloud data that you will be using and processing? You can also view the point cloud data using the `renderPointCloud` function in `render`. You can also choose to turn off the rendering for the highway scene so you can see what the point cloud looks like by it's self:

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderPointCloud(viewer, inputCloud,"inputCloud");
```
<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcd1.png" width="700" height="400" />

The result above is without noise and lidar minDistance set to zero. Wwith a high lidar `minDistance`, you can remove the points above that are hitting the roof of your car, since these won't help you detect other cars. Also some noise varience helps to create more interesting looking point clouds. Additionally, adding noise will help you to develop more robust point processing functions.

## Outro

Awesome! You have created a lidar sensor in your scene and have generated a high resoultion point cloud data with it. The data that you have generated is sufficent to detect the sourrounding cars. Next, you will look at creating a point processor object to detect which parts of the cloud represent the cars in traffic.

Check out the next section, Point Cloud Segmentation.

https://github.com/awbrown90/SensorFusionHighway/blob/master/LidarObstacleDetection/PointCloudSegmentation.md
