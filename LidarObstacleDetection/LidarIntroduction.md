# Introduction to Lidar

### Let's dive into lidar

To do that, let's first test running our enviroment, we will be working out of the `src/environment.cpp file`. If you cloned the repo and did all the proper installation for PCL, test the environment by compiling it. 

## Compiling 

Create a new from the root of the repo named **build**, `mkdir build`

Then go into the build directory, `cd build`

Run cmake pointing to the CMakeLists.txt in the root, `cmake ..`

If everything went well you should see something like

```javascript
-- Configuring done
-- Generating done
-- Build files have been written to: /your directory/simple_highway/build
```

If you got **errors** then most likely PCL is not set up properly, see the Installation page from README.md for help on linux or try googling what the specific error message is, and how to resolove it.

If cmake was successful, then from inside build run make, `make`

If make built target environment 100%, it will have generated an executable called environment, this is defined from the CMakeLists.txt.

## Running the Environment

Once we have built an executable file, we can lunch it by doing `./environment`

Now we should see a window poping up that looks like this.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/environment.png" width="700" height="400" />

Here we have a simple highway simulator environment with our ego car in green (thats our car), in the center lane, and other traffic cars in blue. Everything is rendered using PCL with simple boxes, lines and colors. 

We can move around our environment with the mouse. Try holding the left mouse button to oribit around the scene. We can also pan around the scene by holding the middle mouse button and moving, to zoom use the middle scroll mouse botton or the right mouse button and moving.

Congratulation! Our environement is now running and we are ready to start looking at what happens when we introduce lidar!

The first thing we are going to do is create a lidar object, this is defined by the `src/sensors/lidar.h` header file and is included at the top of `environment.cpp`. In `environment.cpp` insdie the function `simpleHighway` functions which takes a reference argument for pcl viewer, we can create a new lidar for our car by adding the code `Lidar* lidar = new Lidar(cars,0);`. This will create a new lidar object allocated on the heap since we our using the `new` key word, notice that also our lidar is a pointer object `Lidar*`, so to call its functions we do `->`. The arguments that our lidar takes in our necessary for it to do ray casting. The rays need to know whats defined as obstacles in our scene, which in our case is other cars and the ground plane. The `0` argument indicates that our ground slope is at 0 degress, flat slope. 

To go further with our newly created lidar object, lets check out `src/sensors/lidar.h` to see how everything is defined. we can see the ray object being defined in the lidar header file, lidar will use these rays to sense its sourrounding by doing ray casting. The `scan()` function from the lidar struct will be what's doing the ray casting. Also if you are wondering why lidar is a struct instead of a class, in C++ the two types are actually very similar with the only difference being that by defualt everything is public in a struct and everything is private in a class. Its also in C that structs are very different from classes since they don't contain functions. Now after that C++ aside, let's call the lidar scan function and see how lidar rays look. Back in our environment file right after where we created our lidar call the scan function and render its rays, the code for this should look like this.

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderRays(viewer,lidar->position, inputCloud);
```

The lidar scan function produces a pcl PointCloud object, the object uses templates because we can have many different types of point clouds, some that are 3D, some that are 2D, some that include color and intensity. Here we are working with plain 3D point clouds so we are using PointXYZ. The Ptr type indicates that are object is actually a pointer, its a 32 bit integar that contains the memory address of our point cloud object. Many functions in pcl use point cloud pointers as arguments so it's convient to return it in this form. The renderRays function is defined in `src/render`, it contains functions that allow us to render points, and shapes to the pcl viewer. We will be using it to render our lidar rays, rendered as line segments in the viewer. The arguments for the renderRays function is viewer, which gets passed in by reference so any changes to it in the function directly affect it outside the function scope, the lidar position also gets passed in, and the point cloud that our scan function generated. The result should look like when complied and ran.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/rays.png" width="700" height="400" />

We can orbit and move around the scene and see the different angles that the rays are being cast. Now with this lidar we are not going to be able to do much, the resoultion is not high enough and we can see from the scene that only one of our rays right now is even hitting a car. So The next task for you will be to increase your lidar's resoultion so we can clearly see the other cars around us. To do this follow the instructions from the TODO statements in `lidar.h`. The changes include increasing the minimum distance, we dont want to include points that are hitting the roof of our car, as well increasing both the horizontal and vertical angle resolution, and finally adding noise. The noise we will be adding is actually quite high, but it will yield more interesting point data in our case. Also feel completely free to experiment and play around with these lidar hyper parameters!  

Now that we can see what are lidar rays look like what about the actual point cloud data that we will be using and processing? We can also view the point cloud data using the renderPointCloud function in render. We can also choice to turn off the rendering for the highway scene so we can see what the pcd looks like by it's self.

```
// TODO:: Create lidar sensor 
Lidar* lidar = new Lidar(cars,0);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
renderPointCloud(viewer, inputCloud,"inputCloud");
```
<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcd1.png" width="700" height="400" />

The result above is without noise and lidar minDistance set to zero, with a high lidar minDistance we can remove the points above that are hitting the roof of our car, since these won't help us detect other cars. Also some noise varience helps to create more interesting looking pcds, as well we want our point processing functions to be robust on data containing noise.

Awesome, we have created a lidar sensor in our scene and have generated high resoultion point cloud data with it. The data that we generated is sufficent to detecting the sourrounding cars around us. Next we will look at creating a point processor object to detect where the traffic cars are at.

Check out the next section, Point Cloud Segmentation.

https://github.com/awbrown90/SensorFusionHighway/blob/master/LidarObstacleDetection/PointCloudSegmentation.md
