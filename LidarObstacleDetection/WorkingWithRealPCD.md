# Working With Real PCD

We have now gone through the full process of segmenting and clustering obstacles in our simple simulator point cloud. Now let's take a look at some actual point cloud data recorded from the lidar of a self-driving car driving along a city block. 

To first load up one of the pcd files, we will want to create a point processor, similar to the one we created before in the `simpleHighway` function. This time however we will be using the pcl PointXYZI type, the "I" stands for intensity, which will now be an additional feature for each point in the cloud. Let's create a new function that will be in the same layout as the `simpleHighway` function but this time called CityBlock. The arguments for `CityBlock` will be the same as `simpleHighway`, a reference to the pcl viewer. Inside the new `CityBlock` function we will create a new point process this type with to process `PointXYZI` clouds. We will use the point process to load one of the point clouds and then use the `renderPointCloud` function to view it. Don't forget to call `cityBlock` now instead of `simpleHighway` in the `main` function.

```
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  renderPointCloud(viewer,inputCloud,"inputCloud");
}
```

Here is what it looks like, if the color is not specified in the `renderPointCloud` function argument, it will default to using the intensity color coding.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcd2.png" width="700" height="400" />

Looking around the pcd, we can see several cars parked along the sides of the road and a truck approaching to pass our car on the left side. Our goal will be to fit bounding boxes around these cars and the passing truck, so then our system could later use that information in its path planner, trying to avoid any collisions. 

The first thing we might notice when looking at this point cloud is its quite high resoultion and spans pretty far. We want our processor to be able to digest cloud inputs as quickly as possible, so let's filter the cloud down by using a voxel grid and focus on a smaller region centered around our car. To do this we will fill in the point process function `FilterCloud`. The arguments to this function will be our input cloud, voxel grid size, and min/max points representing our region of interest. The function will return the downsampled cloud with only points that were inside the region specified. Let's look at the code for doing the filtering.

```
pcl::VoxelGrid<PointT> vg;
typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
vg.setInputCloud(cloud);
vg.setLeafSize(filterRes, filterRes, filterRes);
vg.filter(*cloudFiltered);

typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

pcl::CropBox<PointT> region(true);
region.setMin(minPoint);
region.setMax(maxPoint);
region.setInputCloud(cloudFiltered);
region.filter(*cloudRegion);
```

In the code we create a pcl voxel grid that consists of cubes with edge lengths defined by the our input argument `filterRes`. The voxel grid will filter the cloud by only leaving a single point per voxel cube, so the larger `filterRes` the lower the resoultion `cloudFiltered` will have. We do the region selection by creating a pcl `CropBox` and give it our `minPoint`, `maxPoint` The input for `CropBox` is the cloudFiltered that we just downsampled from cloud, and the final resulting cloud is `cloudRegion`. 

Let's go ahead and perform the filtering back in `environment.cpp` and take a look at our filtered cloud results.

```
// Experiment with the ? values and find what works best
inputCloud = pointProcessorI->FilterCloud(inputCloud, ? , Eigen::Vector4f (?, ?, ?, 1), Eigen::Vector4f ( ?, ?, ?, 1));
renderPointCloud(viewer,inputCloud,"inputCloud");
```

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/filtered.png" width="700" height="400" />

In the filter cloud we can now see that our point resoultion is much lower than the original, and we cropped everything inside the box points that we specified. Its important to experiment and play around with the filter input hyper parameters. voxel size should be large enough to help speed up the processing but not so large that object definition is completely lost. For picking a good region, try having a new amount of space in front of the car so it could react quickly in time to any obstacles moving towards it. Also for the sides try to cover at least the width of the road. Whats most important is obstacles that we want to detect are inside the region. Also worth noting is in the `environment.cpp` main function we have different camera angles that we can set for the viewer. This will make it easier to set the camera to have a top down overview or a side overview. This can be helpful when trying to find the right values for the region points. One last thing is we it would be helpful to try to remove points that are hitting our roof, you can use a pcl `CropBox` to find the roof point indicies and then feed those indicies to a pcl ExtractIndices object to remove them (similar to what our segmentation algorithm used to extract points). The `renderBox` function can be really helpful as well for figuring out how big boxes will look in the scene.

### Setting Camera Angle

To start out the camera is set to looking 45 degrees in the xy plane. Feel free to change this to one of the other CameraAngle enum values, TopDown, Side, and FPS (First Person Sense)

```
// Feel free to change the setAngle value to one of the other CameraAngle options
CameraAngle setAngle = XY;
  
switch(setAngle)
{
	case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
	case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
	case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
	case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
}
if(setAngle!=FPS)
	viewer->addCoordinateSystem (1.0);
```

### Steps for Obstacle Detection

#### References the functions discussed in the last previous sections.

Now that are pcd is filtered we are ready to deploy the same segmentation and clustering technqiues that we applied before now using the new intensity point processor `pointProcessorI`.

1. Segment the filtered cloud into two parts, road and obstacles.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/seg2.png" width="700" height="400" />

Here we have are filtered point cloud segmented (road in green), (obstacles in red), with points only in the region of interest. We are also displaying a purple box that showed the space where our car's roof points were contained, we removed those points.

2. Cluster the obstacle cloud.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/cluster2.png" width="700" height="400" />

Next we cluster the obstacle cloud based on the proximity of neighboring points, showing the clusters in cycled colors of reg, yellow, and blue. In the image we see that the oncoming truck is broken up into two colors, front and back showing its actaully being identified as two seperate entities. This illustrates the challenges with clustering based on proximity, the gap between the front of the truck and the back of the truck is large enough so that they look seperate. We might think to fix this by increasing our distance tolerance, but we can also see that the truck is getting really close to one of the side parked cars. Increasing the distance tolerance would run the risk of the truck and parked car being grouped together.

3. Find bounding boxes for the clusters

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/box2.png" width="700" height="400" />

Finally we place bounding boxes around our individual clusters. Since all the detectable vehicles in this scene are along the same axis as our car, the simple already set up bounding box function in point processor should yield good results.

Awesome, congrats for making it through the point process pipeline on a real pcd file! Once you are happy with your results from a single frame, let's look at processing a stream of frames.

Check out the final section on lidar obstacle detection, Processing Streaming PCD.

https://github.com/awbrown90/SensorFusionHighway/blob/master/LidarObstacleDetection/ProcessingStreamingPCD.md









