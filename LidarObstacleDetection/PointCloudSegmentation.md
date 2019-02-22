# Point Cloud Segmentation

### Now that we have point cloud data to work with, we need a way to process it. 

The first thing we are going to do is create a processPointCloud object, this is defined by the `src/processPointClouds.cpp` and `src/processPointClouds.h` files. This class object is going to contain all the methods needed for processing our lidar data, that we will be going over in this module. The process class also has helper methods for loading and saving PCD files.

Right below where we are setting up our lidar we can create our point processor by doing 

```
ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
```

Our pointProcessor is a pointer to our point processor object which is working with PointXYZ data, plain 3D point clouds.

Let's put our point processor to use now. We want to be able to locate obstacles in our scene, this also means that some objects in our scene our not obstacles. So what would be objects that appear in our pcd but are not obstacles? For the most part any free space on the road is not an obstacle, and if the road is flat its fairly stright forward to pick out road points from non-road points. To do this we will use a method called RANSAC https://en.wikipedia.org/wiki/Random_sample_consensus. PCL has built in functions that will allow us to perform RANSAC on our point data that will find a best fitting plane in our data, this plane will be our road. RANSAC is an iterative method, we will specify how many iterations we want to run. The more iterations we run the longer it takes to process, but can also lead to better results when searching for inliers, points associated with the proposed plane. The trick here is defining a good number of iterations to get nice results, but not do too many iterations where it can really slow down our processing time. The next argument that the RANSAC segmentation will use is a distance metric for determing inliers, the larger the distance tolerance the more points the plane can contain. The balance is finding a distance tolerance small enough so that most of the plane points actually belong to the road but not too small so we miss road points that varry a lot because of the shape and misalignment of the terrain.

We will define our SegmentPlane function in `src/processPointClouds.cpp`, there is already a declaration for this function and we just need to fill it in. At the top of the function you will notice a defintion for a template PointT. We will be using this as a variable to represent any type of point cloud, it will come in handy later when we are processing point clouds with intensity values. Take a look at the return type. SegmentPlane will return an `std::pair` with point cloud pointer types. A pair in the C++ std library is similar tuple in other languages like Python. Although there is an actual tuple type in C++ which can hold more than two types, a pair is restricted to just two items. We will use the pair object to hold our segmented results for obstacle points clouds, and the road point clouds. This way we can later visualize both point clouds in the pcl viewer and analyze our results. Back to the SegmentPlane function, the arguments for the function are a point cloud that we want to segment and the two hyper parameters for RANSAC, that we talked about above.

The first thing that we see in our SegmentPlane function place holder is a timer, this can be really useful measuring how long it takes to run the function. If its taking a really long time to process the segmentation then the function is not going to useful running in real-time on a self-driving car. In our case long function calls are going to lead to slow render times when we are looking at multiple data frames. To get started filling in the function we can add this code right after the first start timer variable.

```
// Create two new point clouds, one cloud with obstacles and other with segmented plane
typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
// Obstacle cloud starts out equal to input cloud
*obstCloud = *cloud;

typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

// Create the segmentation object for the planar model and set all the parameters
//
//
pcl::SACSegmentation<PointT> seg;
pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setMaxIterations(maxIterations);
seg.setDistanceThreshold(distanceThreshold);
```
We create two new point cloud pointers, one for obstacles (non-plane points), and one for road (plane points). The new obstacle point cloud will start out equal to the input cloud that we feed it. Inorder to do this notice that we dereference the pointers from both ends and set them equal `*obstCloud = *cloud`, this is because we are dealing with pointers. The pointers are seperate values, different memory addresses but we want the data that they hold equal. The next steps are creating a PCL Segmentation object and setting it up to do plannar RANSAC with our function arguments, maxIterations, and distanceThreshold. The other variables to point out here are, inliers and coefficients. Lets see how we will be using those.

```
// Segment the largest planar component from the input cloud
seg.setInputCloud(obstCloud);
seg.segment (*inliers, *coefficients);
if(inliers->indices.size() == 0)
{
  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
}

// Place plane inliers indices into plane cloud
for(int index : inliers->indices)
  planeCloud->points.push_back(cloud->points[index]);
```

Our inlier variable is a representation for a list of point indicies. In the second code block we feed it, as well as coefficients into the segmentation object. Note that `seg.segment` function has reference type arguments so any modifications done to the variables inside the function modifies the variables values outside the function scope as well. This is a rather convient and memory efficent way to retrieve the modified variables without actually having to return them as a new copied type from the function. Also since both inliers and coefficents are pointer types we dereference them before putting them into the `seg.segment` function, since we don't mean to change the pointer value just the value at the pointer.

After calling segment, the inliers now contain all the indicies from our input cloud that were found to belong to a fitted plane in the data. we can iterate through those indicies and add those indexed points from input cloud into our plane cloud. If however there were no indicies found and inliers is empty, then we failed to find any plane at all. We have access to the coefficients values that make up the plane as well, we are not going to be using them here, but they could be used to render the plane as well if we wanted.

Now that we have our plane point cloud, we need the obstacle cloud, which can be calculated by simply subtracting the plane cloud from the input cloud. The equivalent of this is removing all the inlier indicies from input. One way of using pcl functionality to do this is shown below with an extract object. Also remember obstacle cloud started out being equal to the input cloud. 

```
// Extract the planar inliers from the obstacle cloud
pcl::ExtractIndices<PointT> extract;
extract.setInputCloud (obstCloud);
extract.setIndices (inliers);
extract.setNegative (true);
extract.filter (*obstCloud);
```

The setNegative in extract means to to remove those list of inlier indicies where if it was false then we would be returning those points. So actually we could have created the plane cloud using this same method but by chaning setNegative to false. It's helpful to look at several different ways of achieving the same task, and built in methods that pcl offers as well. In the last line in the above code block, filter takes in a reference to obstCloud's pointed data, and so it gets overriden with the new extraction point cloud. So we use `obstCloud` twice, once to set it as the input to filter from and then at the end to reassign to the new cloud with extracted indicies. 

Now finally we can return the std::pair with our newly created obstacle and plane clouds.

```
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
```

Back in `environment.cpp` we can then call `pointProcessor` function on the input cloud and render the two segmented point clouds.

```
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
```

In the example above we are using 100 iterations, and a distance tolerance of 0.2 meters, you are highly encouraged to play and experiment with these values! Our point cloud is very simple and 100 iterations is way more than needed, you can also monitior how changing the iterations affects the time it takes for the segmentation function to process using the functions predefined timer log. Before rendering the two clouds you will want to remember to turn off rendering from the input cloud done in the previous section, otherwise the clouds will all overlap and it will be hard to distinguish the segmented ones. The renderPointCloud function includes color options (Red,Green,Blue) , by default the cloud is white if no color is specified. Here we will render the obstacle cloud as red, and the plane cloud as green. Check out the results below.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/seg1.png" width="700" height="400" />

Now that we are able to seperate obstacles points from points on the road, let's see how we can further break up our obstacle cloud and identify individual cars using Euclidean clustering.

Also for more information about using pcl to perform segmentation, check out this doc, http://pointclouds.org/documentation/tutorials/extract_indices.php#id1

Check out the next section, Clustering Obstacles.

https://github.com/awbrown90/SensorFusionHighway/blob/master/LidarObstacleDetection/ClusteringObstacles.md