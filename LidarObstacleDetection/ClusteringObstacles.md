# Clustering Obstacles

We have a way to segment points and recongonize which ones represent obstacles for our car. It would be great to break up and group those points, especially if we want to do multiple object tracking with cars, pedestrians, and bicyclists for instace. To do that we will look at one way to cluster point cloud data which is called euclidean clustering. The idea is we associate groups of points by how close together they are, to do this nearest neighbor search efficently we use a KD tree data structure which on average increases look up time from O(n) to O(log(n)) since the data structure allows us to break up our search space. By grouping points into regions in a KD tree we can avoid calcaulating distance for let's say thousands of points just because we know they are not even considered in a close enough region. Our eculidean clustering algorithm will take in a distance tolerance, any points within that distance will be grouped together. We will also be using a min and max number of points to represent clusters. The idea is if a cluster is really small, its probably just noise and we are not concerned with it. Also a max number of points allows us to better break up very large clusters. If a cluster is very large it might just be because of many other cluster overlapping and a max tolerance can help us better dissolve the object detections.  

The point processor `Clustering` function is located right under the `Segmentation` function. Inside the Clustering function template, we will define two pcl objects, tree (representing our kd tree), and EuclideanClusterExtraction (this will actually use the tree).

```
// Creating the KdTree object for the search method of the extraction
typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
tree->setInputCloud(cloud);

std::vector<pcl::PointIndices> clusterIndices;
pcl::EuclideanClusterExtraction<PointT> ec;
ec.setClusterTolerance(clusterTolerance); 
ec.setMinClusterSize(minSize);
ec.setMaxClusterSize(maxSize);
ec.setSearchMethod(tree);
ec.setInputCloud(cloud);
ec.extract(clusterIndices);
```

We feed the tree the input cloud and set up the values to feed into EuclideanClusterExtraction, this includes arguments from our function with clusterTolerance, minSize, maxSize, and the tree that we just created. When we run extract we store a vector of all the cluster PointIndices, PointIndicies is a grouping of point indicies. The point indicies reference the input cloud points, so now all we have to do is go through the vector of PointIndicies and create new point clouds for each of the individual clusters, feeding in the appropriate input cloud indexed points from clusterIndices. One way of doing that in code is shown below.

```
std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

for(pcl::PointIndices getIndices: clusterIndices)
{
	typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

    for(int index : getIndices.indices)
    	cloudCluster->points.push_back (cloud->points[index]); 

    cloudCluster->width = cloudCluster->points.size ();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);

}
```

We have our vector of point cloud object pointers called clusters. We iterate through clusterIndices getting each PointIndices. We create a new point cloud, which is our new cluster and we iterate through all the point indicies in PointIndices and push those points into the new cluster. We finish up setting the width, height, and density values of the cluster and then push that cluster onto our vector of clusters

Using pcl's built in functions has made the process of doing euclidean clustering quite strightforward, we could always play around with trying to feed in other data structures besides kd trees into cluster extractor to see how results compare. Another common data structure used for containing point clouds is an octrees. Unlike a kd tree which breaks up space into two halves an octree will break space into eight equal parts.

Going back to `environment.cpp` let's see how we can then render our different clusters.

```
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
	
int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
{
  	std::cout << "cluster size ";
  	pointProcessor->numPoints(cluster);
  	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
  	++clusterId;
}
```

In the example above we call the cluster function and then iterate through it and call renderPointCloud on each cluster. The renderPointCloud is expecting each pcl viewer point cloud to have a unique name identifer so we count the clusters with clusterId and extend that on to the cloud's name. To get different colors for each of the clusters we define a list of colors, here simply use red, blue and green. As a bonus we also log how many points each cluster had, this can be a helpful debugging tool later when trying to pick a good min and max point values. In this example the min points for a cluster is set to 3, and the max set to 30, the distance tolerance is also set to 1. Some time and effort will need to be taken to pick good hyperparameters but in alot of cases actually there won't be a perfect set to get great results accross all possible examples. This is because euclidean clustering is a rather simple way to do clustering, more advance methods now of days even look to deep learning and doing segmentation from training on thousands of annotated data sets of point clouds and labled objects.


Below lets look at how our clustring functon did, we see that we succesfully found three clusters each belonging to one of the different cars sourrounding us. Each cluster is colored differently, red, green, and blue.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/clusters1.png" width="700" height="400" />

As a final touch we can add bounding boxes around these clusters to help distinguish them. The bounding box volume could also be thought of as space the car is not allowed to enter, or it would result in a collision. Our point processor already has a bounding box function all set up for us This function is already predefined since its doing a very simple technique for finding defining a box, looking at min,max point values and storing it in a box struct container. To add bounding boxes we can simpley add the following code inside the for loop from the code block above.

```
Box box = pointProcessor->BoundingBox(cluster);
renderBox(viewer,box,clusterId);
```

Here is what the clusters look like with red transparent bounding boxes.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/box1.png" width="700" height="400" />

Some comments about the way bounding boxes are calculated here. For this bounding box function, boxes are always oriented along the x and y axis, this is ok if the cluster that we are looking at has its majority of points orientated along these axis as well, but what if the cluster was a very long rectangular object at a 45 degree angle to the x axis. The result using this bounding box function would be a unnecessarily large box, and would contrain our car's available space to move within. See the diagram below for reference.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/boxexample.png" width="700" height="400" />

In the diagram the bounding box on the right is much more efficent, containing all the points with the minimum requried area. It would be great to be able to do the same in this, taking into account box rotation along xy plane. Rotation along the z axis would yield weird results since our car in the majority of situations is not concerned with the z dimension, or has any control over z movement. The file containing the box struct is located in `src/render/box.h` and contains an aditional struct called BoxQ, this struct has a quaternion member that allows rotations. Also there is an additional renderBox function in `render.cpp` that takes a BoxQ argument and renders the rotation enabled box. There is a great blog post about fitting the smallest possible 3D box around a 3D point cloud here, http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html. However the solution which uses PCA, principle component analysis covered in the post includes z axis rotations as well. A challenge problem to the student then is to find the smallest box but which is oriented flat with the XY plane. 

### Conclusions from Clustering Obstacles

Congratulations, you have now been able to find bounding boxes which each coincide with the different cars on the highway simulator scene. You did this by using a lidar to create a high resolution point cloud, and then segmented that point cloud into two parts, road, and obstacles. Finally you used euclidean clustering to break up the obstacle point cloud into seperate entities, in this case the different cars, and put bounding boxes around them.

You have graduated from the highway simulator, and are now ready to tackle real point cloud data from self-driving cars!

Check out the next section, Working with Real PCD.

// link here to next section 






