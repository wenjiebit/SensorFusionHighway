# Processing Streaming PCD

In the previous section we were able to process obstacle detection on a single real pcd file, now we are going to be using that same processing pipeline on multiple pcd files. To do this lets slightly modify our previous `cityBlock` function from `environment.cpp` to support some additional arguments. We will be passing in the point processor to the `cityBlock` function, this is because we don't want to have to recreate every frame. Also the point cloud input will vary from frame to frame, so input point cloud will now also become a variable. Our function header should now look like this, and we no longer create the point processor or load a point cloud from inside the function.

```
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
```

Notice that in the function header we can make `inputCloud` a constant reference by doing `const` and `&` at the end of the variable definition. We don't have to do this but we are not actually changing the inputCloud at all, just using it as an input for our point processor function. The benefit of using a constant reference is better memory efficiency, since we don't have to write to that variable's memory, just read from it, so it's a slight performance increase. If we do make this a const reference though, we need to make sure not modify or reassign inputCloud, or else we will get a compile error.

So now instead of creating our point processor, and loading pcl files from inside `cityBlock` we will do this in side the `main` function in `environment.cpp` right after where we set up the pcl viewer camera position.

```
ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
auto streamIterator = stream.begin();
pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
```

In the code above, we are making use of a new method from point processor called, `streamPcd`. We tell `streamPcd` a folder directory that contains all the sequentially ordered pcd files we want to process, and it return a chronologically ordered vector of all those file names, called `stream`. We can then go through the `stream` vector in a couple of ways, one option is to use an iterator. The `auto` key defintion can be a nice way to define that itrator which basically it knows by cast typing that `streamIterator` is of type `std::vector<boost::filesystem::path>::iterator` and we avoid having to write all that defintion out, replacing it with just `auto`. At the end of the code block we also set up a variable for our input point cloud, where inside are stream viewer we will be overwriting it each time with a new loaded pcd from `stream`. 

The final thing to look at here is the pcl viewer run cycle which is down at the bottom of `envrionment.cpp`. while the pcl viewer hasn't stopped, we want to process a new frame, do obstacle detection on it, and then view the results. Let's see what this pcl viewer run cycle method looks like.

```
while (!viewer->wasStopped ())
{

  // Clear viewer
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();

  // Load pcd and run obstacle detection process
  inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
  cityBlock(viewer, pointProcessorI, inputCloudI);
    
  streamIterator++;
  if(streamIterator == stream.end())
    streamIterator = stream.begin();

  viewer->spinOnce ();
}
```

The first thing we do in the method is clear any previous rendered point clouds or shapes. Next we load up our point cloud using our point processor and stream iterator. Then we can call our `cityBlock` function, and update the iterator. If the iterator hits the end of the vector we simply set it back to the beginning and that's it. The `viewer->spinOnce()` call controls the frame rate, by default it waits 1 time step, so run as fast as possible. Depending on how timing efficent the obstacle detection functions were set up, the viewer's frame rate will depend on that. If you want to check out the input pcd data at the fastest rate then run the code above and only run a single `renderPointCloud` on the input cloud inside `cityBlock`. Let's check out the results of the streaming pcd viewer below.

## Playing back the pcd files 

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcdStream.gif" width="700" height="400" />

## Playing back files with obstacle detection

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/pcdStreamDetection.gif" width="700" height="400" />

### Conclusions

Congratulations! You have just made it through the entire lidar obstacle detection process. You are able to stream back multiple pcd files and perform filtering, segmentation, clustering, and bounding box detection. Now that you are able to detect in single frames, you can make your pipeline even more robust by tracking detections over the history of frames. You can create associations between detections in frames and use that to track objects. One way to create associations between two different frames is by how close in proximity two detections are to each other and how similar they look. There is also a lot more filtering procedures that you can explore, such as maybe a detection has to be seen in some number of consecutive frames before its considered. You could also filter based on bounding boxes, their volume and shapes. By deploying traccking methods and associations you could even try to dynamically build the shapes of obstacles. Examples of this might be, maybe you see the back side of a long truck, the lidar only first sees the back of the truck. Then later you drive past the truck. leting the lidar see the trucks side. There are many ways to keep exploring and making the detection process more robust. Also if you are up for an additional challenge check out `"src/sensors/data/pcd/data_2"` to see how well you can detect/track a bicyclist riding infront of the car, along with detecting/tracking the other surrounding obstacles.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/challengeSet.gif" width="700" height="400" />

See you in the next module where we will be checking out a new sensor, radar!

// link here (In Development)