# PCL ROS Wrapper

A wrapper around some of useful Point Cloud Library (PCL) components.  
Useful in combination with ROS messages such as ```sensor_msgs/PointCloud2```.

## Dependencies

Depends, of course, on PCL. Install as follows:
```bash
sudo apt install libpcl-dev
```

## Description

A brief description of wrapped components.

### Registration

* **[icp.hpp](include/pcl_ros_wrapper/registration/icp.hpp)** - Provides useful function for aligning point clouds using the Iterative Closest Point Method (ICP)

### Filters

* **[crop_box.hpp](include/pcl_ros_wrapper/filters/crop_box.hpp)** - A box filter wrapper
* **[voxel.hpp](include/pcl_ros_wrapper/filters/voxel.hpp)** - A voxel grid wrapper
* **[outlier.hpp](include/pcl_ros_wrapper/filters/outlier.hpo)** - A statistical outlier filter wrapper

### Segmentation

* **[sample_consensus.hpp](include/pcl_ros_wrapper/segmentation/sample_consensus.hpp)** - Useful wrappers around the Sample Consesus methods (e.g. plane detection)