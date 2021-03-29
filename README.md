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

**[icp.hpp](include/pcl_ros_wrapper/registration/icp.hpp)** - Provides useful function for aligning point clouds using the Iterative Closest Point Method (ICP)