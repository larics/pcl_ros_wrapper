# PCL ROS Wrapper

| Doxygen |
|--------------------------------------------------------------------------------------------------------------------------------|
| [![Docs](https://github.com/lmark1/uav_ros_lib/workflows/Docs/badge.svg)](https://github.com/lmark1/uav_ros_lib/actions) |

A wrapper around some of useful Point Cloud Library (PCL) components.  
Useful in combination with ROS messages such as ```sensor_msgs/PointCloud2```.

## Dependencies

Depends, of course, on PCL. Install as follows:
```bash
sudo apt install libpcl-dev
```
## Documentation

Documentation can be found at [lmark1.github.io/pcl_ros_wrapper](https://lmark1.github.io/pcl_ros_wrapper/).
## Description

A brief description of wrapped components.

### Common
* **[general.hpp](include/pcl_ros_wrapper/common/general.hpp)** - Contains useful functions for handling pointclouds (e.g. load, save, convert, etc.)
* **[surface.hpp](include/pcl_ros_wrapper/common/surface.hpp)** - Contains useful functions dealing with surface-based methods (e.g. concave/convex hull, normal estimation, etc.)
### Registration

* **[icp.hpp](include/pcl_ros_wrapper/registration/icp.hpp)** - Provides useful function for aligning point clouds using the Iterative Closest Point Method (ICP)

### Filters

* **[crop_box.hpp](include/pcl_ros_wrapper/filters/crop_box.hpp)** - Wrapper around the CropBox filter
* **[voxel.hpp](include/pcl_ros_wrapper/filters/voxel.hpp)** - Wrapper around the VoxelGrid filter
* **[outlier.hpp](include/pcl_ros_wrapper/filters/outlier.hpo)** - Wrapper around the StatisticalOutlierFilter
* **[upsampling.hpp](include/pcl_ros_wrapper/filters/upsampling.hpp)** - Useful functions for upsampling pointclouds

### Segmentation

* **[sac.hpp](include/pcl_ros_wrapper/segmentation/sac.hpp)** - Useful wrappers around the Sample Consesus methods (e.g. plane detection, clustering, etc.)