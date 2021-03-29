#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

namespace pcl_ros_wrapper {
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
}// namespace pcl_ros_wrapper

#endif /* POINTCLOUD_HPP */