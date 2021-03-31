#ifndef IO_COMMON_HPP
#define IO_COMMON_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros_wrapper/common/types.hpp>

namespace pcl_ros_wrapper {
namespace common {

  /**
   * @brief Get a Pointer to the PCL type point cloud from *.ply file.
   *
   * @param plyPath Path to the ply file.
   * @return A Pointer to the loaded point cloud.
   */
  PointCloudT::Ptr pcl_from_ply(const std::string& plyPath);

  /**
   * @brief Save a given pointcloud as *.pcd file.
   *
   * @param plyPath Path to the desired save point.
   * @param cloud A cloud that will be saved.
   */
  void save_to_pcd(const std::string& plyPath, const PointCloudT::Ptr& cloud);

  /**
   * @brief Load pointcloud either from a *.pcd or from *.ply file or die trying :)
   *
   * @param infile Path to *.pcd or *.plcy file.
   * @return A Pointer to the loaded point cloud;
   */
  PointCloudT::Ptr load_pcl_or_die(const std::string& infile);

  /**
   * @brief Convert a PCL PointCloud type to 'sensor_msgs::PointCloud2'
   *
   * @param cloud A Point Cloud to be converted.
   * @param frame Frame of the converted point cloud.
   * @return A sensor_msgs::PointCloud2 point cloud.
   */
  sensor_msgs::PointCloud2 toPointcloud2(const PointCloudT& cloud,
                                         const std::string& frame = "mavros/world");

  /**
   * @brief Check if a given point is inside the pointcloud.
   *
   * @param cloud Given point cloud pointer.
   * @param point A point to check.
   * @param epsilon_dist A threshold distance.
   * @return True if point is inside the pointcloud, otherwise false.
   */
  bool point_is_in_pointcloud(const PointCloudT::Ptr& cloud,
                              const pcl::PointXYZ&    point,
                              const float             epsilon_dist = 0.1f);

  /**
   * @brief A structure containing a point and its appropriate index withing a point
   * cloud.
   *
   */
  struct point_with_index
  {
    pcl::PointXYZ point;
    int           index;
  };

  /**
   * @brief Closest point in the pointcloud to the given point.
   *
   * @param cloud Given pointcloud.
   * @param point Given point.
   * @return Closest point.
   */
  point_with_index closest_point(const PointCloudT::Ptr& cloud,
                                 const pcl::PointXYZ     point);

}// namespace common
}// namespace pcl_ros_wrapper

#endif /* IO_COMMON_HPP */