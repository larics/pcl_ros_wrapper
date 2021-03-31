#include <pcl_ros_wrapper/common/general.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

pcl_ros_wrapper::PointCloudT::Ptr pcl_ros_wrapper::common::pcl_from_ply(
  const std::string& plyPath)
{
  auto plyCloud = boost::make_shared<PointCloudT>();
  ROS_INFO("pcl_from_ply() - Reading from path: %s", plyPath.c_str());
  if (pcl::io::loadPLYFile(plyPath, *plyCloud) == -1) {
    ROS_FATAL("pcl_from_ply() - unable to load mesh, exiting...");
    throw std::runtime_error("pcl_from_ply() - unable to load wall mesh");
  }
  return plyCloud;
}

void pcl_ros_wrapper::common::save_to_pcd(const std::string&      plyPath,
                                          const PointCloudT::Ptr& cloud)
{
  pcl::io::savePCDFileASCII(plyPath, *cloud);
}

pcl_ros_wrapper::PointCloudT::Ptr pcl_ros_wrapper::common::load_pcl_or_die(
  const std::string& infile)
{
  const auto has_suffix = [](const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size()
           && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
  };

  if (has_suffix(infile, ".ply")) { return pcl_from_ply(infile); }

  if (has_suffix(infile, ".pcd")) {
    auto           pointcloud = boost::make_shared<PointCloudT>();
    pcl::PCDReader reader;
    reader.read(infile, *pointcloud);
    return pointcloud;
  }

  throw std::runtime_error("Unable to read pointcloud: " + infile);
}

sensor_msgs::PointCloud2 pcl_ros_wrapper::common::toPointcloud2(const PointCloudT& cloud,
                                                                const std::string& frame)
{
  sensor_msgs::PointCloud2 outputMessage;
  pcl::toROSMsg(cloud, outputMessage);
  outputMessage.header.stamp    = ros::Time::now();
  outputMessage.header.frame_id = frame;
  return outputMessage;
}

bool pcl_ros_wrapper::common::point_is_in_pointcloud(const PointCloudT::Ptr& cloud,
                                                     const pcl::PointXYZ&    point,
                                                     const float             epsilon_dist)
{
  for (const auto& pt : *cloud) {
    if (pcl::euclideanDistance(pt, point) < epsilon_dist) return true;
  }
  return false;
}

pcl_ros_wrapper::common::point_with_index pcl_ros_wrapper::common::closest_point(
  const PointCloudT::Ptr& cloud,
  const pcl::PointXYZ     point)
{
  point_with_index closest;
  double           min_dist = 1e9;
  int              i        = 0;
  for (const auto& pt : cloud->points) {
    double dist = pcl::euclideanDistance(pt, point);
    ++i;
    if (dist < min_dist) {
      min_dist      = dist;
      closest.point = pt;
      closest.index = i;
    }
  }
  return closest;
}
