#include <pcl_ros_wrapper/filters/voxel.hpp>

using namespace pcl_ros_wrapper;

int main()
{
  // Default usage
  const auto cloud = boost::make_shared<PointCloudT>();
  cloud->points.emplace_back(-100, -100, -100);
  cloud->points.emplace_back(-100.06, -100.06, -100.06);
  cloud->points.emplace_back(1, 1, 1);
  cloud->points.emplace_back(1.05, 1.05, 1.05);

  std::cout << "[VOXEL GRID]\nBefore filtering:\n";
  for (const auto& point : cloud->points) { std::cout << point << "\n"; }

  float leaf_x = 0.1;
  float leaf_y = 0.1;
  float leaf_z = 0.1;

  filters::do_voxel_filtering(boost::make_shared<PointCloudT>(), leaf_x, leaf_y, leaf_z);
  filters::do_voxel_filtering(PointCloudT{}, leaf_x, leaf_y, leaf_z);
  auto res = filters::do_voxel_filtering(cloud, leaf_x, leaf_y, leaf_z);

  std::cout << "\nAfter Filtering:\n";
  for (const auto& point : res->points) { std::cout << point << "\n"; }

  // Use it with some other type
  using PointCloudOtherT  = const pcl::PointCloud<pcl::PointXYZINormal>;
  const auto cloud_custom = boost::make_shared<PointCloudOtherT>();
  filters::do_voxel_filtering(
    boost::make_shared<PointCloudOtherT>(), leaf_x, leaf_y, leaf_z);
  filters::do_voxel_filtering(PointCloudOtherT{}, leaf_x, leaf_y, leaf_z);
  auto res_custom = filters::do_voxel_filtering(cloud_custom, leaf_x, leaf_y, leaf_z);

  return 0;
}