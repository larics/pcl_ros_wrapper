#include <pcl_ros_wrapper/filters/crop_box.hpp>

using namespace pcl_ros_wrapper;

int main()
{
  // Default usage
  const auto cloud = boost::make_shared<PointCloudT>();
  cloud->points.emplace_back(100, 100, 100);
  cloud->points.emplace_back(-100, -100, -100);
  cloud->points.emplace_back(1, 1, 1);

  std::cout << "[BOX FILTER]\nBefore filtering:\n";
  for (const auto& point : cloud->points) { std::cout << point << "\n"; }

  Eigen::Vector4f min{ -10, -10, -10, 1 };
  Eigen::Vector4f max{ 10, 10, 10, 1 };

  filters::do_crop_box(boost::make_shared<PointCloudT>(), min, max);
  filters::do_crop_box(PointCloudT{}, min, max);
  auto res = filters::do_crop_box(cloud, min, max);

  std::cout << "\nAfter Filtering:\n";
  for (const auto& point : res->points) { std::cout << point << "\n"; }

  // Use it with some other type
  using PointCloudOtherT  = const pcl::PointCloud<pcl::PointXYZINormal>;
  const auto cloud_custom = boost::make_shared<PointCloudOtherT>();
  filters::do_crop_box(boost::make_shared<PointCloudOtherT>(), min, max);
  filters::do_crop_box(PointCloudOtherT{}, min, max);
  auto res_custom = filters::do_crop_box(cloud_custom, min, max);

  return 0;
}