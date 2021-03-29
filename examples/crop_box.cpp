#include <pcl_ros_wrapper/filters/crop_box.hpp>

using namespace pcl_ros_wrapper;

int main()
{
  // Default usage
  const auto cloud = boost::make_shared<PointCloudT>();
  filters::do_crop_box(
    boost::make_shared<PointCloudT>(), Eigen::Vector4f{}, Eigen::Vector4f{});
  filters::do_crop_box(PointCloudT{}, Eigen::Vector4f{}, Eigen::Vector4f{});
  filters::do_crop_box(cloud, Eigen::Vector4f{}, Eigen::Vector4f{});

  // Use it with some other type
  using PointCloudOtherT  = const pcl::PointCloud<pcl::PointXYZINormal>;
  const auto cloud_custom = boost::make_shared<PointCloudOtherT>();
  filters::do_crop_box(
    boost::make_shared<PointCloudOtherT>(), Eigen::Vector4f{}, Eigen::Vector4f{});
  filters::do_crop_box(PointCloudOtherT{}, Eigen::Vector4f{}, Eigen::Vector4f{});
  filters::do_crop_box(cloud_custom, Eigen::Vector4f{}, Eigen::Vector4f{});

  return 0;
}