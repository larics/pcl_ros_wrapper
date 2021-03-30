#include <pcl_ros_wrapper/segmentation/sac.hpp>

using namespace pcl_ros_wrapper;

int main()
{
  // Default usage
  const auto cloud = boost::make_shared<PointCloudT>();
  cloud->points.emplace_back(-10, 10, 0);
  cloud->points.emplace_back(1, 1, 0);
  cloud->points.emplace_back(-10, -10, 0);
  cloud->points.emplace_back(10, 10, 0);

  std::cout << "[PLANE DETECTION]:\n";
  for (const auto& point : cloud->points) { std::cout << point << "\n"; }

  segmentation::plane_detection_params params{ 100, 0.2 };
  segmentation::do_plane_detection(boost::make_shared<const PointCloudT>(*cloud), params);
  segmentation::do_plane_detection(cloud, params);
  auto res = segmentation::do_plane_detection(*cloud, params);

  std::cout << "\nPlane parameters:\n";
  std::cout << res.coefficients->values.at(0) << ", " << res.coefficients->values.at(1)
            << ", " << res.coefficients->values.at(2) << ", "
            << res.coefficients->values.at(3) << "\n";
  return 0;
}