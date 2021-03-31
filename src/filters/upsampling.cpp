#include <pcl_ros_wrapper/filters/upsampling.hpp>

pcl_ros_wrapper::PointCloudT::Ptr pcl_ros_wrapper::filters::do_upsampling(
  PointCloudT::Ptr& cloud,
  upsampling_params params)
{
  auto output = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointNormal> mls_points;
  auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setComputeNormals(params.compute_normals);
  mls.setPolynomialOrder(params.polynomial_order);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(params.search_radius);
  mls.setUpsamplingMethod(params.method);

  // SAMPLE_LOCAL_PLANE
  mls.setUpsamplingRadius(params.upsampling_radius);
  mls.setUpsamplingStepSize(params.upsampling_stepsize);
  // RANDOM_UNIFORM_DENSITY
  // num of points in search radius
  mls.setPointDensity(params.upsampling_desired_num_points_in_search_radius);
  // VOXEL_GRID_DILATION
  mls.setDilationVoxelSize(params.upsampling_dilation_voxelsize);
  mls.setDilationIterations(params.upsampling_dilation_iterations);

  mls.process(mls_points);

  // for (int i = 0; i < mls_points.points.size(); ++i) {
  for (const auto& point : mls_points.points) {
    output->points.emplace_back(point.x, point.y, point.z);
  }

  return output;
}
