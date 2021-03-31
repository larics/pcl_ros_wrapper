#include <pcl_ros_wrapper/common/surface.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/make_shared.hpp>

float pcl_ros_wrapper::common::get_cloud_volume(const PointCloudT::Ptr& input)
{
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(input);
  feature_extractor.compute();

  pcl::PointXYZ   minPt;
  pcl::PointXYZ   maxPt;
  pcl::PointXYZ   position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  feature_extractor.getOBB(minPt, maxPt, position_OBB, rotational_matrix_OBB);

  return abs(maxPt.z - minPt.z) * abs(maxPt.x - minPt.x) * abs(maxPt.y - minPt.y);
}

pcl_ros_wrapper::PointCloudT::Ptr pcl_ros_wrapper::common::convex_hull(
  const PointCloudT::Ptr& cloud)
{
  auto                           hull_points = boost::make_shared<PointCloudT>();
  pcl::ConvexHull<pcl::PointXYZ> conv_hull;
  conv_hull.setInputCloud(cloud);
  conv_hull.reconstruct(*hull_points);
  return hull_points;
}

pcl_ros_wrapper::common::concave_hull_result pcl_ros_wrapper::common::concave_hull(
  const PointCloudT::Ptr& cloud,
  const float             alpha)
{
  auto                            hull_points  = boost::make_shared<PointCloudT>();
  auto                            hull_indices = boost::make_shared<pcl::PointIndices>();
  pcl::ConcaveHull<pcl::PointXYZ> conv_hull;
  conv_hull.setInputCloud(cloud);
  conv_hull.setAlpha(alpha);
  conv_hull.setKeepInformation(true);
  conv_hull.reconstruct(*hull_points);
  conv_hull.getHullPointIndices(*hull_indices);
  return concave_hull_result{ hull_points, hull_indices };
}

pcl_ros_wrapper::PointCloudNormalsT::Ptr pcl_ros_wrapper::common::do_normal_estimation(
  const PointCloudT::Ptr& cloud,
  float                   radius,
  bool                    visualize)
{
  auto cloud_normals = boost::make_shared<PointCloudNormalsT>();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setRadiusSearch(radius);
  ne.compute(*cloud_normals);

  if (visualize) {
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 8);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            10);
    viewer.spin();
  }
  return cloud_normals;
}