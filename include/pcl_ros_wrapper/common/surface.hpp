#ifndef SURFACE_COMMON_HPP
#define SURFACE_COMMON_HPP

#include <pcl_ros_wrapper/common/types.hpp>

namespace pcl_ros_wrapper {
namespace common {

  /**
   * @brief Comput the volume of the point cloud.
   *
   * @param input Given point cloud input.
   * @return Volume of the point cloud.
   */
  float get_cloud_volume(const PointCloudT::Ptr& input);

  /**
   * @brief Create a convex hull from the given point cloud.
   *
   * @param cloud Given point cloud.
   * @return Convex hull of the given point cloud.
   */
  PointCloudT::Ptr convex_hull(const PointCloudT::Ptr& cloud);

  /**
   * @brief This structure represents the result after computing the concave hull.
   *
   */
  struct concave_hull_result
  {
    PointCloudT::Ptr       cloud;
    pcl::PointIndices::Ptr indices;
  };

  /**
   * @brief Create a concave hull from the given point cloud.
   *
   * @param cloud Given point cloud.
   * @param alpha Smaller value results in a detailed hull.
   * @return Result of the concave hull.
   */
  concave_hull_result concave_hull(const PointCloudT::Ptr& cloud,
                                   const float             alpha = 2.0);

  /**
   * @brief Estimate normals of the given point cloud.
   *
   * @param cloud Given point cloud.
   * @param radius Sphere radius for the noraml estimation.
   * @param visualize If true a PCL visualization windows pops up.
   * @return A pointer to the pointcloud with estimated normals.
   */
  PointCloudNormalsT::Ptr do_normal_estimation(const PointCloudT::Ptr& cloud,
                                               float                   radius,
                                               bool                    visualize = false);

}// namespace common
}// namespace pcl_ros_wrapper

#endif /* SURFACE_COMMON_HPP */