#ifndef UPSAMPLING_FILTER_HPP
#define UPSAMPLING_FILTER_HPP

#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <boost/make_shared.hpp>

#include <pcl_ros_wrapper/common/types.hpp>

namespace pcl_ros_wrapper {
namespace filters {

  using UpsamplingMethod =
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod;

  /**
   * @brief Parameters used for upsampling pointclouds.
   *
   */
  struct upsampling_params
  {
    UpsamplingMethod method;
    bool             compute_normals;
    int              polynomial_order;
    float            search_radius;// used for surface approximation
    float            upsampling_radius;// only for SAMPLE_LOCAL_PLANE
    float            upsampling_stepsize;// only for SAMPLE_LOCAL_PLANE
    float            upsampling_dilation_voxelsize;// only for VOXEL_GRID_DILATION
    int              upsampling_dilation_iterations;// only for VOXEL_GRID_DILATION
    int              upsampling_desired_num_points_in_search_radius;// only for
                                                       // RANDOM_UNIFORM_DENSITY
  };
  
  /**
   * @brief Perform pointcloud upsampling with respect to given upsampling parameters.
   *
   * @param cloud Given point cloud.
   * @param params Upsampling parameters.
   * @return A Pointer to the upsampled cloud.
   */
  PointCloudT::Ptr do_upsampling(PointCloudT::Ptr& cloud, upsampling_params params);

}// namespace filters
}// namespace pcl_ros_wrapper

#endif /* UPSAMPLING_FILTER_HPP */