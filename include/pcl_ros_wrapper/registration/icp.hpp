#ifndef ICP_WRAPPER_HPP
#define ICP_WRAPPER_HPP

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/console/time.h>

#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/registration/warp_point_3d_yaw.hpp>
#include <pcl_ros_wrapper/registration/warp_point_2d_yaw.hpp>
#include <pcl_ros_wrapper/registration/warp_point_2d.hpp>
#include <pcl_ros_wrapper/registration/warp_point_yaw.hpp>

namespace pcl_ros_wrapper {
namespace registration {
  using TransformationEstimationLM =
    pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>;
  using Warp3D    = pcl::registration::WarpPoint3DYaw<pcl::PointXYZ, pcl::PointXYZ>;
  using Warp2DYaw = pcl::registration::WarpPoint2DYaw<pcl::PointXYZ, pcl::PointXYZ>;
  using Warp2D    = pcl::registration::WarpPoint2D<pcl::PointXYZ, pcl::PointXYZ>;
  using WarpYaw   = pcl::registration::WarpPointYaw<pcl::PointXYZ, pcl::PointXYZ>;

  /**
   * @brief ICP parameter structure
   *
   */
  struct icp_params
  {
    int    maximum_iterations;
    double ransac_outlier_rejection_threshold;
    double max_correspondence_distance;
  };

  struct icp_info
  {
    PointCloudT::Ptr aligned;
    bool             has_converged;
    Eigen::Matrix4f  final_transformation;
    double           fitness_score;
    double           yaw_angle = 0;
  };

  /**
   * @brief Align target pointcloud to the source pointcloud by transforming along x-y
   * axis and around yaw.
   *
   * @param source Source pointcloud.
   * @param target Target pointcloud.
   * @param params ICP parameters.
   * @param verbose Verbose text output.
   * @param guess Initial guess.
   * @return icp_info
   */
  icp_info do_xy_yaw_icp(const PointCloudT::ConstPtr& source,
                         const PointCloudT::ConstPtr& target,
                         const icp_params&            params,
                         bool                         verbose = false,
                         const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity());

  /**
   * @brief Align target pointcloud to the source pointcloud by transforming along x-y-z
   * axis and around yaw.
   *
   * @param source Source pointcloud.
   * @param target Target pointcloud.
   * @param params ICP parameters.
   * @param verbose Verbose text output.
   * @param guess Initial guess.
   * @return icp_info
   */
  icp_info do_xyz_yaw_icp(const PointCloudT::ConstPtr& source,
                          const PointCloudT::ConstPtr& target,
                          const icp_params&            params,
                          bool                         verbose = false,
                          const Eigen::Matrix4f& guess = Eigen::Matrix4f::Identity());

  /**
   * @brief Align target pointcloud to the source pointcloud by transforming along x-y
   * axis.
   *
   * @param source Source pointcloud.
   * @param target Target pointcloud.
   * @param params ICP parameters.
   * @param verbose Verbose text output.
   * @param guess Initial guess.
   * @return icp_info
   */
  icp_info do_xy_icp(const PointCloudT::ConstPtr& source,
                     const PointCloudT::ConstPtr& target,
                     const icp_params&            params,
                     bool                         verbose = false,
                     const Eigen::Matrix4f&       guess   = Eigen::Matrix4f::Identity());

  /**
   * @brief Align target pointcloud to the source pointcloud by transforming around yaw.
   *
   * @param source Source pointcloud.
   * @param target Target pointcloud.
   * @param params ICP parameters.
   * @param verbose Verbose text output.
   * @param guess Initial guess.
   * @return icp_info
   */
  icp_info do_yaw_icp(const PointCloudT::ConstPtr& source,
                      const PointCloudT::ConstPtr& target,
                      const icp_params&            params,
                      bool                         verbose = false,
                      const Eigen::Matrix4f&       guess   = Eigen::Matrix4f::Identity());


  /**
   * @brief Align target pointcloud to the source pointcloud by transforming using
   * the transformation estimation object.
   *
   * @param source Source pointcloud.
   * @param target Target pointcloud.
   * @param params ICP parameters.
   * @param estimation Transformation estimation obejct.
   * @param verbose Verbose text output.
   * @param guess Initial guess.
   * @return icp_info
   */
  icp_info do_icp(
    const PointCloudT::ConstPtr& source,
    const PointCloudT::ConstPtr& target,
    const icp_params&            params,
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::TransformationEstimationPtr
                           estimation,
    bool                   verbose = false,
    const Eigen::Matrix4f& guess   = Eigen::Matrix4f::Identity());
}// namespace registration
}// namespace pcl_ros_wrapper

#endif /* ICP_WRAPPER_HPP */