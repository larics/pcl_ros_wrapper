#ifndef SAC_SEGMENTATION_HPP
#define SAC_SEGMENTATION_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <boost/make_shared.hpp>

#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/common/type_traits.hpp>
#include <pcl_ros_wrapper/filters/crop_box.hpp>

namespace pcl_ros_wrapper {
namespace segmentation {

  /**
   * @brief Information obtained after plane detection using sample consensus methods.
   *
   */
  struct plane_info
  {
    pcl::PointIndices::Ptr      inliers;
    pcl::ModelCoefficients::Ptr coefficients;
  };

  /**
   * @brief Parameters used for plane detection methods.
   *
   */
  struct plane_detection_params
  {
    int    ransac_iterations;
    double distance_threshold;
  };

  /**
   * @brief Estimate plane information from the given point cloud.
   *
   * @tparam T Type of the given point cloud.
   * @param input Given point cloud.
   * @param params Plane detection parameters.
   * @return Returns detected plane information.
   */
  template<typename T>
  plane_info do_plane_detection(const T& input, const plane_detection_params& params)
  {
    using CloudType = RemoveConst<element_type_t<T>>;
    using PointType = typename CloudType::value_type;

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(params.ransac_iterations);
    seg.setDistanceThreshold(params.distance_threshold);

    if constexpr (std::is_same<T, element_type_t<T>>::value) {
      seg.setInputCloud(boost::make_shared<element_type_t<T>>(input));
    } else {
      seg.setInputCloud(input);
    }

    auto inliers      = boost::make_shared<pcl::PointIndices>();
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    seg.segment(*inliers, *coefficients);
    return plane_info{ inliers, coefficients };
  }

  /**
   * @brief Crop the largest ground plane from the given input cloud.
   *
   * @param input Given input cloud.
   * @param params Plane detection parameters.
   * @param ground_depth Depth of the cropped ground cloud.
   * @param verbose Output shown if true.
   */
  void crop_ground_plane(PointCloudT::Ptr&             input,
                         const plane_detection_params& params,
                         double                        ground_depth = 1.5,
                         bool                          verbose      = false);


  /**
   * @brief Crop the largest ground plane from the given input cloud. Crop only detected plane points.
   *
   * @param input Given input cloud.
   * @param params Plane detection parameters.
   * @param ground_depth Depth of the cropped ground cloud.
   * @param verbose Output shown if true.
   */
  void crop_ground_plane_smart(PointCloudT::Ptr&             input,
                         const plane_detection_params& params,
                         bool                          verbose      = false);

  /**
   * @brief Iteratively detect and remove ground planes from the given point cloud.
   *
   * @param input Given point cloud.
   * @param params Plane detection parameters.
   * @param max_iters_plane_removal Maximum iteration for plane removal.
   * @param verbose Output shown if true.
   */
  void iterative_ground_plane_filter(PointCloudT::Ptr&             input,
                                     const plane_detection_params& params,
                                     int  max_iters_plane_removal,
                                     bool verbose = false);

  /**
   * @brief Parameters used for pointcloud clustering.
   *
   */
  struct cluster_params
  {
    int    min_cluster_size;
    int    max_cluster_size;
    double cluster_tolerance;
  };

  /**
   * @brief Perform distance-based (euclidean) clustering on the given input cloud.
   *
   * @param input Given point cloud input.
   * @param params Clustering parameters.
   * @param verbose Output shown if true.
   * @return Vector of pointers to pointcloud clusters.
   */
  std::vector<PointCloudT::Ptr> do_euclidean_clustering(const PointCloudT::Ptr& input,
                                                        const cluster_params&   params,
                                                        bool verbose = false);

  /**
   * @brief Project point cloud to a plane.
   *
   * @param cloud Given point cloud.
   * @param coefficients Given plane coefficients.
   * @return A Pointer to the projected point cloud.
   */
  PointCloudT::Ptr project_to_plane(const PointCloudT::Ptr&            cloud,
                                    const pcl::ModelCoefficients::Ptr& coefficients);
}// namespace segmentation
}// namespace pcl_ros_wrapper

#endif /* SAC_SEGMENTATION_HPP */
