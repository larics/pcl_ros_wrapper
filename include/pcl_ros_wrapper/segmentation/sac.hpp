#ifndef SAC_SEGMENTATION_HPP
#define SAC_SEGMENTATION_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <boost/make_shared.hpp>

#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/common/type_traits.hpp>

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

}// namespace segmentation
}// namespace pcl_ros_wrapper

#endif /* SAC_SEGMENTATION_HPP */