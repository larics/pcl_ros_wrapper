#ifndef OUTLIER_REMOVAL_FILTER_HPP
#define OUTLIER_REMOVAL_FILTER_HPP

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/common/type_traits.hpp>

namespace pcl_ros_wrapper {
namespace filters {

  /**
   * @brief Apply a statistical outlier filter on the given point cloud.
   *
   * @tparam T A type of the given point cloud.
   * @param input The give input point cloud.
   * @param meanK Number of nearest neighbours for mean distance estimation.
   * @param multiplier_threshold Standard deviation multiplier for distance threshold
   * calculation
   * @return A Pointer to the filtered Point cloud.
   */
  template<typename T>
  boost::shared_ptr<RemoveConst<element_type_t<T>>> do_outlier_filtering(
    const T& input,
    int      meanK                = 50,
    double   multiplier_threshold = 1.0)
  {
    using CloudType = RemoveConst<element_type_t<T>>;
    using PointType = typename CloudType::value_type;
    auto output     = boost::make_shared<CloudType>();

    pcl::StatisticalOutlierRemoval<PointType> sor;
    if constexpr (std::is_same<T, element_type_t<T>>::value) {
      sor.setInputCloud(boost::make_shared<element_type_t<T>>(input));
    } else {
      sor.setInputCloud(input);
    }
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(multiplier_threshold);
    sor.filter(*output);
    return output;
  }
}// namespace filters
}// namespace pcl_ros_wrapper

#endif /* OUTLIER_REMOVAL_FILTER_HPP */