#ifndef CROP_BOX_FILTER_HPP
#define CROP_BOX_FILTER_HPP

#include <pcl/filters/crop_box.h>
#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/common/type_traits.hpp>

namespace pcl_ros_wrapper {
namespace filters {

  /**
   * @brief Apply the Crop Box filter on the given point cloud.
   *
   * @tparam T Type of the given Point Cloud.
   * @param input A given point cloud.
   * @param minCoords Minimum box coordinates.
   * @param maxCoords Maximum box coordinates.
   * @param negative True - normal crop box, false - inverted crop box
   * @return PointCloudT::Ptr A cropped point cloud pointer
   */
  template<typename T>
  boost::shared_ptr<RemoveConst<element_type_t<T>>> do_crop_box(
    const T&               input,
    const Eigen::Vector4f& minCoords,
    const Eigen::Vector4f& maxCoords,
    bool                   negative = false)
  {
    using CloudType = RemoveConst<element_type_t<T>>;
    using PointType = typename CloudType::value_type;
    auto output     = boost::make_shared<CloudType>();

    pcl::CropBox<PointType> boxFilter;
    // Check if pointer
    if constexpr (std::is_same<T, element_type_t<T>>::value) {
      boxFilter.setInputCloud(boost::make_shared<element_type_t<T>>(input));
    } else {
      boxFilter.setInputCloud(input);
    }
    boxFilter.setMax(maxCoords);
    boxFilter.setMin(minCoords);
    boxFilter.setNegative(negative);
    boxFilter.filter(*output);
    return output;
  }

  /**
   * @brief Apply the Crop Box filter to the given point cloud.
   *
   * @tparam T Type of the given pointcloud;
   * @param input
   * @param minCoords
   * @param maxCoords
   * @param negative
   * @return pcl::IndicesPtr
   */
  template<typename T>
  pcl::IndicesPtr do_crop_box_indices(const T&               input,
                                      const Eigen::Vector4f& minCoords,
                                      const Eigen::Vector4f& maxCoords,
                                      bool                   negative = false)
  {
    pcl::IndicesPtr output = boost::make_shared<std::vector<int>>();
    using CloudType        = RemoveConst<element_type_t<T>>;
    using PointType        = typename CloudType::value_type;
    pcl::CropBox<PointType> boxFilter;
    // Check if pointer
    if constexpr (std::is_same<T, element_type_t<T>>::value) {
      boxFilter.setInputCloud(boost::make_shared<element_type_t<T>>(input));
    } else {
      boxFilter.setInputCloud(input);
    }
    boxFilter.setMax(maxCoords);
    boxFilter.setMin(minCoords);
    boxFilter.setNegative(negative);
    boxFilter.filter(*output);
    return output;
  }

}// namespace filters
}// namespace pcl_ros_wrapper
#endif /* CROP_BOX_FILTER_HPP */