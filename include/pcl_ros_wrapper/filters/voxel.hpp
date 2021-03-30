#ifndef VOXEL_GRID_FILTER_HPP
#define VOXEL_GRID_FILTER_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros_wrapper/common/types.hpp>
#include <pcl_ros_wrapper/common/type_traits.hpp>

namespace pcl_ros_wrapper {
namespace filters {

  /**
   * @brief Apply Voxel Grid filter on the given point cloud.
   * 
   * @tparam T 
   * @param input A given point cloud.
   * @param leaf_size_x Leaf size in x direction.
   * @param leaf_size_y Leaf size in y direction
   * @param leaf_size_z Leaf size in z direction
   * @return Pointer to the filtered pointcloud result
   */
  template<typename T>
  boost::shared_ptr<RemoveConst<element_type_t<T>>> do_voxel_filtering(const T& input,
                                                                       float leaf_size_x,
                                                                       float leaf_size_y,
                                                                       float leaf_size_z)
  {
    using CloudType = RemoveConst<element_type_t<T>>;
    using PointType = typename CloudType::value_type;
    auto output     = boost::make_shared<CloudType>();

    pcl::VoxelGrid<PointType> voxel;
    if constexpr (std::is_same<T, element_type_t<T>>::value) {
      voxel.setInputCloud(boost::make_shared<element_type_t<T>>(input));
    } else {
      voxel.setInputCloud(input);
    }
    voxel.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    voxel.filter(*output);
    return output;
  }

}// namespace filters
}// namespace pcl_ros_wrapper

#endif /* VOXEL_GRID_FILTER_HPP */