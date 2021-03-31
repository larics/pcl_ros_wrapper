cmake_minimum_required(VERSION 2.8.3)
project(pcl_ros_wrapper)

# NOTE: The following three lines are needed to force compile PCL with -std=c++17 see
# issue https://github.com/PointCloudLibrary/pcl/issues/2686
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED roscpp roslib)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  pcl_wrapper
  sample_consensus_wrapper
  DEPENDS
  PCL)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# LIBRARIES

add_library(registration_wrapper src/registration/icp.cpp)
target_link_libraries(registration_wrapper ${PCL_LIBRARIES})

add_library(segmentation_wrapper src/segmentation/sac.cpp)
target_link_libraries(segmentation_wrapper ${PCL_LIBRARIES})

add_library(filters_wrapper src/filters/upsampling.cpp)
target_link_libraries(filters_wrapper ${PCL_LIBRARIES})

add_library(common_wrapper src/common/general.cpp src/common/surface.cpp)
target_link_libraries(common_wrapper ${PCL_LIBRARIES})

# EXAMPLES

add_executable(crop_box_example examples/crop_box.cpp)
target_link_libraries(crop_box_example ${PCL_LIBRARIES})

add_executable(voxel_grid_example examples/voxel_grid.cpp)
target_link_libraries(voxel_grid_example ${PCL_LIBRARIES})

add_executable(outlier_filter_example examples/outlier_filter.cpp)
target_link_libraries(outlier_filter_example ${PCL_LIBRARIES})

add_executable(sample_consensus_example examples/sample_consensus.cpp)
target_link_libraries(sample_consensus_example ${PCL_LIBRARIES} segmentation_wrapper)
