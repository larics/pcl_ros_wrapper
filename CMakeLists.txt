cmake_minimum_required(VERSION 2.8.3)
project(pcl_ros_wrapper)

# NOTE: The following three lines are needed to force compile PCL with -std=c++17 see
# issue https://github.com/PointCloudLibrary/pcl/issues/2686
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES pcl_wrapper DEPENDS PCL)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(registration_wrapper src/registration/icp.cpp)
target_link_libraries(registration_wrapper ${PCL_LIBRARIES})

add_executable(crop_box_example examples/crop_box.cpp)
target_link_libraries(crop_box_example ${PCL_LIBRARIES})