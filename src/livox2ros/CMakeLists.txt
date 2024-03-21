cmake_minimum_required(VERSION 3.0)
project(livox2ros)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosbag sensor_msgs std_msgs pcl_conversions
  DEPENDS PCL EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(livox2ros src/livox2ros.cpp)
target_link_libraries(livox2ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(livox2ros ${catkin_EXPORTED_TARGETS})
