cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_distance)

find_package(catkin REQUIRED pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(pointcloud_distance src/pointcloud_distance.cpp)
target_link_libraries(pointcloud_distance ${catkin_LIBRARIES})