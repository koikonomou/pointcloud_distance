cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_distance)

find_package(catkin REQUIRED 
  navigation_srv
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  )

catkin_package( 
    CATKIN_DEPENDS navigation_srv)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_distance src/pointcloud_distance.cpp)
target_link_libraries(pointcloud_distance ${catkin_LIBRARIES})
