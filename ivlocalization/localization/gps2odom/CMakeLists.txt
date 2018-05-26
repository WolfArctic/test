cmake_minimum_required(VERSION 2.8.3)
project(gps2odom)

find_package(catkin REQUIRED COMPONENTS
  ivlocmsg
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  serial
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
   tf
   sensor_msgs
   serial
   ivlocmsg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps2odom src/gps2odom.cpp ../../../avoslib/geotool.cpp)
add_dependencies(gps2odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps2odom ${catkin_LIBRARIES})
