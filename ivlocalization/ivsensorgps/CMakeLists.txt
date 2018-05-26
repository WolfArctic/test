cmake_minimum_required(VERSION 2.8.3)
project(ivsensorgps)

find_package(catkin REQUIRED COMPONENTS
  ivlocmsg
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   serial
   ivlocmsg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(ivsensorgps src/ivsensorgps.cpp src/gpsparse.cpp ../../avoslib/geotool.cpp)
 add_dependencies(ivsensorgps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(ivsensorgps ${catkin_LIBRARIES})
