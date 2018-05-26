cmake_minimum_required(VERSION 2.8.3)
project(map_loader)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  tf 
  serial
  pcl_ros
  pcl_conversions
  ivlocalization
)

find_package(PCL REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})  
   
# add_executable(talker src/talker.cpp)  
# target_link_libraries(talker ${catkin_LIBRARIES})  
# add_dependencies(talker mapmerge_generate_messages_cpp)  
  
add_executable(map_loader src/map_loader.cpp)  
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})  
add_dependencies(map_loader mapmerge_generate_messages_cpp)  

add_dependencies(map_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
