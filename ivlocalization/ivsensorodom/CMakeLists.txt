cmake_minimum_required(VERSION 2.8.3)
project(ivsensorodom)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf2 tf2_ros std_msgs genmsg serial ivlocmsg)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(   
   CATKIN_DEPENDS roscpp rospy std_msgs tf2 tf2_ros ivlocmsg
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
    ivsensorodom
    src/ivsensorodom.cpp
    src/aodom.cpp
    src/odom_node_can.cpp
)
target_link_libraries(ivsensorodom ${catkin_LIBRARIES})
add_dependencies(ivsensorodom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
