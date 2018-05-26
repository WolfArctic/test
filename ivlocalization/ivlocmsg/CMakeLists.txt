cmake_minimum_required(VERSION 2.8.3)
project(ivlocmsg)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs	
  sensor_msgs
  message_generation
)

 add_message_files(
    FILES
    ivsensorgps.msg
    ivsensorimu.msg
    ivsensorodom.msg
    ndt_status.msg
    ivmsglocpos.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})  
