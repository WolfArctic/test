cmake_minimum_required(VERSION 2.8.3)
project(ivpathplanner)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  message_generation
  roscpp
  nav_msgs
  rospy
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
   FILES
   ivmsgpath.msg
   motionpoint.msg
   motiondebug.msg
   velocitydebug.msg
   ivmsgpathfeedback.msg
   ivmsgpathplanner.msg
   path.msg
   pathpoint.msg
   deliverinfo.msg
   carstateshow.msg
 )



## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ivpathplanner
  CATKIN_DEPENDS
  message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/refpath
  src/motionplanner
  src/motionplanner/mathtool
  src/motionplanner/fuzzycontrol
  src/avoidObsModel/cbp
  src/avoidObsModel/cbp/common
  src/avoidObsModel/cbp/dwa
  src/avoidObsModel/cbp/sehs
  src/avoidObsModel/cbp/Ipopt
  src/avoidObsModel/cbp/Ipopt/include
  src/avoidObsModel/freespace_planner
)

link_directories(src/avoidObsModel/cbp/Ipopt/lib)

set(SOURCE_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/dubins.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/dwa/dwaplanner.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/reeds_shepp.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/visualize.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/hermite_curve.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/sehs/sehs.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/sehs/exploration.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/sehs/modelsimulation.cpp
  )

set(HEADER_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/functions.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/dubins.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/dwa/dwaplanner.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/reeds_shepp.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/visualize.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/common/hermite_curve.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/sehs/sehs.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/sehs/stateheaders.h
	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/cbp/dwa/commonheaders.h
	)

# set(ASTAR_SOURCE_FILES
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/astar_util.cpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/astar_search.cpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/search_info_ros.cpp
#     )

# set(ASTAR_HEADER_FILES
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/astar_util.h
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/astar_search.h
# 	${CMAKE_CURRENT_SOURCE_DIR}/src/avoidObsModel/freespace_planner/search_info_ros.h
# 	)

## Declare a C++ library
# add_library(libtool
#   ../../avoslib/geotool.cpp
#   ../../avoslib/iabasemaptool.cpp
# )

link_directories(../../../../devel/lib)   
include_directories(../../../fmea/ivmonitor/src/monitor_lib)


#src/ivpathplanner.cpp src/yuyanpathplanner.cpp
#src/refpath/bdpath.cpp src/refpath/meroad.cpp
add_executable(ivpathplanner
  src/ivpathplanner.cpp src/omegapathplanner.cpp ../../avoslib/geotool.cpp  
  src/refpath/refpath.cpp src/motionplanner/motionplanner.cpp  src/motionplanner/motionplannerWBD.cpp
  ../../avoslib/iabasemaptool.cpp
  src/motionplanner/mathtool/mathtool.cpp
  src/motionplanner/fuzzycontrol/fuzzycontrol.cpp
  src/motionplanner/minijerk/jmt.cpp src/motionplanner/minijerk/behaviorplanner.cpp src/motionplanner/minijerk/lib/filter.cpp
  src/motionplanner/minijerk/lib/mathLib.cpp src/motionplanner/minijerk/lib/visualization.cpp 
  src/motionplanner/minijerk/pathdecision.cpp src/motionplanner/minijerk/trajectory.cpp src/motionplanner/minijerk/trajectorycost.cpp
  src/motionplanner/minijerk/velocitygeneration.cpp
  src/motionplanner/kalmanfilter/vekf.cpp src/motionplanner/kalmanfilter/oneDimKalman.cpp
  ${SOURCE_FILES}
  )
add_dependencies(ivpathplanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ivpathplanner
  ivmonitor_client
  ${catkin_LIBRARIES}
  )
target_link_libraries(ivpathplanner ${OpenCV_LIBRARIES})
target_link_libraries(ivpathplanner ${PCL_LIBRARIES})

add_executable(ivcbp src/avoidObsModel/cbp/ivcbp.cpp ${SOURCE_FILES}
  ../../avoslib/geotool.cpp ../../avoslib/iabasemaptool.cpp
  )
add_dependencies(ivcbp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ivcbp ${catkin_LIBRARIES})
target_link_libraries(ivcbp ${OpenCV_LIBRARIES})



