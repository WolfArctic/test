cmake_minimum_required(VERSION 2.8.3)
project(ivlocalization)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  ivlocmsg
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Eigen3 REQUIRED)

set(HEADERS
    # Base
    include/kalman/KalmanFilterBase.hpp
    include/kalman/StandardBase.hpp
    include/kalman/SquareRootBase.hpp
    include/kalman/StandardFilterBase.hpp
    include/kalman/SquareRootFilterBase.hpp

    # EKF
    include/kalman/ExtendedKalmanFilter.hpp
    include/kalman/SquareRootExtendedKalmanFilter.hpp

    # UKF
    include/kalman/UnscentedKalmanFilterBase.hpp
    include/kalman/UnscentedKalmanFilter.hpp
    include/kalman/SquareRootUnscentedKalmanFilter.hpp

    # Models
    include/kalman/MeasurementModel.hpp
    include/kalman/SystemModel.hpp
    include/kalman/LinearizedMeasurementModel.hpp
    include/kalman/LinearizedSystemModel.hpp

    # Utils
    include/kalman/Matrix.hpp
    include/kalman/Types.hpp
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
   tf
   sensor_msgs
   ivlocmsg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Eigen3_INCLUDE_DIR}
)

link_directories(../../../../devel/lib)
include_directories(../../../fmea/ivmonitor/src/monitor_lib)

add_executable(ivlocalization 
	../../avoslib/geotool.cpp 
	src/localization.cpp
	${HEADERS})
add_dependencies(ivlocalization 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ivlocalization ivmonitor_client ${catkin_LIBRARIES})
