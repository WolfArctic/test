cmake_minimum_required(VERSION 2.8.3)
project(ndt_matching)

set(CMAKE_BUILD_TYPE "Release")

if(PCL_VERSION VERSION_LESS "1.7.2")
  set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
else(PCL_VERSION VERSION_LESS "1.7.2")
  set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
endif(PCL_VERSION VERSION_LESS "1.7.2")

#set(FAST_PCL_PACKAGES filters registration)
  
find_package(CUDA)
if(CUDA_FOUND)
  option(USING_GPU_NDT_  "set to switch to build use GPU" ON)
  add_definitions(-DUSING_GPU_NDT_)
  set(USING_GPU_NDT_ true)
	message("\n found CUDA\n")
else()
	option(USING_GPU_NDT_  "set to switch to build use CPU" OFF)
	message("\n did not find CUDA\n")
endif()

if(USING_GPU_NDT_)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  add_definitions(-DUSING_HP)
  set(CUDA_ARCH "sm_61")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++11;--ptxas-options=-v)
  message("\n using high performance \n")
else()
	set(CUDA_ARCH "sm_52")
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};--ptxas-options=-v)
endif()
endif()

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS 
  ivlocmsg
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  velodyne_pointcloud
  message_generation
  # ${FAST_PCL_PACKAGES}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs velodyne_pointcloud ivlocmsg #${FAST_PCL_PACKAGES}
  DEPENDS 
)

if(USING_GPU_NDT_)
  include_directories(
    include 
    /usr/local/cuda/include
    ${catkin_INCLUDE_DIRS} 
  )

  cuda_add_library(gpu_ndt
    gpu/VoxelGrid_culib.cu
    gpu/NormalDistributionsTransform_culib.cu
    gpu/MatrixHost.cu
    gpu/NormalDistributionsTransform.cu
    gpu/Registration.cu
    gpu/VoxelGrid.cu
  )
else()
  include_directories(${catkin_INCLUDE_DIRS})
endif()

add_executable(ndt_matching src/ndt_matching.cpp)

if(USING_GPU_NDT_)
  target_link_libraries(ndt_matching ${catkin_LIBRARIES} gpu_ndt)
else()
  target_link_libraries(ndt_matching ${catkin_LIBRARIES})
endif()
