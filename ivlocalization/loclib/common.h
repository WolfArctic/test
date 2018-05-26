#ifndef COMMON_H
#define COMMON_H
#pragma once

//c++ lib
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

template<typename FloatType>
struct sPoint2D {
    FloatType x;
    FloatType y;
};
typedef sPoint2D<float> sPoint2Df;
typedef sPoint2D<double> sPoint2Dd;

template<typename FloatType>
struct sPose2D
{
  FloatType x;
  FloatType y;
  FloatType angle;    
};
typedef sPose2D<float> sPose2Df;
typedef sPose2D<double> sPose2Dd;

template<typename FloatType>
struct sPose3D
{
  FloatType x;
  FloatType y;
  FloatType z;
  FloatType roll;
  FloatType pitch;
  FloatType yaw;    
};
typedef sPose3D<float> sPose3Df;
typedef sPose3D<double> sPose3Dd;

template<typename FloatType>
struct s2DLineEquation {
  //y=kx+b;
  FloatType k;
  FloatType b;
  sPoint2D<FloatType> start;
  sPoint2D<FloatType> end;
  pcl::PointIndices::Ptr inliers;
  bool isValid;
  int confidence;
  FloatType offset;
  s2DLineEquation<FloatType>(): k(0), b(0), isValid(false), confidence(0), offset(0) {}
};
typedef s2DLineEquation<float> s2DLineEquationf;
typedef s2DLineEquation<double> s2DLineEquationd;

template<typename FloatType>
struct sPlaneEquation {
  FloatType a;
  FloatType b;
  FloatType c;
  bool isValid;
};
typedef sPlaneEquation<float> sPlaneEquationf;
typedef sPlaneEquation<double> sPlaneEquationd;

template<typename FloatType>
struct s2DCurveEquation {
  //y = ax2+bx+c
  FloatType a;
  FloatType b;
  FloatType c;
  sPoint2D<FloatType> start;
  sPoint2D<FloatType> end;
  std::vector<int> indices;
  bool isValid;
};
typedef s2DCurveEquation<float> s2DCurveEquationf;
typedef s2DCurveEquation<double> s2DCurveEquationd;


template<typename FloatType, typename PointType>
struct sTrajectoryNode
{
  pcl::PointCloud<PointType> points;
  int id;
  Eigen::Matrix<FloatType, 4, 4> pose;
  ros::Time stamp;
  bool isValid;
};
typedef sTrajectoryNode<float, pcl::PointXYZI> TrajectoryNode;
typedef std::vector<sTrajectoryNode<float, pcl::PointXYZI>> Trajectory;

#endif