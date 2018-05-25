/*******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
* NodeName: ivsteercontrol
* FileName: steercontrol.h, steercontrol.cpp
* 
* Description: 
* 1. calculate the steering angle to track the target path
* 2. calculate the torque
*
* History: 
* jianwei         17/06/20    1.0.0    build this module. 
********************************************************************************/
#ifndef _STEERCONTROL_H
#define _STEERCONTROL_H
#pragma once
//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

//ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"
//#include "monitor_client.h" 

using namespace Eigen;

struct sTargetPath
{
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> angle;
};
struct sNearPath
{
	VectorXd x;
	VectorXd y;
};

struct sThetaTargetNear
{
	double thetatargetnearmax;
	double thetatargetnearmin;
};

struct sPathCurrentVehicle
{
	double xleft;
	double yleft;
	double xright;
	double yright;
};

struct sTheAdjustDpreview
{
	double adjust_dpreview;
	double adjust_dpreview_near;
};

struct sArcReturn
{
	double radiusarc;
	double alpha1;
	double alpha2;
	double arc_center_x;
	double arc_center_y;
};
/*typedef struct 
{
	double alpha;
	double last_value;
	
}lpf_1p_param;*/

/*typedef struct 
{
	std::vector<double> data;
	int window_size;	
}mean_filter_param;*/

struct sPath
{
	VectorXd x;
	VectorXd y;
	VectorXd angle;
	VectorXd distance;
};

struct sTrackError
{ 
	double lateral_err;
	double heading_err;
};

 #endif  // _STEERCONTROL_H
