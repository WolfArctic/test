/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: omegapredict.h
Description: 
1. library define
2. static varrible difine
3. omegapredict class declear

History:
<author>    <time>      <version>    <description> 
hui li      17/06/06    1.0.1       modified function names (delete _ )  

************************************************************/

#ifndef MONITOR_PREDICT
#define MONITOR_PREDICT
#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "monitor_client.h"   //guzhangzhengduan

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

#define periodN 50.0
#define periodeN 50.0

#define later_Time1 10
#define later_Time2 20
#define later_Time3 30
#define later_Time4 50
#define later_Time5 100
#define later_Time6 500

using namespace std;

class monitor
{
public:
	monitor();
	~monitor();
	void checkWarningVap(double runTime,double CallbackTime);
private:
	Monitor *predictMonitor;
};

#endif