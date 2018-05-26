/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: ivdrivercontrol
* FileName: drivercontrol.h, drivercontrol.cpp
*
* Description:
* 1. Using Pid algrithm to control actuator.
*
* History:
* yanbo         16/11/20    1.0.0    build this module.
******************************************************************************/
#ifndef _IVDRIVERCONTROL_H
#define _IVDRIVERCONTROL_H
#pragma once

// C++ lib
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

typedef short int16;
typedef double float64;
typedef float float32;
typedef struct sPointMsg {
  float32 x;
  float32 y;
  float32 length;
  float32 velocity;
  // float32 ramp_type;
} sPointMsg;



// ROS lib
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory
#include "ivdrivercontrol/ivdrivercontrol.h"
#include "ivdrivercontrol/ivdriverdebug.h"
#include "ivdrivercontrol/ivdrivercontrolstates.h"
#include "ivactuator/ivactuator.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/motionpoint.h"
//#include "ivdrivercontrol.h"

#include "drivercontrol.h"
#include "drivercontrolXJ.h"
#include "../../../avoslib/globalvariables.h"

#define SAFE_DISTANCE 0.8 //0.8
#define INVALID_DISTANCE -1.0
#define ERROR -1

// varying velocity test
 #define VELOCITY_TEST

// log switch
//#define DEBUG
//#define WARN

#ifdef WARN
#define WARN_LOG(message) warn_log(__FUNCTION__, __LINE__, (message))
#else
#define WARN_LOG(message)
#endif

#ifdef DEBUG
#define DEBUG_LOG(message) debug_log(__FUNCTION__, __LINE__, (message))
#else
#define DEBUG_LOG(message)
#endif

#define MAX(a, b) ((a) >= (b)? (a) : (b))
#define MIN(a, b) ((a) <= (b)? (a) : (b))





#endif  // _IVDRIVERCONTROL_H
