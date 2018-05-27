/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: buffer.h
Description: 
1. 
2. 
3. 

History:
<author>    <time>      <version>    <description> 
hui li      17/11/10    1.0.0       buffer moved to container  

************************************************************/

#ifndef PREDICT_BUFFER
#define PREDICT_BUFFER
#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
using namespace std;

//self lib
#include "../glovariable.h"
#include "../globaldefine.h"
#include "kalmanpredict.h"

class buffer
{
    friend class omegapredict;
  public:
    buffer();
    ~buffer();
    int Update(sIAMap iamap);
    ivpredict::ivpredictbuffer obj_info;
};

#endif