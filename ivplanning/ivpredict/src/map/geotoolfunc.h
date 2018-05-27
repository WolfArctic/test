#ifndef _GEOTOOL_PREDICT
#define _GEOTOOL_PREDICT
#pragma once
//c++ lib

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivpredict/predictobj.h"
#include "ivpredict/objbuffer.h"

#include "../glovariable.h"
#include "../globaldefine.h"
//#include "../predictmethods/commonfunctions.h"
#include "../../../../avoslib/geotool.h"

using namespace std;

class geotoolfunc
{
public:
  geotoolfunc();
  ~geotoolfunc();
  int PredictGCC2VCS(ivpredict::ivmsgpredict *target_Predict, ivmap::ivmapmsglocpos *pIvlocpos);
  int PredictGCC2VCS_one(ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos);
};

#endif