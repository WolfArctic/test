/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: direction-decide.h
Description: 
1. human intension

History:
<author>    <time>      <version>    <description> 
hui li      17/06/06    1.0.1       modified function names (delete _ )  

************************************************************/

#ifndef DIRECTION_EVALUATOR
#define DIRECTION_EVALUATOR
#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "../../globaldefine.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

class direction_decide
{
public:
	direction_decide();
	~direction_decide();
	// void checkWarningVap(double runTime,double CallbackTime);
	int UpdateState(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict, ivpredict::predictobj* answer_predict);
private:
	// Monitor *predictMonitor;
	int GetDis(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict);
	int Predict90(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result);
	int Stop(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result);
	int NoPredict(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result);
	int Predict0(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result, ivpredict::predictobj* self_predict);
	int Predict45(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result);
	bool ToStop(ivpredict::objbuffer *pObj);
	bool ToCross(ivpredict::objbuffer *pObj);
	bool To45(ivpredict::objbuffer *pObj);
	bool ToParallel(ivpredict::objbuffer *pObj);
	int UpdateStates(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict);
	int GetCrossPoint(ivpredict::predictpos* obj_pos, ivpredict::predictpos* start_pos, ivpredict::predictpos* end_pos, ivpredict::predictpos* cross_pos);
};

#endif