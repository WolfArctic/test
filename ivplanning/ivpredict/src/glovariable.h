/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: glovariable.h
Description:
1. -------
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年03月17日	V1.1		build this module
************************************************************/

#ifndef _GLOVARIABLE_H
#define _GLOVARIABLE_H
#pragma once

#include <boost/concept_check.hpp>
#include "globaldefine.h"
#include "ivpredict/ivmsgpredictdebug.h"
#include "ivpredict/predictobjdebug.h"

static std::vector<sSingleSegNearPoint> gSavedSegNearPoints;
static std::vector<sSingleObjIDandSegID> gAllObjIDandSegID;
static float gMinVelThreshold=1.5;		//minimum velocity to classify static objects in condition of noise. unit:m/s.
static int stop_flag;
static sMidVariables gMidVarialblesRecord;	//middle controlling variables.
static sMapInformation gLoadedMap;
static sMapInformationAll gLoadedMapAll;
static std::vector<sCurSegAndGoalSeg> gLastObjIDandCurrentSeg;

static bool gFirstTimeP0=false;
//for egoCar
static double gEgoCarPosXg,gEgoCarPosYg,gEgoCarVg,gExistEgoCarLastSeg=false;
static sSingleObjIDandSegID gEgoCarLastSeg;
static int gBufferSizeForIntention=10;
static double gHalfLaneWidthForEgoCar=3.75/2;

//predict time and timeStep
static double gTimeLength=5.0;
static double gTimeStep=0.1;

static bool   gSetShortTimeMode;
static bool   gSetLongTimeMode;
static bool   gSetTimeMode=true;
static bool   gSetDisMOde;
static double gPathTimeSet;
static double gPathDisSet=50.0;
static double gSpeed_stop=1.0;

//variables for debug
static ivpredict::ivmsgpredictdebug debugMsgForRviz;
static bool gDebug=false;
//static int predict_lidar = 
static int predict_lidar = 0;  //0 : only deal with abs positions and speed 2: 

#endif
