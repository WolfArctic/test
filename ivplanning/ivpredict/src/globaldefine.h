/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: matchpath.h
Description:
1. -------
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年03月17日	V1.1		build this module
************************************************************/

#ifndef _GLOBALDEFINE_H
#define _GLOBALDEFINE_H
#pragma once

// #define PI 3.1415926536
// #define CELLSIZERATE 0.1	// that is: cellsize/realsize.
#define kAngThresholdMin 3.0
#define kAngThresholdMax 60.0
#define kSegmentBoundary "e"
#define kSegmentCentreLine "c"
#define INVALID_OBJECT_ID -88
#define INVALID_SPEED_ID -88
#define FREQUENCY 10
#define BUFFER_SIZE FREQUENCY*3


// typedef struct sSegRel{
//   string segID;
//   double* segName;
//   int maxLineNum;
//   //string relation;
//   bool isInvSearch;
//   std::vector<sSegRel> subSeg;
// }sSegRel;

// typedef struct sBeCrossRoadStatus{
//   bool isInCrossRoad;
//   double disToCrossRoad;
//   int timeToGreenSig;
// }sBeCrossRoadStatus;
//msg
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/ivmapmsgroad.h"
#include "ivmap/ivmapmsgvsp.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/ivmapmsgapp.h"
#include "ivmap/ivmapmsguserfun.h"
#include "ivmap/ivmapmsgtraffic.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgloctag.h"
#include "ivpredict/ivmsgpredict.h"
#include "ivpredict/ivmsgpredictdebug.h"
#include "ivpredict/predictobj.h"
#include "ivpredict/predictobjdebug.h"
#include "ivpredict/predictpos.h"
#include "ivpredict/ivpredictbuffer.h"
#include "ivpredict/objbuffer.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/motionpoint.h"

using namespace std;

typedef struct sIAMap
{
    ivmap::ivmapmsgobj ivobj;
    ivmap::ivmapmsgroad ivroad;
    ivmap::ivmapmsgvsp ivvsp;
    ivmap::ivmapmsgvap ivvap;
    ivmap::ivmapmsgapp ivapp;
    ivmap::ivmapmsguserfun ivuserfun;
    ivmap::ivmapmsgtraffic ivtraffic;
    ivmap::ivmapmsglocpos ivlocpos;
    ivmap::ivmapmsgloctag ivloctag;
}sIAMap;

typedef struct sGPSpoint{
  double lon;
  double lat;
}sGPSpoint;

typedef struct sGCCSpoint{
  double xg;
  double yg;
}sGCCSpoint;

//segment relation
typedef struct sSegRel{
  string segmenttype;
  int segmentnumber;
  double possibility;
  int maxLineNum;
  std::vector<sGPSpoint> GPS;
  std::vector<sGCCSpoint> GCCS;
  std::vector<sSegRel> subSeg;
  bool isInvSearch;
}sSegRel;

typedef struct sRoadSegRel{
  //int roadType;
  int objID;
  string currentSegmentType;
  int currentSegmentNumber;
  float objHeading;
  std::vector<sSegRel> segRels;
  //sBeCrossRoadStatus nowBeCrossRoad;
}sRoadSegRel;

typedef struct sRelationList{
  string segmentType;
  int segmentNumber;
  float length;
  float speed;
  std::vector<int> fatherID;
  std::vector<int> sonID;
  std::vector<int> parallelID;
}sRelationList;

typedef struct sRelationship
{
  std::vector<sRelationList> list;
  // std::vector<int> relationMatrix;
}sRelationship;

typedef struct sMapInformation
{
  sRelationship mysRelationship;
  std::vector<sSegRel> segments_buffer;
}sMapInformation;

typedef struct sMapInformationAll
{
  string mapType;
  std::vector<sMapInformation> mapStructure;
}sMapInformationAll;

//----------store nearest point on each adjacent segment------------------
typedef struct sSingleSegNearPoint{
  int objID;
  string segmenttype;
  int segmentnumber;
  int maxLineNum;
  int pLineNum;
}sSingleSegNearPoint;

typedef struct sSingleObjIDandSegID{
  int objID;
  string segmenttype;
  int segmentnumber;
  int segmentmapindex;
}sSingleObjIDandSegID;

typedef struct sCurSegAndGoalSeg{
  int objID;
  string currentsegmenttype;
  int    currentsegmentnumber;
  int    currentsegmentmapindex;
  std::vector<int> historyCurrentSegMapIndex;
  bool   finishedGoalSegment;
  string goalsegmenttype;
  int    goalsegmentnumber;
//   int    goalsegmentmapindex;
}sCurSegAndGoalSeg;

typedef struct sMidVariables{
     sSegRel *pSCurrentSegment;
//   float gAngBetweenMoveAndCurrentSeg;	//angle between moving direction and current segment for the object. unit:°.
//   bool  needChangeSegRemoved;		//does it need to change segments?
}sMidVariables;

enum eSegRelation{
  NORELATION=0,
  PARALLELRELATIONSEG=5,
};

enum eObjIntention{
  GOSTRAIGHT=1,
  CHANGESEGMENT=2,
  CROSSROAD=3,
  STOP=4,
};

enum ePredictMethod{
  eOMEGA=0,
  eYUYAN=1,
  eAbs=2,
};

#endif
