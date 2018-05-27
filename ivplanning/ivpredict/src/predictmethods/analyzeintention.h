/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: analyzeintention.h
Description:
1. analyze intention of the concerned object accoring to angle between moving direction and road segment
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年06月12日	V1.1		build this module
************************************************************/

#ifndef _ANALYZEINTENTION_H
#define _ANALYZEINTENTION_H
#pragma once

#include "../globaldefine.h"
#include "../glovariable.h"
#include "locateobject.h"

/**************************************************************
Function: CheckObjIntention()
Author: zongwei
Date: 2017.6.09
Description: // 函数功能、性能等的描述
1.  analyze intention of the concerned object accoring to angle between moving direction and road segment
2. -------
3. -------
Input:
1. angle: the angle between moving direction and road segment.unit: degree
Output:nothing
Return:
1. eObjIntention: that is, GOSTRAIGHT, CHANGESEGMENT or CROSSROAD.
Others: // 其他说明
************************************************************/
eObjIntention CheckObjIntention(ivpredict::predictobj *pShortPredictPath,sSegRel *pSCurrentSegment, sRoadSegRel *pNowRoadSegRel)
{
  eObjIntention eNowObjIntention;
  for(int i=pNowRoadSegRel->segRels.size()-1;i>=0;i--)
  {
    if(pNowRoadSegRel->currentSegmentType == pNowRoadSegRel->segRels[i].segmenttype && pNowRoadSegRel->currentSegmentNumber==pNowRoadSegRel->segRels[i].segmentnumber)
    {
      eNowObjIntention=GOSTRAIGHT;  //intention of going straight
    }
    else
    {
      eNowObjIntention=CHANGESEGMENT; //intention of changing segment
    }
  }
  
//-----------------------------------old method------------------------------------------------------
//   //------------Calulate Angle Between Moving direction And Current Segment---------
//   int shortPathPointsSize = pShortPredictPath->positions.size();
//   float tmpDetaY=pShortPredictPath->positions[floor(shortPathPointsSize/2)].ygmean - pShortPredictPath->positions[0].ygmean;
//   float tmpDetaX=pShortPredictPath->positions[floor(shortPathPointsSize/2)].xgmean - pShortPredictPath->positions[0].xgmean;
//   float angObjMove = atan2(tmpDetaY,tmpDetaX)*180/3.1415;  //object's moving direction
//   float angle= CalAngBetweenMoveAndCurrentSeg(angObjMove, pShortPredictPath->positions[0].xgmean, pShortPredictPath->positions[0].ygmean, pSCurrentSegment);  
//   //anlyze intention
//   eObjIntention eNowObjIntention;
//   if(abs(angle) < kAngThresholdMin) 
//   {
//     eNowObjIntention=GOSTRAIGHT; //intention of going straight
//   }
//   else if(abs(angle) < kAngThresholdMax)
//   {
//     eNowObjIntention=CHANGESEGMENT;  //intention of changing segment
//   }
//   else
//   {
//     eNowObjIntention=CROSSROAD;    //intention of crossing road.
//   }
//   return eNowObjIntention;
//-----------------------------------old method end-------------------------------------------------
}

#endif
