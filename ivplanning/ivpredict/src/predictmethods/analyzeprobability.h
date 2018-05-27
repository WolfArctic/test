/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: analyzeprobability.h
Description:
1. analyze relative probability for each predicted path.
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年06月12日	V1.1		build this module
************************************************************/

#ifndef _ANALYZEPROBABILITY_H
#define _ANALYZEPROBABILITY_H
#pragma once

#include "../globaldefine.h"
#include "../glovariable.h"
#include "../common/commonfunctions.h"

/**************************************************************
Function: AddProbabilityToShortLongPath()
Author: lihui/zongwei
Date: 2017.4.19;2017.5.12;2017.6.6
Description: // 函数功能、性能等的描述
1. analyze relative probability for each predicted path by compairing predicted paths with 5s short predicted path. 
2. -------
3. -------
Input:
1. short predicted path with 5s.
2. predicted shortlong paths.
Output: nothing
Return: 
1. shortlong paths with probability.
Others: // 其他说明
************************************************************/
ivpredict::ivmsgpredict AddProbabilityToShortLongPath(ivpredict::predictobj shortPredict, ivpredict::ivmsgpredict shortLongPredictedPaths)
{   
  ivpredict::ivmsgpredict outputPredictObj;

  if (shortLongPredictedPaths.objects.size() == 1)
  {
    shortLongPredictedPaths.objects[0].probability = 1.0;
    outputPredictObj.objects.push_back(shortLongPredictedPaths.objects[0]);
  }
  else
  {
    float totalDis = 0.0, tempDis = 0.0, sumTempPossibility = 0.0;      
    
    //calculate distance
    for (int i = 0; i < shortLongPredictedPaths.objects.size(); i++)
    {
      tempDis = CalDisBetweenTwoSegments(shortLongPredictedPaths.objects[i], shortPredict);
      totalDis += fabs(tempDis);
      shortLongPredictedPaths.objects[i].probability = fabs(tempDis);
    }    
    
    //calculate initial probability
    for (int i = 0; i < shortLongPredictedPaths.objects.size(); i++)
    {
      shortLongPredictedPaths.objects[i].probability = 1 - shortLongPredictedPaths.objects[i].probability/totalDis;
      sumTempPossibility += shortLongPredictedPaths.objects[i].probability;
    }  
    
    //revise initial probability according to tempt probability
    for (int i = 0; i < shortLongPredictedPaths.objects.size(); i++)
    {
      shortLongPredictedPaths.objects[i].probability = shortLongPredictedPaths.objects[i].probability/sumTempPossibility;
      // shortLongPredictedPaths.objects[i].intention = shortLongPredictedPaths.objects[i].intention;
      outputPredictObj.objects.push_back(shortLongPredictedPaths.objects[i]);
    }

    
    //calculate final probability
    double minP=1,maxP=0;
    for (int j = 0; j < outputPredictObj.objects.size(); j++)
    {
      if(outputPredictObj.objects[j].probability>maxP)
      {
	maxP=outputPredictObj.objects[j].probability;
      }
      if(outputPredictObj.objects[j].probability<minP)
      {
	minP=outputPredictObj.objects[j].probability;
      }
    }
    if(maxP==minP)
    {
      for (int j = 0; j < outputPredictObj.objects.size(); j++)
      {
	outputPredictObj.objects[j].probability=1;
      }
    }
    else
    {
      for (int j = 0; j < outputPredictObj.objects.size(); j++)
      {   
	double rate=0.5;
	outputPredictObj.objects[j].probability=(outputPredictObj.objects[j].probability/maxP)*rate+(outputPredictObj.objects[j].probability-minP)/(maxP-minP)*(1-rate);
      }
    }
  }

  return outputPredictObj;
}

#endif
