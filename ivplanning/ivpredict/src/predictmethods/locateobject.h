/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: locateobject.h
Description:
1. locate current segment and corresponding segments of the object
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年06月12日	V1.1		build this module
************************************************************/

#ifndef _LOCATEOBJECT_H
#define _LOCATEOBJECT_H
#pragma once

#include "../globaldefine.h"
#include "../glovariable.h"
#include "../common/commonfunctions.h"

#define SEGSIZE 5

sSingleObjIDandSegID LocateEgoCarCurrentSegment(sMapInformation *pSMap, double xg, double yg)
{
  sSingleObjIDandSegID egoCarCurrentSegment;
  double halfLaneWidthForEgoCar=3.75/2;
  double errorWidth=0.2;

  //revise halfLaneWidthForEgoCar
  float closedDis = 1000;
  float secondClosedDis=1000;
  int   egocarCurrentSegMapIndex=-1;
  int   secondDisSegIndex=-1;
  std::vector<float> disArray; 
  for (int i = 0; i < pSMap->segments_buffer.size(); i++)
  {
    float distance= CalPoint2SegmentDis(xg, yg, &(pSMap->segments_buffer[i])); 
    if (distance < closedDis)
    {
      closedDis = distance;
      egocarCurrentSegMapIndex=i;
    }
    disArray.push_back(distance);
  }
  for (int i = 0; i < pSMap->segments_buffer.size(); i++)
  {
    if(disArray[i]<secondClosedDis && i!=egocarCurrentSegMapIndex)
    {
      secondClosedDis=disArray[i];
      secondDisSegIndex=i;
    }
  }
  if(egocarCurrentSegMapIndex<0 || secondDisSegIndex<0) {
//cout<<"pSMap->segments_buffer[<0] on locateobject.h, please check map!"<<endl;
}
  
  int pointIndex1 = CalNearestSegmentPointIndex(xg, yg, &(pSMap->segments_buffer[egocarCurrentSegMapIndex]));
  int pointIndex2 = CalNearestSegmentPointIndex(xg, yg, &(pSMap->segments_buffer[secondDisSegIndex]));
  double x1=pSMap->segments_buffer[egocarCurrentSegMapIndex].GCCS[pointIndex1].xg;
  double y1=pSMap->segments_buffer[egocarCurrentSegMapIndex].GCCS[pointIndex1].yg;
  double x2=pSMap->segments_buffer[secondDisSegIndex].GCCS[pointIndex2].xg;
  double y2=pSMap->segments_buffer[secondDisSegIndex].GCCS[pointIndex2].yg;    
  double ang=atan2(y1-yg,x1-xg)-atan2(y2-yg,x2-xg);
  if(cos(ang)<0 && closedDis+secondClosedDis<3.75+1.5)
  {
    halfLaneWidthForEgoCar=(closedDis+secondClosedDis)/2;
  }
  else
  {
    halfLaneWidthForEgoCar=3.75/2;
  }
  gHalfLaneWidthForEgoCar=halfLaneWidthForEgoCar;
  //analyze current segment according to different conditions
  if(closedDis<=halfLaneWidthForEgoCar-errorWidth)
  {
    egocarCurrentSegMapIndex=egocarCurrentSegMapIndex;
  }    
  else if(closedDis<=halfLaneWidthForEgoCar+errorWidth)
  {  
    if(gExistEgoCarLastSeg==false)
    {
      egocarCurrentSegMapIndex=egocarCurrentSegMapIndex;
    }
    else
    {  
      egocarCurrentSegMapIndex=gEgoCarLastSeg.segmentmapindex;
    }
  }
  else if(closedDis>halfLaneWidthForEgoCar+errorWidth)
  {
    egocarCurrentSegMapIndex=egocarCurrentSegMapIndex;
  }  

  //save as last current segment for the object
  egoCarCurrentSegment.segmenttype  =pSMap->segments_buffer[egocarCurrentSegMapIndex].segmenttype;
  egoCarCurrentSegment.segmentnumber=pSMap->segments_buffer[egocarCurrentSegMapIndex].segmentnumber;
  egoCarCurrentSegment.segmentmapindex=egocarCurrentSegMapIndex; 
  
  gEgoCarLastSeg=egoCarCurrentSegment;
  gExistEgoCarLastSeg=true;

  return egoCarCurrentSegment;
}


/**************************************************************
Function: LocateObjCurrentSeg()
Author: lihui/zongwei
Date: 2017.6.09/2017.7.10
Description: // 函数功能、性能等的描述
1. analyze current segment accoring to minimum distance.
2. -------
3. -------
Input:
1. pShortPredictPath: pointer to predicted short term path
2. pSMap: whole road segment map
Output: nothing
Return: current segment which the object is located on.
Others: // 其他说明
************************************************************/
sSegRel *LocateObjCurrentSeg(ivpredict::predictobj *pShortPredictPath, sMapInformation *pSMap, double xg, double yg)
{
  double errorWidth=0.4; //unit:m
  double halfLaneWidth=3.75/2;
  
  float closedDis = 1000;
  int   objCurrentSegMapIndex=-1;  
  float secondClosedDis=1000;
  int   secondDisSegMapIndex=-1;
  std::vector<float> disArray;
  for (int i = 0; i < pSMap->segments_buffer.size(); i++)
  {
    float distance= CalPoint2SegmentDis(xg, yg, &(pSMap->segments_buffer[i])); 

    if (distance < closedDis)
    {
      closedDis = distance;
      objCurrentSegMapIndex=i;
    }
    disArray.push_back(distance);
  }
  for (int i = 0; i < pSMap->segments_buffer.size(); i++)
  {
    if(disArray[i]<secondClosedDis && i!=objCurrentSegMapIndex)
    {
      secondClosedDis=disArray[i];
      secondDisSegMapIndex=i;
    }
  }

  int pointIndex1 = CalNearestSegmentPointIndex(xg, yg, &(pSMap->segments_buffer[objCurrentSegMapIndex]));
  int pointIndex2 = CalNearestSegmentPointIndex(xg, yg, &(pSMap->segments_buffer[secondDisSegMapIndex]));
  double x1=pSMap->segments_buffer[objCurrentSegMapIndex].GCCS[pointIndex1].xg;
  double y1=pSMap->segments_buffer[objCurrentSegMapIndex].GCCS[pointIndex1].yg;
  double x2=pSMap->segments_buffer[secondDisSegMapIndex].GCCS[pointIndex2].xg;
  double y2=pSMap->segments_buffer[secondDisSegMapIndex].GCCS[pointIndex2].yg;
  double ang=atan2(y1-yg,x1-xg)-atan2(y2-yg,x2-xg);
  if(cos(ang)<0 && closedDis+secondClosedDis<3.75+1.5)
  {
    halfLaneWidth=(closedDis+secondClosedDis)/2;
  }
  else
  {
    halfLaneWidth=3.75/2;
  }

  //analyze current segment according to different conditions
  if(closedDis<=halfLaneWidth-errorWidth)
  {
    objCurrentSegMapIndex=objCurrentSegMapIndex;
  }
  else if(closedDis<=halfLaneWidth+errorWidth)
  {
    bool existLastCurrentSegForTheObj=false;
    int indexJ=-1;
    int storedObjNum=gLastObjIDandCurrentSeg.size();
    for(int j=0;j<storedObjNum;j++)
    {    
      if(gLastObjIDandCurrentSeg[j].objID==pShortPredictPath->id)
      {
	existLastCurrentSegForTheObj=true;
	indexJ=j;
	break;
      }
    } 
    if(existLastCurrentSegForTheObj==true)
    {
      objCurrentSegMapIndex=gLastObjIDandCurrentSeg[indexJ].currentsegmentmapindex;
    }
    else
    {
      objCurrentSegMapIndex=objCurrentSegMapIndex;
      sCurSegAndGoalSeg tmpObj;
      tmpObj.objID=pShortPredictPath->id;
      tmpObj.currentsegmenttype=pSMap->segments_buffer[objCurrentSegMapIndex].segmenttype;
      tmpObj.currentsegmentnumber=pSMap->segments_buffer[objCurrentSegMapIndex].segmentnumber;
      tmpObj.currentsegmentmapindex=objCurrentSegMapIndex;
      tmpObj.finishedGoalSegment=false;
      gLastObjIDandCurrentSeg.push_back(tmpObj);
    }
  }
  else if(closedDis>halfLaneWidth+errorWidth)
  {
    objCurrentSegMapIndex=objCurrentSegMapIndex;
  }
  
  //save as last current segment for the object
  bool existLastCurrentSegForTheObj=false;
  for(int j=gLastObjIDandCurrentSeg.size()-1;j>=0;j--)
  {
    if(gLastObjIDandCurrentSeg[j].objID==pShortPredictPath->id)
    {
      //save current segment
      gLastObjIDandCurrentSeg[j].objID=pShortPredictPath->id;
      gLastObjIDandCurrentSeg[j].currentsegmenttype  =pSMap->segments_buffer[objCurrentSegMapIndex].segmenttype;
      gLastObjIDandCurrentSeg[j].currentsegmentnumber=pSMap->segments_buffer[objCurrentSegMapIndex].segmentnumber;
      gLastObjIDandCurrentSeg[j].currentsegmentmapindex=objCurrentSegMapIndex;
      existLastCurrentSegForTheObj=true;
      //save History current segment
      gLastObjIDandCurrentSeg[j].historyCurrentSegMapIndex.push_back(gLastObjIDandCurrentSeg[j].currentsegmentmapindex);
      for(int k=gLastObjIDandCurrentSeg[j].historyCurrentSegMapIndex.size()-1;k>=gBufferSizeForIntention;k--)
      {
	gLastObjIDandCurrentSeg[j].historyCurrentSegMapIndex.erase(gLastObjIDandCurrentSeg[j].historyCurrentSegMapIndex.begin()+k);
      }
      break;
    }
  }
  if(existLastCurrentSegForTheObj==false)
  {
    sCurSegAndGoalSeg tmpObj;
    tmpObj.objID=pShortPredictPath->id;
    tmpObj.currentsegmenttype=pSMap->segments_buffer[objCurrentSegMapIndex].segmenttype;
    tmpObj.currentsegmentnumber=pSMap->segments_buffer[objCurrentSegMapIndex].segmentnumber;
    tmpObj.currentsegmentmapindex=objCurrentSegMapIndex;
    tmpObj.finishedGoalSegment=false;
    gLastObjIDandCurrentSeg.push_back(tmpObj);
  }
  
  return &(pSMap->segments_buffer[objCurrentSegMapIndex]);
}

/**************************************************************
Function: FindSegmentData()
Author: lihui/zongwei
Date: 2017.6.11
Description: // 函数功能、性能等的描述
1. get all points of the segment.
2. -------
3. -------
Input:
1. type: type of target segment
2. number: number of target segment
3. pSMap: road segment map
Output: nothing
Return: target segment with all point coordinates
Others: // 其他说明
modified -------------lihui--------0628
************************************************************/
sSegRel FindSegmentData(string type, int number, sMapInformation *pSMap)
{
  sSegRel tempSegment;
  // for (int i = 0; i < pSMap->segments_buffer.size(); i++)
  {
      // if (pSMap->segments_buffer[i].segmenttype == type && pSMap->segments_buffer[i].segmentnumber == number)
      {
        tempSegment = pSMap->segments_buffer[number-1];
        // break;
      }
  }
  return tempSegment;
}

/**************************************************************
Function: FindParallelSegments()
Author: lihui/zongwei
Date: 2017.6.09
Description: // 函数功能、性能等的描述
1. find parallel segments with current segment
2. -------
3. -------
Input:
1. isParall: whether parallel or not
2. direction: direction for son segments
3. pCurrentSeg: pointer for current segment
4. pSMap: pointer for road segment map
// 用、取值说明及参数间关系。
Output: nothing
Return: local road segment map
Others: // 其他说明
//update the FindParallelSegments for new segment load ------lihui----0628
************************************************************/
sRoadSegRel FindParallelSegments(bool isParall, bool direction, sSegRel *pCurrentSeg, sMapInformation *pSMap)
{
  sRoadSegRel sOutputSegs;
  sSegRel sTempSeg;
  string currentSegType=pCurrentSeg->segmenttype;
  int    currentSegNum =pCurrentSeg->segmentnumber - 1;
  // cout << "current seg numb: " << currentSegNum << endl;
  // int 	 segsTotalNum  = pSMap->mysRelationship.list.size();

  if (isParall == true)
  {
    for (int j = 0; j < SEGSIZE; j++)
    {
      // cout << currentSegType << currentSegNum << endl;
      //cout << pSMap->mysRelationship.relationMatrix[i*(pSMap->mysRelationship.list.size()) + j] << endl;
      int temp_ID = pSMap->mysRelationship.list[currentSegNum].parallelID[j];
      // printf("id: %d\n", temp_ID);
      if (temp_ID != NORELATION)
      {
        string tempType = pSMap->mysRelationship.list[currentSegNum].segmentType;
        // int tempNum = pSMap->mysRelationship.list[currentSegNum].segmentNumber;
        int tempNum = temp_ID;//submit 1 in FindSegmentData function
        sTempSeg    = FindSegmentData(tempType,tempNum,pSMap);//submit 1 in this function
        sOutputSegs.segRels.push_back(sTempSeg);
  // cout<< sTempSeg.segmenttype << sTempSeg.segmentnumber <<endl;
       // cout<< sTempSeg.GCCS[0].xg << sTempSeg.GCCS[0].yg <<endl;
       // cout<< sTempSeg.GCCS[sTempSeg.GCCS.size()-1].xg << sTempSeg.GCCS[sTempSeg.GCCS.size()-1].yg <<endl;
      }
    }
  }
  else
  {
    if (direction == true)
    {    
      for (int j = 0; j < SEGSIZE; j++)
      {
        //direction
        int temp_ID = pSMap->mysRelationship.list[currentSegNum].fatherID[j];
        if (temp_ID != NORELATION)
        {
          float pointHeadDis, pointEndDis;
          float tempX, tempY;
          bool tempSonDir;
          //int findnumber;
          string tempType = pSMap->mysRelationship.list[currentSegNum].segmentType;
          int tempNum = temp_ID;
          sTempSeg = FindSegmentData(tempType,tempNum,pSMap);
          tempX = pCurrentSeg->GCCS[0].xg;
          tempY = pCurrentSeg->GCCS[0].yg;
          
          int tempEndNum = sTempSeg.GCCS.size()-1;

          pointHeadDis = sqrt( (tempX - sTempSeg.GCCS[0].xg)*(tempX - sTempSeg.GCCS[0].xg) + (tempY - sTempSeg.GCCS[0].yg)*(tempY - sTempSeg.GCCS[0].yg));
          pointEndDis = sqrt( (tempX - sTempSeg.GCCS[tempEndNum].xg)*(tempX - sTempSeg.GCCS[tempEndNum].xg) + (tempY - sTempSeg.GCCS[tempEndNum].yg)*(tempY - sTempSeg.GCCS[tempEndNum].yg));

          if (pointHeadDis > pointEndDis)
          {
            tempSonDir = true;
          }
          else
          {
            tempSonDir = false;
          }
          sTempSeg.isInvSearch = false;//tempSonDir;
          // sTempSeg.subSeg.push_back(sTempSeg); 
          sOutputSegs.segRels.push_back(sTempSeg);
        //  cout<< releventsegment.segmenttype << releventsegment.segmentnumber <<endl;
        } 
      }
    }
    else
    {
      for (int j = 0; j < SEGSIZE; j++)
      {
        //direction
        int temp_ID = pSMap->mysRelationship.list[currentSegNum].sonID[j];
        if (temp_ID != NORELATION)
        {
          float pointHeadDis, pointEndDis;
          float tempX, tempY;
          bool tempSonDir;
          //int findnumber;
          string tempType = pSMap->mysRelationship.list[j].segmentType;
          int tempNum = temp_ID;
           sTempSeg = FindSegmentData(tempType,tempNum,pSMap);

          tempX = pCurrentSeg->GCCS[pCurrentSeg->GCCS.size()-1].xg;
          tempY = pCurrentSeg->GCCS[pCurrentSeg->GCCS.size()-1].yg;
          
          int tempEndNum = sTempSeg.GCCS.size()-1;
          pointHeadDis = sqrt( (tempX - sTempSeg.GCCS[0].xg)*(tempX - sTempSeg.GCCS[0].xg) + (tempY - sTempSeg.GCCS[0].yg)*(tempY - sTempSeg.GCCS[0].yg));
          pointEndDis = sqrt( (tempX - sTempSeg.GCCS[tempEndNum].xg)*(tempX - sTempSeg.GCCS[tempEndNum].xg) + (tempY - sTempSeg.GCCS[tempEndNum].yg)*(tempY - sTempSeg.GCCS[tempEndNum].yg));
          if (pointHeadDis > pointEndDis)
          {
            tempSonDir = true;
          }
          else
          {
            tempSonDir = false;
          }
          sTempSeg.isInvSearch = false; //tempSonDir;
          // sTempSeg.subSeg.push_back(sTempSeg); 
          sOutputSegs.segRels.push_back(sTempSeg);
        //  cout<< releventsegment.segmenttype << releventsegment.segmentnumber <<endl;
        } 
      }
    }
  }
  return sOutputSegs;
}

/**************************************************************
Function: CalAngBetweenMoveAndCurrentSeg()
Author: lihui/zongwei
Date: 2017.6.09/2017.6.12
Description: // 函数功能、性能等的描述
1. calculate angel between moving direction and current segment
2. -------
3. -------
Input:
1. angObjMove: moving angle of the object
2. objNowXg, objNowYg: global coordinate of object. unit m
3. pCurrentSeg: pointer of current segment.
Output:
Return: angle between moving direction and current segment with unit degree.
Others: 
************************************************************/
float CalAngBetweenMoveAndCurrentSeg(float angObjMove, float objNowXg, float objNowYg, sSegRel *pCurrentSeg)
{
  int nearPointIndex=CalNearestSegmentPointIndex(objNowXg, objNowYg, pCurrentSeg); 
  
  float detaX2,detaY2;
  if(nearPointIndex==pCurrentSeg->GCCS.size()-1)
  {
    detaX2=pCurrentSeg->GCCS[nearPointIndex].xg - pCurrentSeg->GCCS[nearPointIndex-1].xg;
    detaY2=pCurrentSeg->GCCS[nearPointIndex].yg - pCurrentSeg->GCCS[nearPointIndex-1].yg;
  }
  else
  {
    detaX2=pCurrentSeg->GCCS[nearPointIndex+1].xg - pCurrentSeg->GCCS[nearPointIndex].xg;
    detaY2=pCurrentSeg->GCCS[nearPointIndex+1].yg - pCurrentSeg->GCCS[nearPointIndex].yg;
  }
  float angStoredSegment=atan2(detaY2, detaX2)*180/3.1415;
  float angle = (angStoredSegment - angObjMove)*3.1415/180;  //now range of angle is -2*pi<angle<2*pi.
  angle=atan2(abs(sin(angle)),abs(cos(angle)))*180/3.1415;  //now range of angle is 0<angle<90°.
  return angle;
}

/**************************************************************
Function: GetSubSeg()
Author: lihui/zongwei
Date: 2017.6.12
Description: // 函数功能、性能等的描述
1. find down level segments
2. -------
3. -------
Input:
1. tempDir: search direction
2. inputSeg: segment need to analyze
3. map: whole road segment map
Output:
Return:
1. son segments
Others:
************************************************************/
sSegRel GetSubSeg(sSegRel inputSeg, sMapInformation *map)
{
  sSegRel returnSeg = inputSeg;
  sRoadSegRel tempSonSeg = FindParallelSegments(false, inputSeg.isInvSearch, &inputSeg, map);
  //if (tempDir == false)
  {
    for (int i = 0; i < tempSonSeg.segRels.size(); i++)
    {
      returnSeg.subSeg.push_back(tempSonSeg.segRels[i]);
    }
  }
  return returnSeg;
}

std::vector<int> Find6NearObjs(ivpredict::ivpredictbuffer *pBufferObjInfo,sMapInformation *pSMap)
{
  std::vector<int> near6ObjID;
  typedef struct staticNearObj{
    double minY;
    double maxY;
    double minDisX;
    bool   existObj;
    int    objID;
  }staticNearObj;
  staticNearObj leftFrontObj,leftBackObj,midFrontObj,midBackObj,rightFrontObj,rightBackObj;
   leftFrontObj.existObj=false;
    leftBackObj.existObj=false;
    midFrontObj.existObj=false;
     midBackObj.existObj=false;
  rightFrontObj.existObj=false;
   rightBackObj.existObj=false;
   leftFrontObj.minDisX=1000;
    leftBackObj.minDisX=1000;
    midFrontObj.minDisX=1000;
     midBackObj.minDisX=1000;
  rightFrontObj.minDisX=1000;
   rightBackObj.minDisX=1000;
   
  //----------------------find segment that ego car is being on-----------------
  sSingleObjIDandSegID egoCarSegment=LocateEgoCarCurrentSegment(pSMap, gEgoCarPosXg,gEgoCarPosYg);
  
  // find nearest point index on egoCarSegment
  int pointIndex = CalNearestSegmentPointIndex(gEgoCarPosXg, gEgoCarPosYg, &(pSMap->segments_buffer[egoCarSegment.segmentmapindex]));
  
  //calculate 
  double x1,y1,x2,y2,vectorRoadX,vectorRoadY,vectorEgoX,vectorEgoY;
  if(pointIndex < pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS.size()-2)
  {
    x1=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex].xg;
    y1=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex].yg;
    x2=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex+1].xg;
    y2=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex+1].yg;
  }
  else
  {
    x1=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex-1].xg;
    y1=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex-1].yg;
    x2=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex].xg;
    y2=pSMap->segments_buffer[egoCarSegment.segmentmapindex].GCCS[pointIndex].yg;
  }
  vectorRoadX=x2-x1;
  vectorRoadY=y2-y1;
  vectorEgoX=gEgoCarPosXg-x2;
  vectorEgoY=gEgoCarPosYg-y2;
  
  //calculate
  double cross=vectorRoadX*vectorEgoY-vectorRoadY*vectorEgoX;
  double dis=fabs(cross)/sqrt(vectorRoadX*vectorRoadX+vectorRoadY*vectorRoadY);
  if(cross>0)
  {
     leftFrontObj.minY= leftBackObj.minY=midFrontObj.maxY=midBackObj.maxY=-dis+3.75/2;
    rightFrontObj.maxY=rightBackObj.maxY=midFrontObj.minY=midBackObj.minY=-dis-3.75/2;
  }
  else
  {
     leftFrontObj.minY= leftBackObj.minY=midFrontObj.maxY=midBackObj.maxY=dis+3.75/2;
    rightFrontObj.maxY=rightBackObj.maxY=midFrontObj.minY=midBackObj.minY=dis-3.75/2;
  }
   leftFrontObj.maxY= leftBackObj.maxY= leftFrontObj.minY+3.75;
  rightFrontObj.minY=rightBackObj.minY=rightFrontObj.maxY-3.75;
  
  //find nearest objects
  for (int i = pBufferObjInfo->objects.size()-1; i>=0; i--)
  {
    if(pBufferObjInfo->objects[i].id==-88)
    {
      continue;
    }
    
    int posSize=pBufferObjInfo->objects[i].positions.size();
    double xrel=pBufferObjInfo->objects[i].positions[posSize-1].xmean;
    double yrel=pBufferObjInfo->objects[i].positions[posSize-1].ymean;
    if(yrel<=leftFrontObj.maxY && yrel>=leftFrontObj.minY && xrel>=0 && fabs(xrel)<=leftFrontObj.minDisX)
    {
      leftFrontObj.existObj=true;
      leftFrontObj.minDisX =fabs(xrel);
      leftFrontObj.objID=pBufferObjInfo->objects[i].id;
    }
    else if (yrel<=leftBackObj.maxY && yrel>=leftBackObj.minY && xrel<=0 && fabs(xrel)<=leftBackObj.minDisX)
    {
      leftBackObj.existObj=true;
      leftBackObj.minDisX =fabs(xrel);
      leftBackObj.objID=pBufferObjInfo->objects[i].id;
    }
    else if (yrel<=midFrontObj.maxY && yrel>=midFrontObj.minY && xrel>=0 && fabs(xrel)<=midFrontObj.minDisX)
    {
      midFrontObj.existObj=true;
      midFrontObj.minDisX =fabs(xrel);
      midFrontObj.objID=pBufferObjInfo->objects[i].id;
    }
    else if (yrel<=midBackObj.maxY && yrel>=midBackObj.minY && xrel<=0 && fabs(xrel)<=midBackObj.minDisX)
    {
      midBackObj.existObj=true;
      midBackObj.minDisX =fabs(xrel);
      midBackObj.objID=pBufferObjInfo->objects[i].id;
    }
    else if (yrel<=rightFrontObj.maxY && yrel>=rightFrontObj.minY && xrel>=0 && fabs(xrel)<=rightFrontObj.minDisX)
    {
      rightFrontObj.existObj=true;
      rightFrontObj.minDisX =fabs(xrel);
      rightFrontObj.objID=pBufferObjInfo->objects[i].id;
    }
    else if (yrel<=rightBackObj.maxY && yrel>=rightBackObj.minY && xrel<=0 && fabs(xrel)<=rightBackObj.minDisX)
    {
      rightBackObj.existObj=true;
      rightBackObj.minDisX =fabs(xrel);
      rightBackObj.objID=pBufferObjInfo->objects[i].id;
    }
  }
    
    //save id
   if(leftFrontObj.existObj==true) 	near6ObjID.push_back(leftFrontObj.objID);
   if( leftBackObj.existObj==true) 	near6ObjID.push_back( leftBackObj.objID);
   if( midFrontObj.existObj==true) 	near6ObjID.push_back( midFrontObj.objID);
   if(  midBackObj.existObj==true) 	near6ObjID.push_back(  midBackObj.objID);
   if(rightFrontObj.existObj==true) 	near6ObjID.push_back(rightFrontObj.objID);
   if( rightBackObj.existObj==true) 	near6ObjID.push_back( rightBackObj.objID);
   return near6ObjID;
}

/**************************************************************
Function: LocateObj()
Author: lihui/zongwei
Date: 2017.6.09
Description: // 函数功能、性能等的描述
1. locating object,that is, find local road segment map for the concerned object. 
2. -------
3. -------
Input:
1. pShortPredictPath: pointer for predicted short term path
2. pSMap: pointer for whole road segment map
Output:
Return: local road segment map
Others: 
modified -------------lihui--------0628---add subseg
modified -------------zongwei------0714---revise method
************************************************************/
sRoadSegRel LocateObj(ivpredict::predictobj *pShortPredictPath, sMapInformation *pSMap, ivpredict::objbuffer *pObj)
{
  //----------------------find segment that ego car is being on-----------------
  sSingleObjIDandSegID egoCarSegment=LocateEgoCarCurrentSegment(pSMap, gEgoCarPosXg,gEgoCarPosYg);  
  //----------------------find current segment for the object-------------------------
  int  objPosSize=pObj->positions.size();
  double objNowXg=pObj->positions[objPosSize-1].xgmean;
  double objNowYg=pObj->positions[objPosSize-1].ygmean;
  sSegRel *pSCurrentSegment = LocateObjCurrentSeg(pShortPredictPath, pSMap, objNowXg, objNowYg);
  gMidVarialblesRecord.pSCurrentSegment=pSCurrentSegment;   
  //----------------------find parallel segment(multi lines) for the object-----------
  bool flagFindParallel=true;
  sRoadSegRel nowRoadSegRel = FindParallelSegments(flagFindParallel, false, pSCurrentSegment, pSMap); 
  nowRoadSegRel.objID = pShortPredictPath->id;
  nowRoadSegRel.currentSegmentType  =pSCurrentSegment->segmenttype;
  nowRoadSegRel.currentSegmentNumber=pSCurrentSegment->segmentnumber;   
  
  //----------------------------get nowRoadSegRel (new method)---------------------------
  //-------0. set key parameters-------
  double    errorWidth=0.4;
  double halfLaneWidthForObj=3.75/2;
  double halfLaneWidthForEgoCar=3.75/2;
  bool findoutOneSpecialSeg=false;
  int goalSegIndexInNowRegSel=-1; 
  int  curSegIndexInNowRegSel=-1;
  int     tmpLastGoalSegIndex=-1;
  double    crDis_t=3.75/3;  

  //-------0.1 revise parameter halfLaneWidthForObj--------
  if(nowRoadSegRel.segRels.size()>1)
  {
    float closedDis = 1000;
    float secondClosedDis=1000;
    int   closedDisSegIndex=-1;
    int   secondDisSegIndex=-1;
    std::vector<float> disArray;
    for (int i = 0; i < nowRoadSegRel.segRels.size(); i++)
    {
      float distance= CalPoint2SegmentDis(objNowXg, objNowYg, &(nowRoadSegRel.segRels[i]));
      if (distance < closedDis)
      {
	closedDis = distance;
	closedDisSegIndex=i;
      }
      disArray.push_back(distance);
    }
    for (int i = 0; i < nowRoadSegRel.segRels.size(); i++)
    {
      if(disArray[i]<secondClosedDis && i!=closedDisSegIndex)
      {
	secondClosedDis=disArray[i];
	secondDisSegIndex=i;
      }
    }
    int pointIndex1 = CalNearestSegmentPointIndex(objNowXg, objNowYg, &(nowRoadSegRel.segRels[closedDisSegIndex]));
    int pointIndex2 = CalNearestSegmentPointIndex(objNowXg, objNowYg, &(nowRoadSegRel.segRels[secondDisSegIndex]));    
    double x1=nowRoadSegRel.segRels[closedDisSegIndex].GCCS[pointIndex1].xg;
    double y1=nowRoadSegRel.segRels[closedDisSegIndex].GCCS[pointIndex1].yg;
    double x2=nowRoadSegRel.segRels[secondDisSegIndex].GCCS[pointIndex2].xg;
    double y2=nowRoadSegRel.segRels[secondDisSegIndex].GCCS[pointIndex2].yg;    
    double ang=atan2(y1-objNowYg,x1-objNowXg)-atan2(y2-objNowYg,x2-objNowXg);
    if(cos(ang)<0 && closedDis+secondClosedDis<3.75+1.5)
    {
      halfLaneWidthForObj=(closedDis+secondClosedDis)/2;
    }
    else
    {
      halfLaneWidthForObj=3.75/2;
    }
  }
  else
  {
    halfLaneWidthForObj=3.75/2;
  }
 
  //-------0.2 revise parameter halfLaneWidthForEgoCar--------
  halfLaneWidthForEgoCar=gHalfLaneWidthForEgoCar;
  
  //-------1. calculate distance between object and egocar's segment-----------
  double disBetweenObjAndEgocarSeg=CalPoint2SegmentDis(objNowXg,objNowYg,&(pSMap->segments_buffer[egoCarSegment.segmentmapindex]));    

  //-------1.0. calculate curSegIndexInNowRegSel----
  for (int k = nowRoadSegRel.segRels.size()-1; k >=0; k--)
  {
    if(nowRoadSegRel.segRels[k].segmenttype==nowRoadSegRel.currentSegmentType && nowRoadSegRel.segRels[k].segmentnumber==nowRoadSegRel.currentSegmentNumber)
    {
      curSegIndexInNowRegSel=k;
      break;
    }
  }
  for(int j=gLastObjIDandCurrentSeg.size()-1;j>=0;j--)
  {
    if(gLastObjIDandCurrentSeg[j].objID==pShortPredictPath->id)
    {
      tmpLastGoalSegIndex=j;
      break;
    }
  }

  //-------1.1. calculate heading of the object----
  int pointIndex=CalNearestSegmentPointIndex(objNowXg, objNowYg, &(nowRoadSegRel.segRels[curSegIndexInNowRegSel]));
  if(pointIndex<nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS.size()-4)
  {
    double tmpDx=nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex+4].xg-nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex].xg;
    double tmpDy=nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex+4].yg-nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex].yg;
    nowRoadSegRel.objHeading=atan2(tmpDx,tmpDy);
  }
  else
  {
    double tmpDx=nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex].xg-nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex-4].xg;
    double tmpDy=nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex].yg-nowRoadSegRel.segRels[curSegIndexInNowRegSel].GCCS[pointIndex-4].yg;
    nowRoadSegRel.objHeading=atan2(tmpDx,tmpDy);
  }

//cout<<"------------id="<<nowRoadSegRel.objID<<"--------------"<<endl;
  //-------1.2. if distance is too small-----------
  if(disBetweenObjAndEgocarSeg<=halfLaneWidthForEgoCar-errorWidth*0.5)
  {
    for(int i = nowRoadSegRel.segRels.size()-1; i >=0; i--)
    {
      if(nowRoadSegRel.segRels[i].segmenttype==egoCarSegment.segmenttype && nowRoadSegRel.segRels[i].segmentnumber==egoCarSegment.segmentnumber)
      {
	goalSegIndexInNowRegSel=i;
	findoutOneSpecialSeg=true;
	break;
      }
    }
    if(findoutOneSpecialSeg==false)
    {
      goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
      findoutOneSpecialSeg=true;
    }
//cout<<"**1"<<": Index="<<tmpLastGoalSegIndex<<"; goIndex="<<goalSegIndexInNowRegSel<<endl;
  }  
  //-------1.3. if above distance is in a special range,then take last segment-----------
  else if(disBetweenObjAndEgocarSeg>halfLaneWidthForEgoCar-errorWidth*0.5 && disBetweenObjAndEgocarSeg<=halfLaneWidthForEgoCar+errorWidth)
  {
    if(gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].finishedGoalSegment==true)
    {
      for (int k = nowRoadSegRel.segRels.size()-1; k >=0; k--)
      {
	if(nowRoadSegRel.segRels[k].segmenttype==gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].goalsegmenttype && nowRoadSegRel.segRels[k].segmentnumber==gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].goalsegmentnumber)
	{
	  goalSegIndexInNowRegSel=k;
	  findoutOneSpecialSeg=true;
	  break;
	}
      }
      if(findoutOneSpecialSeg==false)
      {
	goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
	findoutOneSpecialSeg=true;
      }
	
//cout<<"**2"<<": Index="<<tmpLastGoalSegIndex<<"; goIndex="<<goalSegIndexInNowRegSel<<endl;
    }
    else if(gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].finishedGoalSegment==false)
    {
//cout<<"**3"<<endl;
      goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
      findoutOneSpecialSeg=true;      
      gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].finishedGoalSegment=true;
      gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].goalsegmenttype  =nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmenttype;
      gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].goalsegmentnumber=nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmentnumber;
    }
  }  
  //------1.4 if above distance is too far----------
  else if(disBetweenObjAndEgocarSeg>halfLaneWidthForEgoCar+errorWidth)
  {
    if(gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].historyCurrentSegMapIndex.size()>=gBufferSizeForIntention)
    {
      double disObj2CurSeg=CalPoint2SegmentDis(objNowXg,objNowYg,&(pSMap->segments_buffer[gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].currentsegmentmapindex]));     
      if(disObj2CurSeg<=halfLaneWidthForObj-errorWidth)
      {
	goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
	findoutOneSpecialSeg=true;
//cout<<"**5"<<endl;
      }
      else if(disObj2CurSeg>halfLaneWidthForObj-errorWidth && disObj2CurSeg<=halfLaneWidthForObj+errorWidth)
      {    
//cout<<"**6"<<endl;
	double disMin=10000;
	double startX=pObj->positions[objPosSize-gBufferSizeForIntention].xgmean;
	double startY=pObj->positions[objPosSize-gBufferSizeForIntention].ygmean;
	double   endX=pObj->positions[objPosSize-1].xgmean;
	double   endY=pObj->positions[objPosSize-1].ygmean;	
	for(int k=nowRoadSegRel.segRels.size()-1;k>=0;k--)
	{
	  double disStart=CalPoint2SegmentDis(startX,startY,&nowRoadSegRel.segRels[k]);
	  double disEnd  =CalPoint2SegmentDis(endX,endY,&nowRoadSegRel.segRels[k]);
//cout<<"**disStart="<<disStart<<"; disEnd="<<disEnd<<"; crDis_t="<<crDis_t<<endl;
	  if(disStart-disEnd>crDis_t && disEnd<halfLaneWidthForObj && disEnd<disMin)
	  {
	    goalSegIndexInNowRegSel=k;
	    findoutOneSpecialSeg=true;
	    disMin=disEnd;
//cout<<"**7"<<": dis="<<disMin<<endl;
	  }
	}
	if(findoutOneSpecialSeg==false)
	{
//cout<<"**8"<<endl;
	  goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
	  findoutOneSpecialSeg=true; 
	}
      }
      else if(disObj2CurSeg>halfLaneWidthForObj+errorWidth)
      {
//cout<<"**9"<<endl;
	goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
	findoutOneSpecialSeg=true; 
      }
    }
    else if(gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].historyCurrentSegMapIndex.size()<gBufferSizeForIntention)
    {
//cout<<"**10"<<endl;
      goalSegIndexInNowRegSel=curSegIndexInNowRegSel;
      findoutOneSpecialSeg=true; 
    }
  }
//cout<<"curSeg="<<gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].currentsegmenttype<<gLastObjIDandCurrentSeg[tmpLastGoalSegIndex].currentsegmentnumber;
//cout<<"; goSeg="<<nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmenttype<<nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmentnumber;
//cout<<"; ecarSeg="<<egoCarSegment.segmenttype<<egoCarSegment.segmentnumber<<endl;
  
  //------------Calulate Angle Between Moving direction And Current Segment---------
  int shortPathPointsSize = pShortPredictPath->positions.size();  
  double vectorObjMoveY=pShortPredictPath->positions[2].ygmean - pShortPredictPath->positions[0].ygmean;
  double vectorObjMoveX=pShortPredictPath->positions[2].xgmean - pShortPredictPath->positions[0].xgmean;
  double angObjMove = atan2(vectorObjMoveY,vectorObjMoveX)*180/3.1415;

  //1.5 remove unrelevant segments and save gLastObjIDandCurrentSeg
  if(findoutOneSpecialSeg==true)
  {
    for(int j=gLastObjIDandCurrentSeg.size()-1;j>=0;j--)
    {
      if(gLastObjIDandCurrentSeg[j].objID==pShortPredictPath->id)
      {
	gLastObjIDandCurrentSeg[j].finishedGoalSegment=true;
	gLastObjIDandCurrentSeg[j].goalsegmenttype  =nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmenttype;
	gLastObjIDandCurrentSeg[j].goalsegmentnumber=nowRoadSegRel.segRels[goalSegIndexInNowRegSel].segmentnumber;
	break;
      }
    }  
    for (int i = nowRoadSegRel.segRels.size()-1; i >=0; i--)
    {
      if(i!=goalSegIndexInNowRegSel)    {nowRoadSegRel.segRels.erase(nowRoadSegRel.segRels.begin()+i);}
    }    
  }

  //1.6 analyze search direction for the segment
  for (int i = nowRoadSegRel.segRels.size()-1; i >=0; i--)
  {
    //find the nearest point index on target segment.
    int nearPointIndex=CalNearestSegmentPointIndex(pShortPredictPath->positions[0].xgmean, pShortPredictPath->positions[0].ygmean, &(nowRoadSegRel.segRels[i]));
    //analyze search direction for the segment
    double angStoredSegment;
    if(nearPointIndex==nowRoadSegRel.segRels[i].GCCS.size()-1)
    {
      double detaX2=nowRoadSegRel.segRels[i].GCCS[nearPointIndex].xg - nowRoadSegRel.segRels[i].GCCS[nearPointIndex-1].xg;
      double detaY2=nowRoadSegRel.segRels[i].GCCS[nearPointIndex].yg - nowRoadSegRel.segRels[i].GCCS[nearPointIndex-1].yg;
      angStoredSegment=atan2(detaY2, detaX2)*180/3.1415;
    }
    else
    {
      double detaX2=nowRoadSegRel.segRels[i].GCCS[nearPointIndex+1].xg - nowRoadSegRel.segRels[i].GCCS[nearPointIndex].xg;
      double detaY2=nowRoadSegRel.segRels[i].GCCS[nearPointIndex+1].yg - nowRoadSegRel.segRels[i].GCCS[nearPointIndex].yg;
      angStoredSegment=atan2(detaY2, detaX2)*180/3.1415;
    }
    double angle = (angStoredSegment - angObjMove)*3.1415/180;  	//now range of angle is -2*pi<angle<2*pi.
    angle=atan2(sin(angle),cos(angle));  			 //now range of angle is   -pi<angle<pi.
    bool isInvSearch = (abs(angle) > 3.1415/2)?true:false;
    nowRoadSegRel.segRels[i].isInvSearch = isInvSearch;
    sSegRel temp_parallSeg = GetSubSeg(nowRoadSegRel.segRels[i], pSMap);
    nowRoadSegRel.segRels[i]=temp_parallSeg;
  }

  //----------------------------return-----------------------------
  //cout<<nowRoadSegRel.segRels.size()<<endl;
  return nowRoadSegRel;
}

#endif
