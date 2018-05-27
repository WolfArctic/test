/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: matchpath.h
Description:
1. this document realizes shortlong path prediction.
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年03月17日	V1.1		build this module
Zongwei		2017年06月26日	V1.2		re-orgnize this module
************************************************************/

#ifndef _MATCHPATH_H
#define _MATCHPATH_H
#pragma once

#include "../globaldefine.h"
#include "../glovariable.h"
#include "../common/commonfunctions.h"
#include "locateobject.h"
#include "analyzeintention.h"
#include "analyzeprobability.h"

/**************************************************************
Function: ClearSavedSegNearPoints()
Author: zongwei
Date: 2017.4.21
Description: // 函数功能、性能等的描述
1. clear global varialbes about saved near points for each segment and object.
2. -------
3. -------
Input:
Output:
Return:
Others: // 其他说明
************************************************************/
void ClearSavedSegNearPoints()
{
  int savedObjNum=gSavedSegNearPoints.size(); 
  int actualObjNum=gAllObjIDandSegID.size();
  for(int i=0;i<savedObjNum;i++)
  {  
    bool existThisObj=false;  
    for(int j=0;j<actualObjNum;j++)
    {  
      if (gSavedSegNearPoints[i].objID==gAllObjIDandSegID[j].objID && gSavedSegNearPoints[i].segmenttype==gAllObjIDandSegID[j].segmenttype && gSavedSegNearPoints[i].segmentnumber==gAllObjIDandSegID[j].segmentnumber)
      {
      	existThisObj=true;
      	break;
      }
    }
    
    if (existThisObj==false)
    {
      gSavedSegNearPoints.erase(gSavedSegNearPoints.begin()+i);
      i--;
      savedObjNum--;
    }
  }
}

void ClearLastObjIDandCurrentSeg()
{
  int savedObjNum=gLastObjIDandCurrentSeg.size(); 
  int actualObjNum=gAllObjIDandSegID.size();
  for(int i=savedObjNum-1; i>=0; i--)
  {
    bool existThisObj=false;  
    for(int j=0;j<actualObjNum;j++)
    {  
      if (gLastObjIDandCurrentSeg[i].objID==gAllObjIDandSegID[j].objID)
      {
      	existThisObj=true;
      	break;
      }
    }
    
    if (existThisObj==false)
    {
      gLastObjIDandCurrentSeg.erase(gLastObjIDandCurrentSeg.begin()+i);
    }
  }
}
/**************************************************************
Function: FindCrossPointLineNum()
Author: zongwei
Date: 2017.6.15
Description: // 函数功能、性能等的描述
1. find cross point between moving direction and corresponding segment.
2. -------
3. -------
Input:
1. isInvSearch: whether inverse search or not
2. initFindLineNum: the starting searching point index about target segment
3. pArrayAddr: pointer for target segment.
4. maxArrayLineNum: total point number for target segment.
5. pShortTermPath: pointer for short term predicted path
Output:
Return:
1. found point index
Others: // 其他说明
************************************************************/
int FindCrossPointLineNum(bool isInvSearch, int initFindLineNum, sGCCSpoint *pArrayAddr, int maxArrayLineNum,ivpredict::predictobj *pShortTermPath)
{
  //---------definition-------------
  int index0,indexN,indexMid;
  int shortSize=pShortTermPath->positions.size();
  float xgmean=pShortTermPath->positions[shortSize-1].xgmean;
  float ygmean=pShortTermPath->positions[shortSize-1].ygmean;
  float angObjMove=atan2(ygmean-(pShortTermPath->positions[shortSize-2].ygmean), xgmean-(pShortTermPath->positions[shortSize-2].xgmean));
  
  //---------initialization---------
  if(isInvSearch==true)
  {
    index0=0;
    indexN=initFindLineNum;
  }
  else
  {
    index0=initFindLineNum;
    indexN=maxArrayLineNum-1;
  }

  //----------start calculation------------
  float angArray0  =atan2(pArrayAddr[index0].yg-ygmean, pArrayAddr[index0].xg-xgmean);
  float angArrayN  =atan2(pArrayAddr[indexN].yg-ygmean, pArrayAddr[indexN].xg-xgmean);
  float angArrayMid;
  if(sin(angArray0-angObjMove)*sin(angArrayN-angObjMove)<0)
  {
    while(1)
    {
      indexMid=round((index0+indexN)/2.0);
      angArrayMid=atan2(pArrayAddr[indexMid].yg-ygmean, pArrayAddr[indexMid].xg-xgmean);
      if(index0==indexMid || indexN==indexN)
      {
	break;
      }
      else if(sin(angArray0-angObjMove)*sin(angArrayMid-angObjMove)>0)
      {
	index0=indexMid;
	angArray0=angArrayMid;
      }
      else if(sin(angArrayN-angObjMove)*sin(angArrayMid-angObjMove)>0)
      {
	indexN=indexMid;
	angArrayN=angArrayMid;
      }
    }
  }
  else
  {
    float dis0=sqrt(pow(pArrayAddr[index0].xg-xgmean,2)+pow(pArrayAddr[index0].yg-ygmean,2));
    float disN=sqrt(pow(pArrayAddr[indexN].xg-xgmean,2)+pow(pArrayAddr[indexN].yg-ygmean,2));
    if(dis0<disN)
    {
      indexMid=index0;
    }
    else
    {
      indexMid=indexN;
    }
  }
  return indexMid;
}


int FindCertainDisSegIndex(double xg, double yg, double moveVectorX, double moveVectorY, sSegRel *pTmpSegment, double centainDis)
{
  int goalPointIndex=-1;
  
  //calculate nearest segment point
  int nearestSegPointIndex=CalNearestSegmentPointIndex(xg, yg, pTmpSegment);
  
  //calculate nearest segment point
  int dataNum=(int)((pTmpSegment->GCCS).size());
  double disSum=0;
  switch(pTmpSegment->isInvSearch)
  {
    case true:
      for(int j = nearestSegPointIndex-1; j>=0; j--)
      {
	double detaX=pTmpSegment->GCCS[j].xg-pTmpSegment->GCCS[j+1].xg;
	double detaY=pTmpSegment->GCCS[j].yg-pTmpSegment->GCCS[j+1].yg;
	disSum+=sqrt(detaX*detaX + detaY*detaY);
	if(disSum>=centainDis) 
	{
	  goalPointIndex=j;
	  break;
	}
      }
      break;
    case false:
      for(int j = nearestSegPointIndex+1; j<dataNum; j++)
      {
	double detaX=pTmpSegment->GCCS[j+1].xg-pTmpSegment->GCCS[j].xg;
	double detaY=pTmpSegment->GCCS[j+1].yg-pTmpSegment->GCCS[j].yg;
	disSum+=sqrt(detaX*detaX + detaY*detaY);
	if(disSum>=centainDis) 
	{
	  goalPointIndex=j;
	  break;
	}
      }
      break;
    default:
      break;
  }

  //return
  return goalPointIndex;
}


/**************************************************************
Function: FindNearPointLineNum()
Author: zongwei
Date: 2017.6.15
Description: // 函数功能、性能等的描述
1. find nearest point index on target segment.
2. -------
3. -------
Input:
1. isInvSearch: whether inverse search or not
2. initFindLineNum: the starting searching point index about target segment
3. pArrayAddr: pointer for target segment.
4. maxArrayLineNum: total point number for target segment.
5. xgmean,ygmean: coordinate of the point.
Output:
Return:
1. nearest point index on target segment
Others: // 其他说明
************************************************************/
int FindNearPointLineNum(bool isInvSearch, int initFindLineNum, sGCCSpoint *pArrayAddr, int maxArrayLineNum,double xgmean,double ygmean)
{
  //----init key parameters-----------
  double minimumDis=15.0;
  //-------start calculation---------
//   if (isInvSearch==false)
//   {
//     for(int j=initFindLineNum; j<=maxArrayLineNum-2; j++)
//     {
//       double tmpDetaX0,tmpDetaY0;
//       if (j==0)
//       {
//       	tmpDetaX0=pArrayAddr[j+2].xg-xgmean;
//       	tmpDetaY0=pArrayAddr[j+2].yg-ygmean;
//       }
//       else if (j>=1)
//       {
//       	tmpDetaX0=pArrayAddr[j-1].xg-xgmean;
//       	tmpDetaY0=pArrayAddr[j-1].yg-ygmean;
//       }
//       double dis0=sqrt(tmpDetaX0*tmpDetaX0+tmpDetaY0*tmpDetaY0);
//       double tmpDetaX1=pArrayAddr[j].xg-xgmean;
//       double tmpDetaY1=pArrayAddr[j].yg-ygmean;
//       double 	  dis1=sqrt(tmpDetaX1*tmpDetaX1+tmpDetaY1*tmpDetaY1);
//       double tmpDetaX2=pArrayAddr[j+1].xg-xgmean;
//       double tmpDetaY2=pArrayAddr[j+1].yg-ygmean;
//       double 	  dis2=sqrt(tmpDetaX2*tmpDetaX2+tmpDetaY2*tmpDetaY2);
//       if (j==0 && dis1<dis2 && dis2<dis0 && dis1<minimumDis)
//       {
// 	return j;
//       }
//       else if (j>=1 && dis0>dis1 && dis1<dis2 && dis1<minimumDis)
//       {
// 	return j;
//       }
//       else if (dis0>=dis1 && dis1>=dis2 && j==(maxArrayLineNum-2))
//       {
// 	return (maxArrayLineNum-1);
//       }
//     }
//   }
//   else if (isInvSearch==true)
//   {
//     for(int j=initFindLineNum; j>=1; j--)
//     {     
//       double tmpDetaX0,tmpDetaY0;
//       if (j==maxArrayLineNum-1)
//       {
// 	tmpDetaX0=pArrayAddr[j-2].xg-xgmean;
// 	tmpDetaY0=pArrayAddr[j-2].yg-ygmean;
//       }
//       else
//       {
// 	tmpDetaX0=pArrayAddr[j+1].xg-xgmean;
// 	tmpDetaY0=pArrayAddr[j+1].yg-ygmean;
//       }    
//       double 	  dis0=sqrt(tmpDetaX0*tmpDetaX0+tmpDetaY0*tmpDetaY0);     
//       double tmpDetaX1=pArrayAddr[j].xg-xgmean;
//       double tmpDetaY1=pArrayAddr[j].yg-ygmean;
//       double 	  dis1=sqrt(tmpDetaX1*tmpDetaX1+tmpDetaY1*tmpDetaY1);	     
//       double tmpDetaX2=pArrayAddr[j-1].xg-xgmean;
//       double tmpDetaY2=pArrayAddr[j-1].yg-ygmean;
//       double 	  dis2=sqrt(tmpDetaX2*tmpDetaX2+tmpDetaY2*tmpDetaY2);
//       if (j==(maxArrayLineNum-1) && dis0>dis2 && dis2>=dis1 && dis1<minimumDis)
//       {
// 	return j;
//       }
//       else if (j<=(maxArrayLineNum-2) && dis2>dis1 && dis1<dis0 && dis1<minimumDis)
//       {
// 	return j;
//       }
//       else if (j==1 && dis2<=dis1)
//       {
// 	return 0;
//       }
//     }
//   }
//   return initFindLineNum;
//   
  
  //-----------------------
  float pointToSegmentDis = 10000;
  int step=10;
  int tempIndex=-1;
  for (int j = 0; j <=maxArrayLineNum; j = j+step)
  {
    float tempDis = sqrt(pow(pArrayAddr[j].xg-xgmean, 2) +  pow(pArrayAddr[j].yg-ygmean, 2)) ;
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      tempIndex=j;
    }
  }
  
  int startNum=(tempIndex-step)>0 ? tempIndex-step : 0;
  int   endNum=(tempIndex+step)<=maxArrayLineNum ? tempIndex+step : maxArrayLineNum;
  for (int j = startNum; j <= endNum; j++)
  {
    float tempDis = sqrt(pow(pArrayAddr[j].xg-xgmean, 2) +  pow(pArrayAddr[j].yg-ygmean, 2)) ;
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      tempIndex=j;
    }
  }
  
  return tempIndex;
}

/**************************************************************
Function: UpdateSavedSegNearPoints()
Author: zongwei
Date: 2017.4.19
Description: // 函数功能、性能等的描述
1. update global varialbes about nearest segment points.
2. -------
3. -------
Input:
1. pNowRoadSegRel: pointer for local road segment map
2. nowXg,nowYg: now coordinate of the object.unit m
Output: // 对输出参数的说明
Return: // 对函数返回值的说明
Others: // 其他说明
************************************************************/
void UpdateSavedSegNearPoints(sRoadSegRel *pNowRoadSegRel,double nowXg,double nowYg)
{
  int objID=pNowRoadSegRel->objID;
  int topLevelNum=pNowRoadSegRel->segRels.size();  
  for(int i=0;i<topLevelNum;i++)
  {
    int storedSegPointNum=gSavedSegNearPoints.size();
    bool existSavedSeg=false;    
    for(int j=0;j<storedSegPointNum;j++)
    {
      if (gSavedSegNearPoints[j].objID==objID && gSavedSegNearPoints[j].segmenttype==pNowRoadSegRel->segRels[i].segmenttype && gSavedSegNearPoints[j].segmentnumber==pNowRoadSegRel->segRels[i].segmentnumber)
      {  
	int tmpNearLineNum=gSavedSegNearPoints[j].pLineNum;
	int maxNum=pNowRoadSegRel->segRels[i].maxLineNum;
	bool isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
	gSavedSegNearPoints[j].maxLineNum=pNowRoadSegRel->segRels[i].maxLineNum;
	gSavedSegNearPoints[j].segmenttype=pNowRoadSegRel->segRels[i].segmenttype;
	gSavedSegNearPoints[j].segmentnumber=pNowRoadSegRel->segRels[i].segmentnumber;
	gSavedSegNearPoints[j].pLineNum=FindNearPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]),maxNum,nowXg,nowYg);
	existSavedSeg=true;
	break;
      }
    }
    if (existSavedSeg==false)
    {  
      sSingleSegNearPoint tmpPoint;
      tmpPoint.objID     =objID;	
      tmpPoint.segmenttype  =pNowRoadSegRel->segRels[i].segmenttype;
      tmpPoint.segmentnumber=pNowRoadSegRel->segRels[i].segmentnumber;
      tmpPoint.maxLineNum=pNowRoadSegRel->segRels[i].maxLineNum;
      if (pNowRoadSegRel->segRels[i].isInvSearch==false)
      {
	tmpPoint.pLineNum=FindNearPointLineNum(false,0,&(pNowRoadSegRel->segRels[i].GCCS[0]),tmpPoint.maxLineNum,nowXg,nowYg);
      }
      else
      {
	tmpPoint.pLineNum=FindNearPointLineNum(true,tmpPoint.maxLineNum-1,&(pNowRoadSegRel->segRels[i].GCCS[0]),tmpPoint.maxLineNum,nowXg,nowYg);
      }
//       tmpPoint.lastcurrentsegmenttype    =pNowRoadSegRel->currentSegmentType;
//       tmpPoint.lastcurrentsegmentnumber  =pNowRoadSegRel->currentSegmentNumber;
//       tmpPoint.lastcurrentsegmentmapindex=pNowRoadSegRel->currentSegmentMapindex;
      gSavedSegNearPoints.push_back(tmpPoint);
    }
  }
}

/**************************************************************
Function: MatchChangeSegLongTermPath()
Author: zongwei
Date: 2017.4.19
Description: // 函数功能、性能等的描述
1. match long term path for intention of changing segment
2. -------
3. -------
Input:
1. pObj: pointer for object buffer
2. pShortTermPath: pointer for short term path
3. pNowRoadSegRel: pointer for local road segment map
// 用、取值说明及参数间关系。
Output: // 对输出参数的说明
Return: matched long term paths
Others: // 其他说明
************************************************************/
std::vector<ivpredict::predictobj> MatchChangeSegLongTermPath(ivpredict::objbuffer *pObj, ivpredict::predictobj *pShortTermPath, sRoadSegRel *pNowRoadSegRel,double timeLength)
{
  std::vector<ivpredict::predictobj> matchedPaths;
  ivpredict::predictobj tmpPrePath = *pShortTermPath;
  matchedPaths.clear();
  
  //-----------------------------step 0: set initial key parameters--------------------------------
  double objVel=pShortTermPath->v_abs; 	   //unit:m/s.
  double disLongTermStart=objVel*3;
  double totalDis;   			   //unit:m
  switch(gSetTimeMode)
  {
    case true:
      totalDis=objVel*timeLength*1.2;
      break;
    case false:
      totalDis=gPathDisSet;
      break;
    default:
      break;
  }
  int objID=pNowRoadSegRel->objID;
  
  //---------------step 1: find nearest point on each segment and store it in gSavedSegNearPoints--------------
  int objPosNum=pObj->positions.size();
  UpdateSavedSegNearPoints(pNowRoadSegRel, pObj->positions[objPosNum-1].xgmean, pObj->positions[objPosNum-1].ygmean);

  //--------------------step 2: get final short term path point-------------------
  int   shortTermPathNum=pShortTermPath->positions.size();
  double shortTermPointX,shortTermPointY;
  for(int i=pNowRoadSegRel->segRels.size()-1; i>=0; i--)
  {
    double objNowXg =pObj->positions[objPosNum-1].xgmean;
    double objNowYg =pObj->positions[objPosNum-1].ygmean;
    double objMoveXg=pObj->positions[objPosNum-1].xgmean-pObj->positions[objPosNum-10].xgmean;
    double objMoveYg=pObj->positions[objPosNum-1].ygmean-pObj->positions[objPosNum-10].ygmean;
    int certainDisSegIndex=FindCertainDisSegIndex(objNowXg,objNowYg,objMoveXg,objMoveYg,&(pNowRoadSegRel->segRels[i]),disLongTermStart);
    if(certainDisSegIndex>=0)
    {
      shortTermPointX=pNowRoadSegRel->segRels[i].GCCS[certainDisSegIndex].xg;
      shortTermPointY=pNowRoadSegRel->segRels[i].GCCS[certainDisSegIndex].yg;
    }
    else
    {
      shortTermPointX=objNowXg+disLongTermStart*objMoveXg/sqrt(objMoveXg*objMoveXg+objMoveYg*objMoveYg);
      shortTermPointY=objNowYg+disLongTermStart*objMoveYg/sqrt(objMoveXg*objMoveXg+objMoveYg*objMoveYg);
    }
  }
  
  //--------------------step 3: match current segment-------------------
  int topLevelNum=pNowRoadSegRel->segRels.size();
  if (topLevelNum<=0)
  {
    //cout<<"There is no parallel segments!"<<endl;
    tmpPrePath.positions.clear();
    ivpredict::predictpos tmpPos;
    tmpPos.xgmean=pObj->positions[pObj->positions.size()-1].xgmean;
    tmpPos.ygmean=pObj->positions[pObj->positions.size()-1].ygmean;
    for(int i=0;i<50;i++)
    {
      tmpPrePath.positions.push_back(tmpPos);
    }
    matchedPaths.push_back(tmpPrePath);
    return matchedPaths;
  }
  for(int i=0;i<topLevelNum;i++)
  {
    //---------------------step 3.0: variable definition----------------------------
    int keySegNum=-1;
    int startNum=matchedPaths.size(); 
  
    //---------------------step 3.1: locate segment to be analyzed------------------
    int storedSegPointNum=gSavedSegNearPoints.size();
    for(int j=0;j<storedSegPointNum;j++)
    {
      if (gSavedSegNearPoints[j].objID==objID && gSavedSegNearPoints[j].segmenttype==pNowRoadSegRel->segRels[i].segmenttype && gSavedSegNearPoints[j].segmentnumber==pNowRoadSegRel->segRels[i].segmentnumber)
      {
	 keySegNum=j;
      }
    }
    if (keySegNum<=-1)
    {
      //cout<<"something wrong to locate segment!"<<endl;
    }
  
    //-------------step 3.2: find nearest segment point from final short term path point-------------
    int tmpNearLineNum=gSavedSegNearPoints[keySegNum].pLineNum;
    bool isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
    int tmpShortNearLineNum=FindNearPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]), pNowRoadSegRel->segRels[i].maxLineNum,shortTermPointX,shortTermPointY);
    //int tmpShortNearLineNum=FindCrossPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]), pNowRoadSegRel->segRels[i].maxLineNum,pShortTermPath);

    //----------step 3.3: if the nearest point is at start or end position, then analyze down level path----------
    if ((tmpShortNearLineNum==0 && pNowRoadSegRel->segRels[i].isInvSearch==true) || (tmpShortNearLineNum==(pNowRoadSegRel->segRels[i].maxLineNum-1) && pNowRoadSegRel->segRels[i].isInvSearch==false))
    {
      int downLevelNum=pNowRoadSegRel->segRels[i].subSeg.size();
      for(int j=0;j<downLevelNum;j++)
      {
	ivpredict::predictobj tmpPath;
	ivpredict::predictpos tmpPos;
	double sumTopAndDownJ=0;//disLongTermStart;//0;      
// cout<<"YY: curSeg="<<pNowRoadSegRel->currentSegmentType<<pNowRoadSegRel->currentSegmentNumber;
// cout<<", goSeg="<<pNowRoadSegRel->segRels[i].segmenttype<<pNowRoadSegRel->segRels[i].segmentnumber;
// cout<<", downSeg="<<pNowRoadSegRel->segRels[i].subSeg[j].segmenttype<<pNowRoadSegRel->segRels[i].subSeg[j].segmentnumber;
// cout<<"("<<pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch<<")"<<endl;
	if (pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch==false)
	{
	  int            maxNum=pNowRoadSegRel->segRels[i].subSeg[j].maxLineNum;
	  sGCCSpoint* pArrayAddr=&(pNowRoadSegRel->segRels[i].subSeg[j].GCCS[0]);
	  
	  int subSegNearPointNum=FindNearPointLineNum(false,0,pArrayAddr, maxNum,shortTermPointX,shortTermPointY);
	  //int subSegNearPointNum=FindCrossPointLineNum(false,0,pArrayAddr, maxNum,pShortTermPath);
	  if (subSegNearPointNum<maxNum-1)
	  {
	    for(int k=subSegNearPointNum+1; k<maxNum; k++)
	    {
	      double detaX=pArrayAddr[k].xg-pArrayAddr[k-1].xg;
	      double detaY=pArrayAddr[k].yg-pArrayAddr[k-1].yg;
	      sumTopAndDownJ+=sqrt(detaX*detaX+detaY*detaY);
	      tmpPos.xgmean=pArrayAddr[k].xg;
	      tmpPos.ygmean=pArrayAddr[k].yg;
	      tmpPath.positions.push_back(tmpPos);
	      if (sumTopAndDownJ>=totalDis)
	      {
		tmpPath.id=objID;
		matchedPaths.push_back(tmpPath);
		break;	
	      }
	    }
	    if (sumTopAndDownJ<totalDis && sumTopAndDownJ>0)
	    {
	      tmpPath.id=objID;
	      matchedPaths.push_back(tmpPath);
	    }
	  }
	  else
	  {
	    tmpPos.xgmean=pArrayAddr[maxNum-1].xg;
	    tmpPos.ygmean=pArrayAddr[maxNum-1].yg;
	    tmpPath.positions.push_back(tmpPos);
	    tmpPath.id=objID;
	    matchedPaths.push_back(tmpPath);
	  }
	}
	else if (pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch==true)
	{
	  int       maxNum=pNowRoadSegRel->segRels[i].subSeg[j].maxLineNum;
	  sGCCSpoint* pArrayAddr=&(pNowRoadSegRel->segRels[i].subSeg[j].GCCS[0]);
	  int subSegNearPointNum=FindNearPointLineNum(true,maxNum-1,pArrayAddr,maxNum,shortTermPointX,shortTermPointY);
	  //int   subSegNearPointNum=FindCrossPointLineNum(true,maxNum-1,pArrayAddr,maxNum,pShortTermPath);
	  if (subSegNearPointNum>0)
	  {
	    for(int k=subSegNearPointNum; k>0; k--)
	    {	  
	      double detaX=pArrayAddr[k].xg-pArrayAddr[k-1].xg;
	      double detaY=pArrayAddr[k].yg-pArrayAddr[k-1].yg;
	      sumTopAndDownJ+=sqrt(detaX*detaX+detaY*detaY);
	      tmpPos.xgmean=pArrayAddr[k-1].xg;
	      tmpPos.ygmean=pArrayAddr[k-1].yg;
	      tmpPath.positions.push_back(tmpPos);
	      if (sumTopAndDownJ>=totalDis)
	      {
		tmpPath.id=objID;
		matchedPaths.push_back(tmpPath);
		break;
	      }
	    }
	    if (sumTopAndDownJ<totalDis && sumTopAndDownJ>0)
	    {
	      tmpPath.id=objID;
	      matchedPaths.push_back(tmpPath);
	    }
	  }
	  else
	  {
	    tmpPos.xgmean=pArrayAddr[0].xg;
	    tmpPos.ygmean=pArrayAddr[0].yg;
	    tmpPath.positions.push_back(tmpPos);
	    tmpPath.id=objID;
	    matchedPaths.push_back(tmpPath);
	  }
	}
      }
    }
 
    //----------step 3.4: if the nearest point isn't at start or end position, then analyze down level path----------
    else if (tmpShortNearLineNum>0 && tmpShortNearLineNum<(pNowRoadSegRel->segRels[i].maxLineNum-1))
    {
      ivpredict::predictobj tmpRootPath;
      ivpredict::predictpos tmpPos;
      double sumTopI=0;
      if (pNowRoadSegRel->segRels[i].isInvSearch==false)
      {	
	for(int j=tmpShortNearLineNum+1; j<pNowRoadSegRel->segRels[i].maxLineNum; j++)
	{
	  sGCCSpoint* pArrayAddr=&(pNowRoadSegRel->segRels[i].GCCS[0]);
	  double detaX=pArrayAddr[j].xg-pArrayAddr[j-1].xg;
	  double detaY=pArrayAddr[j].yg-pArrayAddr[j-1].yg;	  
	  sumTopI+=sqrt(detaX*detaX+detaY*detaY);
	  tmpPos.xgmean=pArrayAddr[j].xg;
	  tmpPos.ygmean=pArrayAddr[j].yg;	  
	  tmpRootPath.positions.push_back(tmpPos);
	  if (sumTopI>=totalDis)
	  {	
	    tmpRootPath.id=objID;	  
	    matchedPaths.push_back(tmpRootPath); 
	   break;
	  }
	}
      }
      else if (pNowRoadSegRel->segRels[i].isInvSearch==true)
      {
	for(int j=tmpShortNearLineNum; j>0; j--)
	{
	  sGCCSpoint* pArrayAddr=&(pNowRoadSegRel->segRels[i].GCCS[0]);
	  double detaX=pArrayAddr[j].xg-pArrayAddr[j-1].xg;
	  double detaY=pArrayAddr[j].yg-pArrayAddr[j-1].yg;
	  sumTopI+=sqrt(detaX*detaX+detaY*detaY);
	  tmpPos.xgmean=pArrayAddr[j-1].xg;
	  tmpPos.ygmean=pArrayAddr[j-1].yg;
	  tmpRootPath.positions.push_back(tmpPos);
	  if (sumTopI>=totalDis)
	  {
	    tmpRootPath.id=objID;
	    matchedPaths.push_back(tmpRootPath);
	    break;
	  }
	}
      }

      if (sumTopI<totalDis)
      {
	int downLevelNum=pNowRoadSegRel->segRels[i].subSeg.size();  
	if (downLevelNum<=0)
	{
	  tmpRootPath.id=objID;
	  matchedPaths.push_back(tmpRootPath);
	}
	else if (downLevelNum>=1)
	{  
	  for(int j=0;j<downLevelNum;j++)
	  {
// cout<<"YY: curSeg="<<pNowRoadSegRel->currentSegmentType<<pNowRoadSegRel->currentSegmentNumber;
// cout<<", goSeg="<<pNowRoadSegRel->segRels[i].segmenttype<<pNowRoadSegRel->segRels[i].segmentnumber;
// cout<<", downSeg="<<pNowRoadSegRel->segRels[i].subSeg[j].segmenttype<<pNowRoadSegRel->segRels[i].subSeg[j].segmentnumber;
// cout<<"("<<pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch<<")"<<endl;
	    ivpredict::predictobj tmpPath=tmpRootPath;
	    int   maxArryNum=pNowRoadSegRel->segRels[i].subSeg[j].maxLineNum;
	    sGCCSpoint* pArrayAddr=&(pNowRoadSegRel->segRels[i].subSeg[j].GCCS[0]);
	    double sumTopAndDownJ=sumTopI;  
	    if (pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch==false)
	    {	    
	      for(int k=1; k<maxArryNum; k++)
	      {	  
		double detaX=pArrayAddr[k].xg-pArrayAddr[k-1].xg;
		double detaY=pArrayAddr[k].yg-pArrayAddr[k-1].yg;
		sumTopAndDownJ+=sqrt(detaX*detaX+detaY*detaY);
		tmpPos.xgmean=pArrayAddr[k].xg;
		tmpPos.ygmean=pArrayAddr[k].yg;
		tmpPath.positions.push_back(tmpPos);
		if (sumTopAndDownJ>=totalDis)
		{
		  matchedPaths.push_back(tmpPath);
		  break;
		}
	      }
	    }
	    else if (pNowRoadSegRel->segRels[i].subSeg[j].isInvSearch==true)
	    {
	      for(int k=maxArryNum-1; k>0; k--)
	      {
		double detaX=pArrayAddr[k].xg-pArrayAddr[k-1].xg;
		double detaY=pArrayAddr[k].yg-pArrayAddr[k-1].yg;
		sumTopAndDownJ+=sqrt(detaX*detaX+detaY*detaY);
		tmpPos.xgmean=pArrayAddr[k-1].xg;
		tmpPos.ygmean=pArrayAddr[k-1].yg;
		tmpPath.positions.push_back(tmpPos);
		if (sumTopAndDownJ>=totalDis)
		{
		  matchedPaths.push_back(tmpPath);
		  break;
		}
	      }
	    }
	    
	    if (sumTopAndDownJ<totalDis && sumTopAndDownJ>0)
	    {
	      tmpPath.id=objID;
	      matchedPaths.push_back(tmpPath);
	    }
	  }
	}
      }
    }
    else
    {
      //ROS_WARN_STREAM("the input search direction of pNowRoadSegRel->segRels["<<i<<"] is wrong!");
    }
    int endNum=matchedPaths.size();

//     for(int m=startNum;m<=endNum-1;m++)
//     {
//       matchedPaths[m].isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
//     }
  }
  
  //----------step 4: sparse long path points----------
  shortTermPathNum=pShortTermPath->positions.size();
  double baseDisX=pShortTermPath->positions[1].xgmean-pShortTermPath->positions[0].xgmean;
  double baseDisY=pShortTermPath->positions[1].ygmean-pShortTermPath->positions[0].ygmean;
  double baseDis=sqrt(baseDisX*baseDisX+baseDisY*baseDisY);
  int longPathNum=matchedPaths.size();
  for(int i=0;i<longPathNum;i++)
  {
    int pathSize=matchedPaths[i].positions.size();
    double tmpNowX=matchedPaths[i].positions[0].xgmean;
    double tmpNowY=matchedPaths[i].positions[0].ygmean;
    for(int j=0;j<=pathSize-1;j++)
    {
      double detaX=matchedPaths[i].positions[j].xgmean-tmpNowX;
      double detaY=matchedPaths[i].positions[j].ygmean-tmpNowY;
      double tmpDis=sqrt(detaX*detaX+detaY*detaY);
      if (tmpDis>=baseDis)
      {
	tmpNowX=matchedPaths[i].positions[j].xgmean;
	tmpNowY=matchedPaths[i].positions[j].ygmean;
      }
      else
      {
	matchedPaths[i].positions.erase(matchedPaths[i].positions.begin()+j);
	j--;
	pathSize--;
      }
    }
  }

  //-----------return----------------------------------
  return matchedPaths;
}

/**************************************************************
Function: MatchParallelLongTermPath()
Author: zongwei
Date: 2017.5.17
Description: // 函数功能、性能等的描述
1. match long term path for intention of moving ahead
2. -------
3. -------
Input:
1. pObj: pointer for object buffer
2. pShortTermPath: pointer for short term path
3. pNowRoadSegRel: pointer for local road segment map
// 用、取值说明及参数间关系。
Output: // 对输出参数的说明
Return: matched long term paths
Others: // 其他说明
************************************************************/
std::vector<ivpredict::predictobj> MatchParallelLongTermPath(ivpredict::objbuffer *pObj, ivpredict::predictobj *pShortTermPath, sRoadSegRel *pNowRoadSegRel,int timeLength)
{
  std::vector<ivpredict::predictobj> matchedPaths;
  matchedPaths.clear();

  //-----------------------------step 0: set initial key parameters--------------------------------
  double vectorObjX,vectorObjY,vectorBoundX,vectorBoundY,vectorCentreX,vectorCentreY,vectorRoadX,vectorRoadY;
  int shortNearLineNum;
  
  //---------------step 1: find nearest point index on target segment and store it in gSavedSegNearPoints-------------------
  int objPosNum=pObj->positions.size();
  UpdateSavedSegNearPoints(pNowRoadSegRel, pObj->positions[objPosNum-1].xgmean, pObj->positions[objPosNum-1].ygmean);

  //--------------------step 2: get final short term path point-------------------
  int   shortTermPathNum=pShortTermPath->positions.size();
  double shortTermPointX=pShortTermPath->positions[shortTermPathNum-1].xgmean;
  double shortTermPointY=pShortTermPath->positions[shortTermPathNum-1].ygmean;
  
  //--------------------step 3: calculate moving vector of the object-------------------
  vectorObjX=shortTermPointX-pShortTermPath->positions[shortTermPathNum-3].xgmean;
  vectorObjY=shortTermPointY-pShortTermPath->positions[shortTermPathNum-3].ygmean;
  
  //-------------step 4: find nearest segment point from target segment and calculate corresponding vector-------------
  int topLevelNum=pNowRoadSegRel->segRels.size(); //number of target parallel segments in nowRoadSegRel
  for(int i=0;i<topLevelNum;i++)
  {
    //---------------------step 4.0: variable definition----------------------------
    int keySegNum=-1;
  
    //---------------------step 4.1: find segment in gSavedSegNearPoints to be analyzed-------------------
    int storedSegPointNum=gSavedSegNearPoints.size();
    for(int j=0;j<storedSegPointNum;j++)
    {
      if (gSavedSegNearPoints[j].objID==pNowRoadSegRel->objID && gSavedSegNearPoints[j].segmenttype==pNowRoadSegRel->segRels[i].segmenttype && gSavedSegNearPoints[j].segmentnumber==pNowRoadSegRel->segRels[i].segmentnumber)
      {
	       keySegNum=j;
      }
    }
    if (keySegNum<=-1)
    {
      //cout<<"something wrong to locate segment in walker's funtion crossMatchLongTermPath!"<<endl;
    }

    //-------------step 4.2: find nearest segment point from final short term path point-------------
    int tmpNearLineNum=gSavedSegNearPoints[keySegNum].pLineNum;
    bool isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
    shortNearLineNum=FindNearPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]), pNowRoadSegRel->segRels[i].maxLineNum,shortTermPointX,shortTermPointY);
   
    //-------------step 4.3: calculate target segment vector-------------
    if (pNowRoadSegRel->segRels[i].isInvSearch==false)
    {
      if (shortNearLineNum==0)
      {
	vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg;
	vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg;
      }
      else
      {
	vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].xg;
	vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].yg;
      }
    }
    else
    {
      if (shortNearLineNum==0)
      {
	vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].xg;
	vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].yg;
      }
      else
      {
	vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg;
	vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg;
      }
    }
    
    //-------------step 4.3: calculate vector from final short term point to target segment-------------
    vectorCentreX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-shortTermPointX;
    vectorCentreY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-shortTermPointY;     
  }

  //--------------------step 5: calculate translation vector---------------------------------
  double moveVectorX,moveVectorY;
  double tmpDisRoad=sqrt(vectorRoadX*vectorRoadX+vectorRoadY*vectorRoadY);
  double tmpDisCenter=sqrt(vectorCentreX*vectorCentreX+vectorCentreY*vectorCentreY);
  double betweenAng=acos(fabs(vectorCentreX*vectorRoadX+vectorCentreY*vectorRoadY)/tmpDisCenter/tmpDisRoad);  
  double projectDis=tmpDisCenter*cos(betweenAng);
  if (vectorCentreX*vectorRoadX+vectorCentreY*vectorRoadY>0)
  {
    moveVectorX=-vectorCentreX+projectDis*vectorRoadX/tmpDisRoad;
    moveVectorY=-vectorCentreY+projectDis*vectorRoadY/tmpDisRoad;     
  }
  else
  {
    moveVectorX=-vectorCentreX-projectDis*vectorRoadX/tmpDisRoad;
    moveVectorY=-vectorCentreY-projectDis*vectorRoadY/tmpDisRoad;     
  }
  
  //--------------------step 6: calculate long path-----------------------------------
  matchedPaths=MatchChangeSegLongTermPath(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
  
  //--------------------step 8: move long path-----------------------------------
  for(int i=0;i<matchedPaths.size();i++)
  {
    for(int j=0;j<matchedPaths[i].positions.size();j++)
    {
      matchedPaths[i].positions[j].xgmean+=moveVectorX;
      matchedPaths[i].positions[j].ygmean+=moveVectorY;
    }
  }
  
  //--------------------step 6: return-------------------
  return matchedPaths;
}


/**************************************************************
Function: MatchParallelLongTermPath2()
Author: zongwei
Date: 2017.5.17
Description: // 函数功能、性能等的描述
1. match long term path for intention of moving ahead
2. -------
3. -------
Input:
1. pObj: pointer for object buffer
2. pShortTermPath: pointer for short term path
3. pNowRoadSegRel: pointer for local road segment map
// 用、取值说明及参数间关系。
Output: // 对输出参数的说明
Return: matched long term paths
Others: // 其他说明
************************************************************/
std::vector<ivpredict::predictobj> MatchParallelLongTermPath2(ivpredict::objbuffer *pObj, ivpredict::predictobj *pShortTermPath, sRoadSegRel *pNowRoadSegRel,int timeLength)
{
  std::vector<ivpredict::predictobj> matchedPaths;
  matchedPaths.clear();

  //-----------------------------step 0: set initial key parameters--------------------------------
  double vectorObjX,vectorObjY,vectorBoundX,vectorBoundY,vectorCentreX,vectorCentreY,vectorRoadX,vectorRoadY;
  int shortNearLineNum;
  
  //---------------step 1: find nearest point index on target segment and store it in gSavedSegNearPoints-------------------
  int objPosNum=pObj->positions.size();
  UpdateSavedSegNearPoints(pNowRoadSegRel, pObj->positions[objPosNum-1].xgmean, pObj->positions[objPosNum-1].ygmean);

  //--------------------step 2: get final short term path point-------------------
  int   shortTermPathNum=pShortTermPath->positions.size();
  double shortTermPointX=pShortTermPath->positions[shortTermPathNum].xgmean;
  double shortTermPointY=pShortTermPath->positions[shortTermPathNum].ygmean;
  
  //--------------------step 3: calculate moving vector of the object-------------------
  vectorObjX=(shortTermPointX-pShortTermPath->positions[shortTermPathNum-3].xgmean);
  vectorObjY=(shortTermPointY-pShortTermPath->positions[shortTermPathNum-3].ygmean);
  
  //-------------step 4: find nearest segment point from target segment and calculate corresponding vector-------------
  int topLevelNum=pNowRoadSegRel->segRels.size(); //number of target parallel segments in nowRoadSegRel
  for(int i=0;i<topLevelNum;i++)
  {
    //---------------------step 4.0: variable definition----------------------------
    int keySegNum=-1;
  
    //---------------------step 4.1: find segment in gSavedSegNearPoints to be analyzed-------------------
    int storedSegPointNum=gSavedSegNearPoints.size();
    for(int j=0;j<storedSegPointNum;j++)
    {
      if (gSavedSegNearPoints[j].objID==pNowRoadSegRel->objID && gSavedSegNearPoints[j].segmenttype==pNowRoadSegRel->segRels[i].segmenttype && gSavedSegNearPoints[j].segmentnumber==pNowRoadSegRel->segRels[i].segmentnumber)
      {
         keySegNum=j;
      }
    }
    if (keySegNum<=-1)
    {
      //cout<<"something wrong to locate segment in walker's funtion crossMatchLongTermPath!"<<endl;
    }

    //-------------step 4.2: find nearest segment point from final short term path point-------------
    int tmpNearLineNum=gSavedSegNearPoints[keySegNum].pLineNum;
    bool isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
    shortNearLineNum=FindNearPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]), pNowRoadSegRel->segRels[i].maxLineNum,shortTermPointX,shortTermPointY);
   
    //-------------step 4.3: calculate target segment vector-------------
    if (pNowRoadSegRel->segRels[i].isInvSearch==false)
    {
      if (shortNearLineNum==0)
      {
        vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg;
        vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg;
      }
      else
      {
        vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].xg;
        vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].yg;
      }
    }
    else
    {
      if (shortNearLineNum==0)
      {
        vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].xg;
        vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].yg;
      }
      else
      {
        vectorRoadX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].xg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg;
        vectorRoadY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].yg-pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg;
      }
    }
    
    //-------------step 4.3: calculate vector from final short term point to target segment-------------
    vectorCentreX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-shortTermPointX;
    vectorCentreY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-shortTermPointY;     
  }

  //--------------------step 5: calculate translation vector---------------------------------
  double moveVectorX,moveVectorY;
  double tmpDisRoad=sqrt(vectorRoadX*vectorRoadX+vectorRoadY*vectorRoadY);
  double tmpDisCenter=sqrt(vectorCentreX*vectorCentreX+vectorCentreY*vectorCentreY);
  double betweenAng=acos(fabs(vectorCentreX*vectorRoadX+vectorCentreY*vectorRoadY)/tmpDisCenter/tmpDisRoad);  
  double projectDis=tmpDisCenter*cos(betweenAng);
  if (vectorCentreX*vectorRoadX+vectorCentreY*vectorRoadY>0)
  {
    moveVectorX=-vectorCentreX+projectDis*vectorRoadX/tmpDisRoad;
    moveVectorY=-vectorCentreY+projectDis*vectorRoadY/tmpDisRoad;     
  }
  else
  {
    moveVectorX=-vectorCentreX-projectDis*vectorRoadX/tmpDisRoad;
    moveVectorY=-vectorCentreY-projectDis*vectorRoadY/tmpDisRoad;     
  }
  
  //--------------------step 6: calculate long path-----------------------------------
  matchedPaths=MatchChangeSegLongTermPath(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
  
  //--------------------step 8: move long path-----------------------------------
  for(int i=0;i<matchedPaths.size();i++)
  {
    for(int j=0;j<matchedPaths[i].positions.size();j++)
    {
      matchedPaths[i].positions[j].xgmean+=moveVectorX;
      matchedPaths[i].positions[j].ygmean+=moveVectorY;
    }
  }
  
  //--------------------step 6: return-------------------
  return matchedPaths;
}

/**************************************************************
Function: MatchVerticalLongTermPath()
Author: zongwei
Date: 2017.4.18
Description: // 函数功能、性能等的描述
1. match long term path for intention of moving vertical to the road
2. -------
3. -------
Input:
1. pObj: pointer for object buffer
2. pShortTermPath: pointer for short term path
3. pNowRoadSegRel: pointer for local road segment map
// 用、取值说明及参数间关系。
Output: // 对输出参数的说明
Return: matched long term paths
Others: // 其他说明
************************************************************/
std::vector<ivpredict::predictobj> MatchVerticalLongTermPath(ivpredict::objbuffer *pObj, ivpredict::predictobj *pShortTermPath, sRoadSegRel *pNowRoadSegRel,int timeLength)
{
  std::vector<ivpredict::predictobj> matchedPaths; 
  matchedPaths.clear();
  ivpredict::predictobj tmpPrePath = *pShortTermPath;
  
  // matchedPaths.clear();
  if (pNowRoadSegRel->segRels.size()<=0)
  {
    //cout<<"no segments on first layer for predict vertical motion path"<<endl;
    matchedPaths.push_back(tmpPrePath);
    return matchedPaths;
  }

  //-----------------------------step 0: set initial key parameters--------------------------------
  double vectorObjX,vectorObjY,vectorBoundX,vectorBoundY;
  int shortNearLineNum;
  int extendNum=0;   
  double initDis=0.1;
  bool needFollow=false;  
  
  //--------------------step 1: find nearest point on each segment and store it in gSavedSegNearPoints-------------------
  int objPosNum=pObj->positions.size();
  UpdateSavedSegNearPoints(pNowRoadSegRel, pObj->positions[objPosNum-1].xgmean, pObj->positions[objPosNum-1].ygmean);  
  
  //--------------------step 2: get final short term path point-------------------
  int   shortTermPathNum=pShortTermPath->positions.size();
  double shortTermPointX=pShortTermPath->positions[shortTermPathNum-1].xgmean;
  double shortTermPointY=pShortTermPath->positions[shortTermPathNum-1].ygmean;  
    
  //--------------------step 3: calculate moving vector of the object------------------
  vectorObjX=shortTermPointX-pShortTermPath->positions[shortTermPathNum-2].xgmean;
  vectorObjY=shortTermPointY-pShortTermPath->positions[shortTermPathNum-2].ygmean;
  
  //-------------step 4: find nearest segment point from final short term path point and calculate corresponding vector-------------
  int topLevelNum=pNowRoadSegRel->segRels.size();
  for(int i=0;i<topLevelNum;i++)
  {
    //---------------------step 4.0: variable definition----------------------------
    int keySegNum=-1;  
  
    //---------------------step 4.1: locate segment to be analyzed-------------------
    int storedSegPointNum=gSavedSegNearPoints.size();
    for(int j=0;j<storedSegPointNum;j++)
    {
      if (gSavedSegNearPoints[j].objID==pNowRoadSegRel->objID && gSavedSegNearPoints[j].segmenttype==pNowRoadSegRel->segRels[i].segmenttype && gSavedSegNearPoints[j].segmentnumber==pNowRoadSegRel->segRels[i].segmentnumber)
      {
	keySegNum=j;
      }
    }
    if (keySegNum<=-1)
    {
      //cout<<"something wrong to locate segment in walker's funtion verticalMatchLongTermPath!"<<endl;
    }
  
    //-------------step 4.2: find nearest segment point from final short term path point-------------
    int tmpNearLineNum=gSavedSegNearPoints[keySegNum].pLineNum;
    bool isInvSearch=pNowRoadSegRel->segRels[i].isInvSearch;
    shortNearLineNum=FindNearPointLineNum(isInvSearch,tmpNearLineNum,&(pNowRoadSegRel->segRels[i].GCCS[0]), pNowRoadSegRel->segRels[i].maxLineNum,shortTermPointX,shortTermPointY);

    //-------------step 4.3: calculate vector from final short term path point to nearest point-------------
    vectorBoundX=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].xg-shortTermPointX;
    vectorBoundY=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum].yg-shortTermPointY;

    //--------------------step 4.4: analyze the boundary segment to follow-------------------
    int maxLineNum=pNowRoadSegRel->segRels[i].maxLineNum;
    double dotProduct=vectorObjX*vectorBoundX+vectorObjY*vectorBoundY;

    if (shortNearLineNum>0 && shortNearLineNum<maxLineNum-1 && dotProduct>0)
    {
      //calculate a1*x+b1*y=1 or y=k1*x+c1.
      double tmpX1=pShortTermPath->positions[shortTermPathNum-2].xgmean;      
      double tmpY1=pShortTermPath->positions[shortTermPathNum-2].ygmean;
      double tmpX2=shortTermPointX;
      double tmpY2=shortTermPointY;
      double a1=(tmpY2-tmpY1)/(tmpX1*tmpY2-tmpX2*tmpY1);
      double b1=(tmpX1-tmpX2)/(tmpX1*tmpY2-tmpX2*tmpY1);
      //calculate a2*x+b2*y=1.
      tmpX1=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].xg;
      tmpY1=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum+1].yg;
      tmpX2=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].xg;
      tmpY2=pNowRoadSegRel->segRels[i].GCCS[shortNearLineNum-1].yg;
      double a2=(tmpY2-tmpY1)/(tmpX1*tmpY2-tmpX2*tmpY1);
      double b2=(tmpX1-tmpX2)/(tmpX1*tmpY2-tmpX2*tmpY1);
      //calculate cross point
      double finalX=(b2-b1)/(a1*b2-a2*b1);
      double finalY=(a1-a2)/(a1*b2-a2*b1); 
      //calculate distance
      double detaX=finalX-shortTermPointX;
      double detaY=finalY-shortTermPointY;
      double distance=sqrt(detaX*detaX+detaY*detaY);
      //check and record
      if (distance>initDis)
      {
	extendNum=ceil(distance/sqrt(vectorObjX*vectorObjX+vectorObjY*vectorObjY));
	initDis=distance;
	needFollow=true;
      }
    }
  }

  //--------------------step 5: calculate long path-------------------
  if (needFollow==true)
  {
    ivpredict::predictobj tmpPrePath;
    ivpredict::predictpos tmpPos;
    for(int i=1;i<=extendNum;i++)
    {
      tmpPos.xgmean=shortTermPointX+i*vectorObjX;
      tmpPos.ygmean=shortTermPointY+i*vectorObjY;
      tmpPrePath.positions.push_back(tmpPos);
    }
    matchedPaths.push_back(tmpPrePath); 
  }

  //--------------------step 6: return-------------------
  return matchedPaths;
}

/**************************************************************
Function: SmoothPath()
Author: zongwei
Date: 2017.4.24
Description: // 函数功能、性能等的描述
1. smooth pregetted initial short long term paths
2. -------
3. -------
Input:
1. dataNumForSmooth: the second parameter dataNumForSmooth must be an odd number.
2. initPrePath: initial predicted paths
// 用、取值说明及参数间关系。
Output: // 对输出参数的说明
Return: smoothed predicted paths
Others: // 其他说明
************************************************************/
ivpredict::predictobj SmoothPath(ivpredict::predictobj initPrePath, int dataNumForSmooth)
{
  int totalNum=initPrePath.positions.size();  
  int halfNumForSmooth=(dataNumForSmooth-1)/2;
  for(int i=halfNumForSmooth;i<totalNum-halfNumForSmooth;i++)
  {
    double sumX=0;
    double sumY=0;
    for(int j=i-halfNumForSmooth;j<=i+halfNumForSmooth;j++)
    {
      sumX+=initPrePath.positions[j].xgmean;
      sumY+=initPrePath.positions[j].ygmean;
    }
    initPrePath.positions[i].xgmean=sumX/(dataNumForSmooth*1.0);
    initPrePath.positions[i].ygmean=sumY/(dataNumForSmooth*1.0);
  }
  return initPrePath;
}

/**************************************************************
Function: CalPathIncludeVelUseCurveB()
Author: zongwei
Date: 2017.4.24
Description: // 函数功能、性能等的描述
1. cosidering velocity of the predicted paths and then resampling points
2. -------
3. -------
Input:
1. initPrePath: predicted paths without considering velocity
2. fitOrder: order for fitting
3. objVel: velocity of the object
4. timeLength: time length for predicting. unit s.
5. timeStep: time step length.unit s
Output: // 对输出参数的说明
Return: predicted paths
Others: // 其他说明
************************************************************/
ivpredict::predictobj CalPathIncludeVelUseCurveB(ivpredict::predictobj initPrePath,int fitOrder,double objVel,double timeLength,double timeStep)
{
  ivpredict::predictobj velPrePath;
  ivpredict::predictpos tmpPoint;
  velPrePath.positions.push_back(initPrePath.positions[0]);
  
  if (fitOrder==3)
  { 
    double detaX=(1.0/6.0-1.0)*initPrePath.positions[0].xgmean+4.0/6.0*initPrePath.positions[1].xgmean+1.0/6.0*initPrePath.positions[2].xgmean;
    double detaY=(1.0/6.0-1.0)*initPrePath.positions[0].ygmean+4.0/6.0*initPrePath.positions[1].ygmean+1.0/6.0*initPrePath.positions[2].ygmean;
    double length=sqrt(detaX*detaX+detaY*detaY);
    
    int totalSize=initPrePath.positions.size();
    for(int i=0;i<=totalSize-1-fitOrder;i++)
    { 
      double detaRate=0.1;
      for(double t=detaRate;t<=1;t+=detaRate)
      {     
	double dP_dtX =(-t*t+2*t-1)/2*initPrePath.positions[i].xgmean+(3*t*t-4*t)/2*initPrePath.positions[i+1].xgmean;
	double dP_dtY =(-t*t+2*t-1)/2*initPrePath.positions[i].ygmean+(3*t*t-4*t)/2*initPrePath.positions[i+1].ygmean;
	dP_dtX+=(-3*t*t+2*t+1)/2*initPrePath.positions[i+2].xgmean+t*t/2*initPrePath.positions[i+3].xgmean;
	dP_dtY+=(-3*t*t+2*t+1)/2*initPrePath.positions[i+2].ygmean+t*t/2*initPrePath.positions[i+3].ygmean;  
	
	double tmpLength=length+sqrt(dP_dtX*dP_dtX+dP_dtY*dP_dtY)*detaRate;
	if (tmpLength<timeStep*objVel)
	{
	  length=tmpLength;
	}
	else
	{	
	  double minDetaRate=detaRate/ceil(tmpLength/timeStep/objVel)/5.0;
	  for(double At=t-detaRate+minDetaRate; At<=t; At+=minDetaRate)
	  {
	    double dP_dtX1 =(-At*At+2*At-1)/2*initPrePath.positions[i].xgmean+(3*At*At-4*At)/2*initPrePath.positions[i+1].xgmean;
	    double dP_dtY1 =(-At*At+2*At-1)/2*initPrePath.positions[i].ygmean+(3*At*At-4*At)/2*initPrePath.positions[i+1].ygmean;	    
	    dP_dtX1+=(-3*At*At+2*At+1)/2*initPrePath.positions[i+2].xgmean+At*At/2*initPrePath.positions[i+3].xgmean;
	    dP_dtY1+=(-3*At*At+2*At+1)/2*initPrePath.positions[i+2].ygmean+At*At/2*initPrePath.positions[i+3].ygmean;
	    length+=sqrt(dP_dtX1*dP_dtX1+dP_dtY1*dP_dtY1)*minDetaRate;
	    int tmpCount=floor(length/(timeStep*objVel));
	    if (tmpCount>=1)
	    {
	      double Px=(-At*At*At+3*At*At-3*At+1)/6*initPrePath.positions[i].xgmean+(3*At*At*At-6*At*At+4)/6*initPrePath.positions[i+1].xgmean;
	      double Py=(-At*At*At+3*At*At-3*At+1)/6*initPrePath.positions[i].ygmean+(3*At*At*At-6*At*At+4)/6*initPrePath.positions[i+1].ygmean;	      
	      Px+=(-3*At*At*At+3*At*At+3*At+1)/6*initPrePath.positions[i+2].xgmean+At*At*At/6*initPrePath.positions[i+3].xgmean;
	      Py+=(-3*At*At*At+3*At*At+3*At+1)/6*initPrePath.positions[i+2].ygmean+At*At*At/6*initPrePath.positions[i+3].ygmean;	
	      tmpPoint.xgmean=Px;
	      tmpPoint.ygmean=Py;
	      velPrePath.positions.push_back(tmpPoint);
 	      length-=timeStep*objVel;
	    }
	  }
	}
      }
    }
  }
  
  int totalNum;
  switch(gSetTimeMode)
  {
    case true:
      totalNum=round(timeLength/timeStep);
      break;
    case false:
      totalNum=round(gPathDisSet/initPrePath.v_abs/timeStep);
      break;
    default:
      break;
  } 
  if (velPrePath.positions.size()>totalNum)
  {
    velPrePath.positions.erase(velPrePath.positions.begin()+totalNum,velPrePath.positions.end());
  }

  velPrePath.id=initPrePath.id;
  velPrePath.time_interval=timeStep;
  velPrePath.steps=totalNum;  
  velPrePath.length=initPrePath.length;
  velPrePath.width=initPrePath.width;
  velPrePath.height=initPrePath.height;
  velPrePath.cell=initPrePath.cell;
  velPrePath.xvrel=initPrePath.xvrel;
  velPrePath.yvrel=initPrePath.yvrel;
  velPrePath.vrel=initPrePath.vrel;
  velPrePath.v_abs=initPrePath.v_abs;
  velPrePath.type=initPrePath.type;
  velPrePath.probability=initPrePath.probability;
  
  return velPrePath;
}

/**************************************************************
Function: FindGoStraightSegment()
Author: zongwei
Date: 2017.6.13
Description: // 函数功能、性能等的描述
1. reserve current segment to match for intention of going ahead.
2. -------
3. -------
Input:
1. pNowRoadSegRel: pointer for local road segment map
Output: // 对输出参数的说明
Return: // 对函数返回值的说明
Others: // 其他说明
************************************************************/
void FindGoStraightSegment(sRoadSegRel *pNowRoadSegRel)
{
  for (int i=pNowRoadSegRel->segRels.size()-1; i>=0; i--)
  {
    if (pNowRoadSegRel->segRels[i].segmenttype!=pNowRoadSegRel->currentSegmentType || pNowRoadSegRel->segRels[i].segmentnumber!=pNowRoadSegRel->currentSegmentNumber)
    {
      pNowRoadSegRel->segRels.erase(pNowRoadSegRel->segRels.begin()+i);
    }
  }
  return;
}

/**************************************************************
Function: FindPossibleChangedSegments()
Author: zongwei
Date: 2017.6.13
Description: // 函数功能、性能等的描述
1. remove segments which are too far to be considered.
2. -------
3. -------
Input: 
1. thresholdMaxDis: unit is m
2. pObj: pointer for the object of buffer.
3. pNowRoadSegRel: pointer for local road segment map
Output: // 对输出参数的说明
Return: // 对函数返回值的说明
Others: // 其他说明
************************************************************/
void RemoveFarSegments(sRoadSegRel *pNowRoadSegRel, ivpredict::objbuffer *pObj, float thresholdMaxDis)
{
  int posNum=pObj->positions.size();
  int nowX=pObj->positions[posNum-1].xgmean;
  int nowY=pObj->positions[posNum-1].ygmean;
  for (int i=pNowRoadSegRel->segRels.size()-1; i>=0; i--)
  {
    float dis=CalPoint2SegmentDis(nowX, nowY, &pNowRoadSegRel->segRels[i]);
    if(dis>thresholdMaxDis)
    {
      pNowRoadSegRel->segRels.erase(pNowRoadSegRel->segRels.begin()+i);      
    }
  }  
  return;
}

/**************************************************************
Function: SelectBoundarySegments()
Author: zongwei
Date: 2017.6.13
Description: // 函数功能、性能等的描述
1. select boundary segments
2. -------
3. -------
Input:
1. pNowRoadSegRel: pointer for local road segment map
Output: // 对输出参数的说明
Return: // 对函数返回值的说明
Others: // 其他说明
************************************************************/
void SelectBoundarySegments(sRoadSegRel *pNowRoadSegRel)
{
  int segTotalNum=pNowRoadSegRel->segRels.size();
  for (int i=segTotalNum-1; i>=0; i--)
  {
    if(pNowRoadSegRel->segRels[i].segmenttype!=kSegmentBoundary)
    {
      pNowRoadSegRel->segRels.erase(pNowRoadSegRel->segRels.begin()+i);
    }
  }
  return;
}

/**************************************************************
Function: PredictObjPath()
Author: zongwei
Date: 2017.4.21
Description: // 函数功能、性能等的描述
1. predict paths of object
2. -------
3. -------
Input:
1. pNowRoadSegRel: pointer for local road segment map
2. pObj: pointer for the object of buffer.
3. pShortTermPath: pointer for short term path
4. eNowObjIntention: object intention.
5. timeLength: time length for predicting. unit s.
6. timeStep: time step length.unit s
Output: // 对输出参数的说明
Return: predicted paths
Others: // 其他说明
************************************************************/
ivpredict::ivmsgpredict PredictObjPath(sRoadSegRel *pNowRoadSegRel,ivpredict::objbuffer *pObj,ivpredict::predictobj *pShortTermPath,eObjIntention eNowObjIntention, double timeLength,double timeStep)
{ 
  ivpredict::ivmsgpredict predictShortLongPaths;
  
  //---------step 0: set key parameters---------------
  int dataNumForSmooth=5; //this value must be odd for smoothing.
  int fitOrder=3; 	//fit order for B curve
  double objVel=pShortTermPath->v_abs; //unit:m/s.
  for(int i=0;i<pNowRoadSegRel->segRels.size();i++)
  {
    sSingleObjIDandSegID sCurrentObjIDandSegID;
    sCurrentObjIDandSegID.objID=pNowRoadSegRel->objID;
    sCurrentObjIDandSegID.segmenttype=pNowRoadSegRel->segRels[i].segmenttype; 
    sCurrentObjIDandSegID.segmentnumber=pNowRoadSegRel->segRels[i].segmentnumber; 
    gAllObjIDandSegID.push_back(sCurrentObjIDandSegID);		//used to minimus calculating time for finding nearest point index on target segment.
    for(int j=0;j<pNowRoadSegRel->segRels[i].subSeg.size();j++)
    {
      sCurrentObjIDandSegID.objID=pNowRoadSegRel->objID;
      sCurrentObjIDandSegID.segmenttype=pNowRoadSegRel->segRels[i].subSeg[j].segmenttype; 
      sCurrentObjIDandSegID.segmentnumber=pNowRoadSegRel->segRels[i].subSeg[j].segmentnumber; 
      gAllObjIDandSegID.push_back(sCurrentObjIDandSegID);
    }
  }
  //---------step 1: pre-process sRoadSegRel--------------
  switch (eNowObjIntention)
  {
    case GOSTRAIGHT:	//only give one current centrel line which the obj is being on.   
      // FindGoStraightSegment(pNowRoadSegRel);  //find go straight segment need to follow
      RemoveFarSegments(pNowRoadSegRel,pObj,15.0);
      break;
    case CHANGESEGMENT: //give all adjacent parallel centrel and boundary lines, and remove far segments.
      RemoveFarSegments(pNowRoadSegRel,pObj,15.0);	//remove too far segmetns mainly used for intersection
      break;
    case CROSSROAD:	//only give adjacent two boundary lines;
      SelectBoundarySegments(pNowRoadSegRel);		//only reserve boundary segments
      RemoveFarSegments(pNowRoadSegRel,pObj,20.0);
      break;
    default:
      break;
  }

  //-----------step 2: calculate path-------------------
  ivpredict::predictobj shortTermPrePath=*pShortTermPath;
  stop_flag = 0;
  //---------step 2.1: calculate long path--------------------
  std::vector<ivpredict::predictobj> tmpLongPredictPaths;
  // smoothedVelPrePath.intention = 4;
  switch (eNowObjIntention)
  {
    case GOSTRAIGHT:
      // tmpLongPredictPaths = MatchParallelLongTermPath(pObj, &shortTermPrePathGoStraight, pNowRoadSegRel,timeLength);
      // tmpLongPredictPaths = MatchParallelLongTermPath2(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
      tmpLongPredictPaths = MatchChangeSegLongTermPath(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
      break;
    case CHANGESEGMENT:
      tmpLongPredictPaths = MatchChangeSegLongTermPath(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
      break;
    case CROSSROAD:
      tmpLongPredictPaths = MatchVerticalLongTermPath(pObj, pShortTermPath, pNowRoadSegRel,timeLength);
      break;
    default:
      break;
  }
  
  //---------step 2.2: calculate short and long path----------
  if (tmpLongPredictPaths.size()>0)
  {
    // switch (eNowObjIntention)
    {
      // case CHANGESEGMENT:
	for (int i = 0; i < tmpLongPredictPaths.size(); i++)
	{
	  ivpredict::predictobj initShortAndLongPrePath=shortTermPrePath;
	  ivpredict::predictpos tmpPos;
//for test
	  initShortAndLongPrePath.positions.clear();
	  tmpPos.xgmean=pObj->positions[pObj->positions.size()-1].xgmean;
	  tmpPos.ygmean=pObj->positions[pObj->positions.size()-1].ygmean;
	  for(int k=floor(dataNumForSmooth/2.0+1);k>=1;k--)
	  {
	    initShortAndLongPrePath.positions.push_back(tmpPos);
	  }
	  
//             for(int j=initShortAndLongPrePath.positions.size()-1;j>=3;j--)
//             {
//               initShortAndLongPrePath.positions.erase(initShortAndLongPrePath.positions.begin()+j);
//             }
//end
	  int longNum=tmpLongPredictPaths[i].positions.size();
	  for(int j=0;j<longNum;j++)
	  {
	    tmpPos.xgmean=tmpLongPredictPaths[i].positions[j].xgmean;
	    tmpPos.ygmean=tmpLongPredictPaths[i].positions[j].ygmean;
	    initShortAndLongPrePath.positions.push_back(tmpPos);
	  }
	  ivpredict::predictobj smoothedShortAndLongPath=SmoothPath(initShortAndLongPrePath,dataNumForSmooth); //filter IAPredict.objects[i].v_abs
	  ivpredict::predictobj smoothedVelPrePath=CalPathIncludeVelUseCurveB(smoothedShortAndLongPath,fitOrder,objVel,timeLength,timeStep); //consider velocity.
	  smoothedVelPrePath.id=pObj->id;
// 	  smoothedVelPrePath.isInvSearch=tmpLongPredictPaths[i].isInvSearch;
	  predictShortLongPaths.objects.push_back(smoothedVelPrePath);
	}
      // break;
    }
    
  }
  else
  {
    pShortTermPath->id=pObj->id;
    shortTermPrePath.positions.clear();
    ivpredict::predictpos tmpPos;
    tmpPos.xgmean=pObj->positions[pObj->positions.size()-1].xgmean;
    tmpPos.ygmean=pObj->positions[pObj->positions.size()-1].ygmean;
    for(int i=0;i<round(timeLength/timeStep);i++)
    {
      shortTermPrePath.positions.push_back(tmpPos);
    }
    predictShortLongPaths.objects.push_back(shortTermPrePath);
  }
  
  //-----------step 3: calculate path with probability and return--------------
  return predictShortLongPaths;
}

/**************************************************************
Function: PredictShortLongPath()
Author: zongwei
Date: 2017.05.27/2017.06.09/2017.06.11
Description: // 函数功能、性能等的描述
1. predicting shortLong paths.
2. -------
3. -------
Input:
1. pMsgShortPaths: pointer for short term paths
2. pMsgShortForProbalPaths: pointer for short term paths for 5s, with purpose of calculating probability.
3. pSMap: pointer for global road segment map.
4. pBufferObjInfo: pointer for buffer infomation.
Output: 
Return:
1. pMsgShortLongPaths: final predicted paths
Others: // 其他说明
************************************************************/
void PredictShortLongPaths(ivpredict::ivmsgpredict *pMsgShortLongPaths,ivpredict::ivmsgpredict *pMsgShortPaths,ivpredict::ivmsgpredict *pMsgShortForProbalPaths,sMapInformation *pSMap,ivpredict::ivpredictbuffer *pBufferObjInfo)
{
  pMsgShortLongPaths->objects.clear();
  
  //find nearest 6 objects need to predicts------------
  std::vector<int> nearest6Objs=Find6NearObjs(pBufferObjInfo,pSMap); 
  
  //predict
  for (int obji = 0; obji < pMsgShortPaths->objects.size(); obji++)
  {
    int bufferObjIndex=-1;
    double objNowXg,objNowYg;
    bool belongToNearObj=false;
    
    //find origin stored position of the object, in order to avoid mistaking invalid id.
    for(int k=pBufferObjInfo->objects.size()-1;k>=0;k--)
    {
      if(pBufferObjInfo->objects[k].id==pMsgShortPaths->objects[obji].id)
      {
	bufferObjIndex=k;
	int size=pBufferObjInfo->objects[bufferObjIndex].positions.size();
	objNowXg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].xgmean;
	objNowYg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].ygmean;
	break;
      }
    }
    for(int k=nearest6Objs.size()-1;k>=0;k--)
    {
      if(nearest6Objs[k]==pMsgShortPaths->objects[obji].id) {belongToNearObj=true;}
    }
    switch (pMsgShortPaths->objects[obji].type)
    {
      case 0:
      {
	//-----------------locate object to get current and corresponding son segments------------
	sRoadSegRel nowRoadSegRel = LocateObj(&pMsgShortPaths->objects[obji], pSMap, &pBufferObjInfo->objects[bufferObjIndex]);	
	
	//-----------------calculte critical values------------------------------------
	double dis=sqrt(pow(objNowXg-gEgoCarPosXg,2)+pow(objNowYg-gEgoCarPosYg,2));
	double crDis=60.0;//min(fabs(pMsgShortPaths->objects[obji].v_abs)*4, 50.0);
	double crVel=max(gEgoCarVg*0.3, gMinVelThreshold);
	double crCosAng=pBufferObjInfo->objects[bufferObjIndex].xvabs*sin(nowRoadSegRel.objHeading)+pBufferObjInfo->objects[bufferObjIndex].yvabs*cos(nowRoadSegRel.objHeading);
	crCosAng/=sqrt(pow(pBufferObjInfo->objects[bufferObjIndex].xvabs,2)+pow(pBufferObjInfo->objects[bufferObjIndex].yvabs,2));

	if(pMsgShortPaths->objects[obji].v_abs>crVel && crCosAng>cos(85.0/180.0*3.1415) && dis<=crDis && belongToNearObj==true)
	{
	    //---------------------------check motion intention of the object---------------------
	    eObjIntention eNowObjIntention=CheckObjIntention(&pMsgShortPaths->objects[obji],gMidVarialblesRecord.pSCurrentSegment,&nowRoadSegRel);
	    
	    //---------------------------predict shortlong path of the object(including time length and time step)---------------------
	    ivpredict::ivmsgpredict shortLongPathWithoutProbability=PredictObjPath(&nowRoadSegRel,&pBufferObjInfo->objects[bufferObjIndex],&pMsgShortPaths->objects[obji],eNowObjIntention,gTimeLength,gTimeStep);

	    //--------------------------calculate probability for each path-----------------------
	    ivpredict::ivmsgpredict shortLongPathWithProbability=AddProbabilityToShortLongPath(pMsgShortForProbalPaths->objects[obji], shortLongPathWithoutProbability);
	    if(shortLongPathWithProbability.objects.size()>1)
	    {
	      double maxP=0;
	      int indexI=-1;
	      for (int i = shortLongPathWithProbability.objects.size()-1; i>=0; i--)
	      {
		if(shortLongPathWithProbability.objects[i].probability>maxP)
		{
		  maxP=shortLongPathWithProbability.objects[i].probability;
		  indexI=i;
		}
	      }
	      for (int i = shortLongPathWithProbability.objects.size()-1; i>=0; i--)
	      {
		if(i!=indexI)
		{
		  shortLongPathWithProbability.objects.erase(shortLongPathWithProbability.objects.begin()+i);
		}
	      }
	    }
	    for (int i = 0; i < shortLongPathWithProbability.objects.size(); i++)
	    {
	      for(int num=shortLongPathWithProbability.objects[i].positions.size();num<=50;num++)
	      {
		shortLongPathWithProbability.objects[i].positions.push_back(shortLongPathWithProbability.objects[i].positions[num-1]);
	      }
	      pMsgShortLongPaths->objects.push_back(shortLongPathWithProbability.objects[i]);
	      
	      //fill predictobj_add msg.
	      ivpredict::predictobjdebug tmpObjDebug;
	      tmpObjDebug.id=pBufferObjInfo->objects[bufferObjIndex].id;
	      tmpObjDebug.type=nowRoadSegRel.objID;
	      tmpObjDebug.heading=nowRoadSegRel.objHeading;
	      tmpObjDebug.intention=eNowObjIntention;
	      debugMsgForRviz.objects.push_back(tmpObjDebug);
	      //end fill
	    }
	}
	else
	{
	  pMsgShortPaths->objects[obji].positions.clear();
	  ivpredict::predictpos tmpPos;
	  tmpPos.xgmean=objNowXg;
	  tmpPos.ygmean=objNowYg;
// 	  for(int i=round(gTimeLength/gTimeStep)-1;i>=0;i--)
// 	  {
	    pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
// 	  }
	  pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
	  
	  //fill predictobj_add msg.
	  ivpredict::predictobjdebug tmpObjDebug;
	  tmpObjDebug.id=pBufferObjInfo->objects[bufferObjIndex].id;
	  tmpObjDebug.type=nowRoadSegRel.objID;
	  tmpObjDebug.heading=nowRoadSegRel.objHeading;
	  debugMsgForRviz.objects.push_back(tmpObjDebug);
	  //end fill
	}
        break;
      }
      case 1:
      {
	//-----------------locate object to get current and corresponding son segments------------
	sRoadSegRel nowRoadSegRel = LocateObj(&pMsgShortPaths->objects[obji], pSMap, &pBufferObjInfo->objects[bufferObjIndex]);		
	
	//-----------------calculte critical values------------------------------------
	double dis=sqrt(pow(objNowXg-gEgoCarPosXg,2)+pow(objNowYg-gEgoCarPosYg,2));
	double crDis=60.0;//min(fabs(pMsgShortPaths->objects[obji].v_abs)*4, 50.0);
	double crVel=max(gEgoCarVg*0.3, gMinVelThreshold);
	double crCosAng=pBufferObjInfo->objects[bufferObjIndex].xvabs*sin(nowRoadSegRel.objHeading)+pBufferObjInfo->objects[bufferObjIndex].yvabs*cos(nowRoadSegRel.objHeading);
	crCosAng/=sqrt(pow(pBufferObjInfo->objects[bufferObjIndex].xvabs,2)+pow(pBufferObjInfo->objects[bufferObjIndex].yvabs,2));
	
	if(pMsgShortPaths->objects[obji].v_abs>crVel && crCosAng>cos(85.0/180.0*3.1415) && dis<=crDis && belongToNearObj==true)
	{	
	    //---------------------------check motion intention of the object---------------------
	    eObjIntention eNowObjIntention=CheckObjIntention(&pMsgShortPaths->objects[obji],gMidVarialblesRecord.pSCurrentSegment,&nowRoadSegRel);
	    
	    //---------------------------predict shortlong path of the object(including time length and time step)---------------------
	    ivpredict::ivmsgpredict shortLongPathWithoutProbability=PredictObjPath(&nowRoadSegRel,&pBufferObjInfo->objects[bufferObjIndex],&pMsgShortPaths->objects[obji],eNowObjIntention,gTimeLength,gTimeStep);

	    //--------------------------calculate probability for each path-----------------------
	    ivpredict::ivmsgpredict shortLongPathWithProbability=AddProbabilityToShortLongPath(pMsgShortForProbalPaths->objects[obji], shortLongPathWithoutProbability);
	    if(shortLongPathWithProbability.objects.size()>1)
	    {
	      double maxP=0;
	      int indexI=-1;
	      for (int i = shortLongPathWithProbability.objects.size()-1; i>=0; i--)
	      {
		if(shortLongPathWithProbability.objects[i].probability>maxP)
		{
		  maxP=shortLongPathWithProbability.objects[i].probability;
		  indexI=i;
		}
	      }
	      for (int i = shortLongPathWithProbability.objects.size()-1; i>=0; i--)
	      {
		if(i!=indexI)
		{
		  shortLongPathWithProbability.objects.erase(shortLongPathWithProbability.objects.begin()+i);
		}
	      }
	    }
	    for (int i = 0; i < shortLongPathWithProbability.objects.size(); i++)
	    {
	      pMsgShortLongPaths->objects.push_back(shortLongPathWithProbability.objects[i]);
	      
	      //fill predictobj_add msg.
	      ivpredict::predictobjdebug tmpObjDebug;
	      tmpObjDebug.id=pBufferObjInfo->objects[bufferObjIndex].id;
	      tmpObjDebug.type=nowRoadSegRel.objID;
	      tmpObjDebug.heading=nowRoadSegRel.objHeading;
	      tmpObjDebug.intention=eNowObjIntention;
	      debugMsgForRviz.objects.push_back(tmpObjDebug);
	      //end fill
	    }
	}
	else
	{
	  pMsgShortPaths->objects[obji].positions.clear();
	  ivpredict::predictpos tmpPos;
	  tmpPos.xgmean=objNowXg;
	  tmpPos.ygmean=objNowYg;
// 	  for(int i=round(gTimeLength/gTimeStep)-1;i>=0;i--)
// 	  {
	    pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
// 	  }
	  pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
	  
	  //fill predictobj_add msg.
	  ivpredict::predictobjdebug tmpObjDebug;
	  tmpObjDebug.id=pBufferObjInfo->objects[bufferObjIndex].id;
	  tmpObjDebug.type=nowRoadSegRel.objID;
	  tmpObjDebug.heading=nowRoadSegRel.objHeading;
	  debugMsgForRviz.objects.push_back(tmpObjDebug);
	  //end fill
	}
        break;
      }
      case 2:
      {
	//find origin stored position of the object, in order to avoid mistaking invalid id.
	for(int k=pBufferObjInfo->objects.size()-1;k>=0;k--)
	{
	  if(pBufferObjInfo->objects[k].id==pMsgShortPaths->objects[obji].id)
	  {
	    bufferObjIndex=k;
	    int size=pBufferObjInfo->objects[bufferObjIndex].positions.size();
	    objNowXg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].xgmean;
	    objNowYg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].ygmean;
	    break;
	  }
	}
	pMsgShortPaths->objects[obji].positions.clear();
	ivpredict::predictpos tmpPos;
	tmpPos.xgmean=objNowXg;
	tmpPos.ygmean=objNowYg;
	pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
	pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
        break;
      }
      case 3:
      {
	//find origin stored position of the object, in order to avoid mistaking invalid id.
	for(int k=pBufferObjInfo->objects.size()-1;k>=0;k--)
	{
	  if(pBufferObjInfo->objects[k].id==pMsgShortPaths->objects[obji].id)
	  {
	    bufferObjIndex=k;
	    int size=pBufferObjInfo->objects[bufferObjIndex].positions.size();
	    objNowXg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].xgmean;
	    objNowYg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].ygmean;
	    break;
	  }
	}
	pMsgShortPaths->objects[obji].positions.clear();
	ivpredict::predictpos tmpPos;
	tmpPos.xgmean=objNowXg;
	tmpPos.ygmean=objNowYg;
	pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
	pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
        break;
      }
      case 4:
      {
	//find origin stored position of the object, in order to avoid mistaking invalid id.
	for(int k=pBufferObjInfo->objects.size()-1;k>=0;k--)
	{
	  if(pBufferObjInfo->objects[k].id==pMsgShortPaths->objects[obji].id)
	  {
	    bufferObjIndex=k;
	    int size=pBufferObjInfo->objects[bufferObjIndex].positions.size();
	    objNowXg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].xgmean;
	    objNowYg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].ygmean;
	    break;
	  }
	}
	pMsgShortPaths->objects[obji].positions.clear();
	ivpredict::predictpos tmpPos;
	tmpPos.xgmean=objNowXg;
	tmpPos.ygmean=objNowYg;
	pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
	pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
        break;
      }
      case 5:
      {
	//find origin stored position of the object, in order to avoid mistaking invalid id.
	for(int k=pBufferObjInfo->objects.size()-1;k>=0;k--)
	{
	  if(pBufferObjInfo->objects[k].id==pMsgShortPaths->objects[obji].id)
	  {
	    bufferObjIndex=k;
	    int size=pBufferObjInfo->objects[bufferObjIndex].positions.size();
	    objNowXg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].xgmean;
	    objNowYg=pBufferObjInfo->objects[bufferObjIndex].positions[size-1].ygmean;
	    break;
	  }
	}
	pMsgShortPaths->objects[obji].positions.clear();
	ivpredict::predictpos tmpPos;
	tmpPos.xgmean=objNowXg;
	tmpPos.ygmean=objNowYg;
	pMsgShortPaths->objects[obji].positions.push_back(tmpPos);
	pMsgShortLongPaths->objects.push_back(pMsgShortPaths->objects[obji]);
        break;
      }    
      default:
      {
	break;
      }
    }
  } 
}

#endif
