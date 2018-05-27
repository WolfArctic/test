/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: commonfunctions.h
Description:
1. common functions for being used by two or more other documents.
2. -------
3. -------
History:
<author>	<time>		<version>	<description>
Zongwei		2017年06月12日	V1.1		build this module
************************************************************/

#include "commonfunctions.h"


commonfunctions::commonfunctions()
{};
commonfunctions::~commonfunctions()
{};
double commonfunctions::min(double a, double b)
{
  return (a>b?b:a);
}
double commonfunctions::max(double a, double b)
{
  return (a>b?a:b);
}

/**************************************************************
Function: CalPoint2TargetSegmentDis()
Author: lihui/zongwei
Date: 2017.4.19/2017.6.11
Description: // 函数功能、性能等的描述
1. vertical distance betweeen the point and target segment.
2. target segment is type of msg ivpredict::predictobj.
3. -------
Input:
1. x,y: coordinates of the point.unit: m
2. pTargetsegment: target segment.
Output: nothing
Return:
1. distance. unit:m
Others: // 其他说明
************************************************************/
float commonfunctions::CalPoint2TargetSegmentDis(float x, float y, ivpredict::predictobj *pTargetsegment)
{
  float pointToSegmentDis = 10000;
  for (int j = 0; j < pTargetsegment->positions.size(); j++)
  {
    float tempDis = sqrt( pow(pTargetsegment->positions[j].xgmean-x , 2)+ pow(pTargetsegment->positions[j].ygmean-y , 2));
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
    }
  }
  return pointToSegmentDis;
}

/**************************************************************
Function: CalDisBetweenTwoSegments()
Author: lihui/zongwei
Date: 2017.4.19
Description: // 函数功能、性能等的描述
1. vertical distance betweeen the two target segments.
2. -------
3. -------
Input:
1. two segments.
Output: nothing
Return: 
1. distance. unit:m
Others: // 其他说明
************************************************************/
float commonfunctions::CalDisBetweenTwoSegments(ivpredict::predictobj targetsegment, ivpredict::predictobj shortPredict)
{
  float totalDis = 0;
  for (int i = 0; i < shortPredict.positions.size(); i++)
  {
    totalDis += CalPoint2TargetSegmentDis(shortPredict.positions[i].xgmean, shortPredict.positions[i].ygmean, &targetsegment);
    // cout << totalDis << endl;
  }
  return totalDis; 
}

/**************************************************************
Function: CalNearestSegmentPointIndex()
Author: zongwei
Date: 2017.6.13
Description: // 函数功能、性能等的描述
1. find nearest point index from given cordinate on target segment
2. -------
3. -------
Input:
1. x,y: coordinate of the given point with unit m
2. pTmpSegment: target segment
Output:
Return:
1. point index on target segment
Others: // 其他说明
************************************************************/
// typedef struct sSegRel{
//   string segmenttype;
//   int segmentnumber;
//   double possibility;
//   int maxLineNum;
//   std::vector<sGPSpoint> GPS;
//   std::vector<sGCCSpoint> GCCS;
//   std::vector<sSegRel> subSeg;
//   bool isInvSearch;
// }sSegRel;
int commonfunctions::CalNearestSegmentPointIndex(float x, float y, sSegRel *pTmpSegment)
{
  float pointToSegmentDis = 10000;
  int pointIndex =-1;
  int stepLength=5;
  int dataNum=(int)((pTmpSegment->GCCS).size());
  for (int j = 0; j < dataNum; j = j+stepLength)
  {
    float tempDis = sqrt( pow(pTmpSegment->GCCS[j].xg-x,2) + pow(pTmpSegment->GCCS[j].yg-y,2));
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      pointIndex=j;
    }
  }
  int minJ=(pointIndex-stepLength<=0)          ?0          :(pointIndex-stepLength);
  int maxJ=(pointIndex+stepLength>=(dataNum-1))?(dataNum-1):(pointIndex+stepLength);
  
  for (int j = minJ; j <= maxJ; j++)
  {
    float tempDis = sqrt( pow(pTmpSegment->GCCS[j].xg-x,2) + pow(pTmpSegment->GCCS[j].yg-y,2));
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      pointIndex=j;
    }
  }

  return pointIndex;
}

/**************************************************************
Function: CalPoint2SegmentDis()
Author: lihui/zongwei
Date: 2017.4.19/2017.6.11
Description: // 函数功能、性能等的描述
1. vertical distance betweeen the point and target segment.
2. target segment is type of sSegRel.
3. -------
Input:
1. xg,yg: coordinates of the point.unit: m
2. pTmpSegment: target segment.
Output: nothing
Return: distance with unit m
Others: // 其他说明
************************************************************/
float commonfunctions::CalPoint2SegmentDis(double xg, double yg, sSegRel *pTmpSegment)
{
  double pointToSegmentDis = 10000.0;
  int step=10;
  long tempIndex=-1;
  long GCCSsize=pTmpSegment->GCCS.size();
  for (int j = 0; j < GCCSsize; j += step)
  {
    double tempDis = sqrt(pow(pTmpSegment->GCCS[j].xg - xg, 2) +  pow(pTmpSegment->GCCS[j].yg - yg, 2)) ;
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      tempIndex=j;
    }
  }
  
  int startNum=(tempIndex-step)>=0 ? (tempIndex-step) : 0;
  int   endNum=(tempIndex+step)<=(GCCSsize-1) ? (tempIndex+step) : (GCCSsize-1);
  for (int j = startNum; j <= endNum; j++)
  {
    double tempDis = sqrt(pow(pTmpSegment->GCCS[j].xg - xg, 2) +  pow(pTmpSegment->GCCS[j].yg - yg, 2)) ;
    if (tempDis < pointToSegmentDis)
    {
      pointToSegmentDis = tempDis;
      tempIndex=j;
    }
  }  

  if(tempIndex>0 && tempIndex<(GCCSsize-1))
  {
    double ang1=atan2(yg-pTmpSegment->GCCS[tempIndex].yg , xg-pTmpSegment->GCCS[tempIndex].xg);
    double ang2=atan2(pTmpSegment->GCCS[tempIndex+1].yg-pTmpSegment->GCCS[tempIndex].yg,pTmpSegment->GCCS[tempIndex+1].xg-pTmpSegment->GCCS[tempIndex].xg);
    pointToSegmentDis*=fabs(sin(ang1-ang2));
  }
  
  return pointToSegmentDis;
}

/**************************************************************
Function: calTotalDis()
Author: lihui/zongwei
Date: 2017.4.19
Description: // 函数功能、性能等的描述
1. vertical distance betweeen the two target segments.
2. -------
3. -------
Input:
1. two segments.
Output: nothing
Return: 
1. distance. unit:m
Others: // 其他说明
************************************************************/
float commonfunctions::calTotalDis(sSegRel targetsegment, ivpredict::predictobj *pShortPredictPath_dis)
{
  float totalDis = 0;
  // cout << " short position size" << pShortPredictPath_dis->positions.size() << endl;
  for (int i = 0; i < pShortPredictPath_dis->positions.size(); i++)
  {
    // totalDis += CalPoint2TargetSegmentDis(pShortPredictPath->positions[i].xgmean, pShortPredictPath->positions[i].ygmean, &targetsegment);
    totalDis += CalPoint2SegmentDis(pShortPredictPath_dis->positions[i].xgmean, pShortPredictPath_dis->positions[i].ygmean, &targetsegment); 
    if (totalDis > 1000)
    {
      totalDis = 1000;
    }  
    ;
  }
  if (totalDis > 1000)
  {
    totalDis = 1000;
  }
  // cout << "total dis of " << targetsegment.segmentnumber << " is " << totalDis <<  endl;
  return totalDis; 
}



void commonfunctions::CrossVector(double x1,double y1,double z1,double x2,double y2,double z2,double *pResultX, double *pResultY,double *pResultZ)
{
  *pResultX=y1*z2-y2*z1;
  *pResultY=x2*z1-x1*z2;
  *pResultZ=x1*y2-y1*x2;
}

/**************************************************************
Function: FitCurveLeastSquare
Author: zongwei
Date: 17/06/23
Description: 
Input: 
Output:
Return:
Others: 
************************************************************/
void commonfunctions::FitCurveLeastSquare(double *X,double *Y,int xyNum,int fitOrder,double *a)
{
  if(fitOrder==2)	//y=a[2]*x^2+a[1]*x+a[0]
  {
    double A[3][3]={0};
    double B[3]={0};
    
    for(int i=0;i<xyNum;i++)
    {
        A[0][0]=xyNum;
        A[0][1]+=X[i];
        A[0][2]+=X[i]*X[i];
        A[1][0]+=X[i];
        A[1][1]+=X[i]*X[i];
        A[1][2]+=X[i]*X[i]*X[i];
        A[2][0]+=X[i]*X[i];
        A[2][1]+=X[i]*X[i]*X[i];
        A[2][2]+=X[i]*X[i]*X[i]*X[i];
        B[0]+=Y[i];
        B[1]+=X[i]*Y[i];
        B[2]+=X[i]*X[i]*Y[i];
    }   
    
    double detA=A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1];
    detA+=-A[0][0]*A[1][2]*A[2][1]-A[0][1]*A[1][0]*A[2][2]-A[0][2]*A[1][1]*A[2][0];      
    
    double invA[3][3];
    invA[0][0]=(A[1][1]*A[2][2]-A[1][2]*A[2][1])/detA;
    invA[0][1]=-(A[1][0]*A[2][2]-A[1][2]*A[2][0])/detA;
    invA[0][2]=(A[1][0]*A[2][1]-A[1][1]*A[2][0])/detA;
    invA[1][0]=-(A[0][1]*A[2][2]-A[0][2]*A[2][1])/detA;
    invA[1][1]=(A[0][0]*A[2][2]-A[0][2]*A[2][0])/detA;
    invA[1][2]=-(A[0][0]*A[2][1]-A[0][1]*A[2][0])/detA;
    invA[2][0]=(A[0][1]*A[1][2]-A[0][2]*A[1][1])/detA;
    invA[2][1]=-(A[0][0]*A[1][2]-A[0][2]*A[1][0])/detA;
    invA[2][2]=(A[0][0]*A[1][1]-A[0][1]*A[1][0])/detA;   

    int lineNumA=sizeof(A)/sizeof(A[0]);
    int  rowNumA=sizeof(A[0])/sizeof(A[0][0]);
    for(int i=0;i<lineNumA;i++)
    {
      a[i]=0;
      for(int j=0;j<rowNumA;j++)
      {
	a[i]+=invA[i][j]*B[j];
      }
    }
  }
  
  if(fitOrder==1)	//y=a[1]*x+a[0]
  {
    double A[2][2]={0};
    double B[2]={0};
    for(int i=0;i<xyNum;i++)
    {
        A[0][0]=xyNum;
        A[0][1]+=X[i];
        A[1][0]+=X[i];
        A[1][1]+=X[i]*X[i];
        B[0]+=Y[i];
        B[1]+=X[i]*Y[i];
    }       
    double detA=A[0][0]*A[1][1]-A[0][1]*A[1][0];    
    double invA[2][2];
    invA[0][0]=A[1][1]/detA;
    invA[0][1]=-A[1][0]/detA;
    invA[1][0]=-A[0][1]/detA;
    invA[1][1]=A[0][0]/detA;     
    int lineNumA=sizeof(A)/sizeof(A[0]);
    int  rowNumA=sizeof(A[0])/sizeof(A[0][0]); 
    a[2]=0;
    for(int i=0;i<lineNumA;i++)
    {
      a[i]=0;
      for(int j=0;j<rowNumA;j++)
      {
	a[i]+=invA[i][j]*B[j];
      }
    }    
  }
  return;
}

/**************************************************************
Function: CrossMatrix
Author: zongwei
Date: 17/06/23
Description: 
Input: 
Output:
Return:
Others: 
************************************************************/
void commonfunctions::CrossMatrix(double **A,double **B,double **C,int lineNumA,int lineNumB,int rowNumB)
{
    int  rowNumA=lineNumB;
    int lineNumC=lineNumA;
    int  rowNumC= rowNumB;
   
   for(int i=0;i<lineNumA;i++)
   {
       for(int j=0;j<rowNumB;j++)
       {     
           double tmpCij=0;     
           for(int k=0;k<rowNumA;k++)
           {    
               tmpCij+=(*((double *)A+i*rowNumA+k)) * (*((double *)B+k*rowNumC+j));
           }
           *((double *)C+i*rowNumC+j)=tmpCij;
       }
   }
}

/**************************************************************
Function: AddMatrix
Author: zongwei
Date: 17/06/23
Description: 
Input: 
Output:
Return:
Others: 
************************************************************/
void commonfunctions::AddMatrix(double **A, double **B, double **C)
{
  int lineNum=sizeof(A)/sizeof(A[0]);
  int  rowNum=sizeof(A[0])/sizeof(A[0][0]);
  for(int i=0;i<lineNum;i++)
  {
    for(int j=0;j<rowNum;j++)
    {
      C[i][j]=A[i][j]+B[i][j];
    }
  }
  return;
}

/**************************************************************
Function: AddMatrix
Author: zongwei
Date: 17/06/23
Description: 
Input: 
Output:
Return:
Others: 
************************************************************/
void commonfunctions::Transpose(double **A, double **B)
{
  int lineNum=sizeof(A)/sizeof(A[0]);
  int  rowNum=sizeof(A[0])/sizeof(A[0][0]);
  for(int i=0;i<lineNum;i++)
  {
    for(int j=0;j<rowNum;j++)
    {
      B[i][j]=A[j][i];
    }
  }
  return;
}
