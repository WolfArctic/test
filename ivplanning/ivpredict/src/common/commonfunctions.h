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

#ifndef _COMMONFUNCTIONS_H
#define _COMMONFUNCTIONS_H
#pragma once

#include "../globaldefine.h"
#include "../glovariable.h"
class commonfunctions
{
public:
    commonfunctions();
    ~commonfunctions();
    double min(double a, double b);
    double max(double a, double b);
    float CalPoint2TargetSegmentDis(float x, float y, ivpredict::predictobj *pTargetsegment);
    float CalDisBetweenTwoSegments(ivpredict::predictobj targetsegment, ivpredict::predictobj shortPredict);
    int CalNearestSegmentPointIndex(float x, float y, sSegRel *pTmpSegment);
    float CalPoint2SegmentDis(double xg, double yg, sSegRel *pTmpSegment);
    float calTotalDis(sSegRel targetsegment, ivpredict::predictobj *pShortPredictPath_dis);
    void CrossVector(double x1,double y1,double z1,double x2,double y2,double z2,double *pResultX, double *pResultY,double *pResultZ);
    void FitCurveLeastSquare(double *X,double *Y,int xyNum,int fitOrder,double *a);
    void CrossMatrix(double **A,double **B,double **C,int lineNumA,int lineNumB,int rowNumB);
    void AddMatrix(double **A, double **B, double **C);
    void Transpose(double **A, double **B);
};

#endif