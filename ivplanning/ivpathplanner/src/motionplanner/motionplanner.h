/****************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: ivpathplanner
* FileName: motionplanner.h, motionplanner.cpp
*
* Description:
* 1. Gengerate reasonable velocity for points on each refpath;
* 2. Select the most optimal refpath;
*
* History:
* Bo Yan    17/10/11    1.0.0    build this module.
****************************************************************/

#ifndef _MOTIONPLANNER_H
#define _MOTIONPLANNER_H
#pragma once

// c++ lib
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <tuple>
#include<algorithm>
// ros lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
//msg
#include "ivpredict/ivmsgpredict.h"
#include "ivpredict/predictpos.h"
#include "ivpredict/predictobj.h"
#include "ivpathplanner/pathpoint.h"
#include "ivpathplanner/path.h"
#include "ivpathplanner/ivmsgpathplanner.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/motionpoint.h"
#include "ivdecision/ivmsgdecision.h"
#include "ivdecision/ivmsgsegments.h"  //@pqg 0327
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/mapobject.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivmap/ivmapmsgstcell.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/staticcell.h"
#include "ivpathplanner/motiondebug.h"
#include "ivmap/ivmapmsguserfun.h"

#include "../../../../avoslib/globalvariables.h"
#include "motionplannercom.h"
#include "refpath.h" //redefine MIN MAX;

#include "mathtool.h"
#include "fuzzycontrol.h"
#include "kalmanfilter/oneDimKalman.h"


/*****sheng fu ***/
#include "../motionplanner//minijerk/spline.h"
#include "../motionplanner//minijerk/helper.h"
#include "../motionplanner//minijerk/jmt.h"
#include "../motionplanner//minijerk/behaviorplanner.h"
#include "../motionplanner//minijerk/pathdecision.h"
#include "../motionplanner//minijerk/velocitygeneration.h"
#include "../motionplanner/minijerk/lib/filter.h"
#include "../motionplanner/minijerk/lib/mathLib.h"
#include "../motionplanner/minijerk/lib/visualization.h"
#include "ivpathplanner/velocitydebug.h"


#define SPEED_STOP 0.0
#define VELOCITY_INVALID -88.0
#define INVALIDDOUBLE 8888.8888
#define INVALIDINT 8888
#define CARVE_INIT -0.1
# define ACCELERATION_INIT 2.2
#define PATH_MAP 0
#define PATH_AVOID 1
#define PATH_MARK 1
#define ROI_FRONT 0
#define ROI_FRONTPRED 1
#define ROI_FRONTL 2
#define ROI_FRONTR 3
#define PARKING 1
#define DRIVE 0

// #define MAX(a, b) ((a) >= (b)? (a) : (b))
// #define MIN(a, b) ((a) <= (b)? (a) : (b))


using namespace std;

typedef struct sObjPos
{
    int posid;
    double dist;
    double time_inter;
} sObjPos;

typedef struct sObjFocus
{
    int path_id;
    int roi_id;
    ivpredict::predictobj objfocus;
} sObjFocus;

typedef struct sobjkeyInfo
{
    double targetAcceleration;
    int targetIndex;
    int targetPreIndex;
    int id;
    int objType;
} sobjkeyInfo;

typedef struct sObjTracker
{
    int cnt_lost;
    vector<sObjFocus> ObjpreTrack;
} sObjTrack;


// the class for motionplan.
class motionplanner
{
public:
    int pathID;
    ivpathplanner::ivmsgpath finalPath;
    ivpathplanner::motiondebug motionVars;
    /******fusheng********/
    sPoint Max_curvature;
    vector<sPoint> Max_path_curvature;
    vector<sPoint> output_curvature;
    ivpathplanner::velocitydebug velocity_bug;

    double now_target_v;
    RoadType roadType;
    double g_start_v;
    BehaviorPlanner planner;
    PathDecision pathDecision;
    VelocityGeneration velocityGeneration;
    vector<double> targetPosVel;
    double egoVelocity;
    doubleVector pathStopFlag;
    ivdecision::ivmsgdecision decisions;
    std::vector<std::vector<sPoint> >  motOptionPathPoints;

public:
    motionplanner();
    ~motionplanner();
    void InitMotionplannerParam();
    void MotionPlanning();
    void GetMsg(const ivpathplanner::ivmsgpathplanner &optionPathsIn,
                const ivpredict::ivmsgpredict &objPredIn,
                const ivmap::ivmapmsguserfun &objVirtualIn,
                const ivmap::ivmapmsgobj &objFusionIn,
                const ivdecision::ivmsgdecision &decisionIn,
                const ivmap::ivmapmsgvap &ivvapIn,
                int currentPathDirection);//@pqg 0327 add ivmsgsegments


    // void GetBasesignals(ivmap::ivmapmsgvap &carStatus);
    // void GetDecision(const ivdecision::ivmsgdecision &decision, bool &darPchange);
    // int SelectPath(const std::vector<std::vector<sPoint> > &optionPathPoints);
    // void GeneratePath(std::vector<std::vector<sPoint> > &optionPathPoints);
    // void generateVelocity(std::vector<std::vector<sPoint> >  &optionPathPoints, ivmap::ivmapmsgvap &carStatus, ivpredict::ivmsgpredict &obs);
    // void GenVelocityBasedObj(std::vector<std::vector<sPoint> > &optionPathPoints,
    // ivpredict::ivmsgpredict &objDynamic,
    // const ivmap::ivmapmsgstcell &objStatic);

    // void GetBasesignals(ivmap::ivmapmsgvap &carStatus);
    // void GenVelocityVirtualObj(std::vector<std::vector<sPoint> > &optionPathPoints, const ivmap::ivmapmsguserfun &objVirtual);

private:
    void RawPathProcess(const ivpathplanner::ivmsgpathplanner &optionPathsRaw);
    void TrafficFreePlanning(std::vector<std::vector<sPoint> > &optionPathPoints);
    void GenVelocityBasedRoadMark(std::vector<sPoint> &PathPoints);
    void GenVelocityBasedRoad(std::vector<sPoint> &PathPoints);
    void GenVelocityBasedRoadOld(std::vector<sPoint> &PathPoints);
    void AnalysisCurve(std::vector<sPoint> &PathPoints);
    int SelectPath(const std::vector<std::vector<sPoint> > &optionPathPoints);
    void GeneratePath(std::vector<std::vector<sPoint> > &optionPathPoints);
    void generateVelocity(std::vector<std::vector<sPoint> >  &optionPathPoints, ivmap::ivmapmsgvap &carStatus, ivpredict::ivmsgpredict &obs);

    void GenVelocityByAccel(std::vector<sPoint> &PathPoints, sobjkeyInfo objInfo);

    void TrafficBasedPlanning(std::vector<std::vector<sPoint> > &optionPathPoints, double egoVelocity, ivpredict::ivmsgpredict &objPredict);
    void TrafficBasedPlanning(std::vector<std::vector<sPoint> > &optionPathPoints, ivpredict::ivmsgpredict &objPredict, double egoVelocity);
    void FittingCurve(std::vector<sPoint> &PathPoints);
    void FittingCurve(std::vector<std::vector<sPoint> > &optionPathPoints);
    void AddPathInfo(const ivpathplanner::ivmsgpathplanner &optionPathsRaw);

    void SmoothVelocity(std::vector<sPoint> &PathPoints);
    void FocusingObjs(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict &objsP, double egoSpeed, int pathNum);
    void SelectKeyObjs(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict objs, int pathNumber);
    void CollectObjsROI(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict objs, int pathNumber);
    double GetTHW(std::vector<sPoint> &PathPoints, int posId);
    void TrackingKeyObjs(std::vector<sPoint> &PathPoints, std::vector<sObjFocus> &keyObjs);
    void GenVelocityTrafficBased(std::vector<sPoint> &PathPoints, std::vector<sObjFocus> &keyObjs, double egoSpeed, int pathNum);
    void GenVelocityAC(std::vector<sPoint> &PathPoints, std::vector<sObjFocus> &Objs, double egoSpeed, int pathNum);
    sobjkeyInfo GenAccelerationAC(std::vector<sPoint> &PathPoints, sObjFocus &Obj, double egoSpeed, int pathNum);
    sobjkeyInfo GenAccelerationFC(std::vector<sPoint> &PathPoints, sObjFocus &Obj, double egoSpeed, int pathNum);
    int GenAccelModeSwitch(double collosionTime, double dist, double distPreview);
    double CalcuVelocityObjS(double dist, double objSpeed, double egoSpeed);
    int GetPrePointIndex(int collisionIndex, int startIndex, double distPre, std::vector<sPoint> &PathPoints);
    double GetFollowingTime(double targetRelspeed);
    double CalcuDistSafe(double distParam, double distMinParam, double egoSpeed, double targetRelspeed);
    double CalcuAccel2Points(double deltaTime, int PointIndex, double speedInit, std::vector<sPoint> &PathPoints);

    sObjPos CalcuObjPos(double x, double y, std::vector<sPoint> &pathPointss);
    double CalcuPassWidth(double distanceObj, double relspeedObj);
    double THW_following(double dist, double egospeed, double distSafe, double THW0, double relspeed);
    double SmoothAcceleration(int filterCnt, double newAcceleration);
    void CalFuzzyInitSpeed(double accelFuzzy, double egoSpeed);
    double SmoothAcceleration1(int filterCnt, double newAcceleration);
    double FosusTimeInterval();
    double SbjspeedFilter(double egospeed);
    void GenVelocityVirtualObj(std::vector<sPoint> &PathPoints, const ivmap::ivmapmsguserfun &objVirtual, double egospeed);
    /******fu sheng******/
    // double convert_jmts_to_originals(vector<State> &jmts, const double originals) const;
    // RoadType pathDecision(std::vector<std::vector<sPoint> >  &optionPathPoints,  const double &ego_speed, vector<double> &spline_value);
    // int startAcceleration(std::vector<std::vector<sPoint> > const &optionPathPoints, double const &current_velocity_target, double const temp_min_a, double const temp_max_a,  double &s_start_a );


    doubleVector GetCoeff(const vector<sPt> &sample, int n);
    double GetDist(double x0, double y0, double x1, double y1);
    double StoreAccel(double Accel);
    double StoreSpd(double spd);
    double CalculationAccelCar(double spd,double spd_last);

    void SmoothCurve(std::vector<sPoint> &PathPoints);   //TODO @pqg 0322

private:
    int TrafficFreePlanningMode;
    int TrafficBasedPlanningMode;

    /*******virtual planning******/
    double xg_PointS1;
    double yg_PointS1;
    double xg_PointS2;
    double yg_PointS2;
    double xg_PointS3;
    double yg_PointS3;
    double xg_PointS4;
    double yg_PointS4;
    double xg_PointE1;
    double yg_PointE1;
    double xg_PointE2;
    double yg_PointE2;
    double xg_PointE3;
    double yg_PointE3;
    double xg_PointE4;
    double yg_PointE4;
    double speed_upper;
    double speed_lower;
    bool speed_init;
    bool speedUpdate;
    double accelPlusMax;
    double accelMinusMax;
    double pathPreview;
    int pathEndStopNum;
    bool emergStatus;
    double lengthPreview;

    double radiusMin;

    int startPointIndex;
    double passWidthLower;
    double passWidthUpper;
    double carLength;
    double timePred;
    double blindsScope;
    double pathPreviewMax;
    double pathLength;
    double pathLengthLast;
    bool pathLengthChange;
    int genAccelMode;
    bool followingObjChange;
    double distPreObj;
    double distPreObjMin;
    double distSafe;
    double tableCurvature1[10] =  {0, 0.002, 0.005, 0.008, 0.01, 0.02, 0.05, 0.1, 0.2, 1};
    // double tableVelocity1[10] = {40, 40, 40, 40, 30, 30, 20, 15, 15, 0};
    double tableVelocity1[10] = {30, 30, 30, 30, 20, 20, 10, 6, 6, 0};
    bool VelocityReplanObj;
    double fuzzyAccelLast;
    double fuzzyInitSpeed;
    double egoSpeedLast;
    
    double sbjSpeed_last_for_filter;
    double speedlost_cnt;
    double sbjSpeed;
    double vehSpd; //0131
    double vehSpd_last;
    double accelCar;
    double speedObj;
    double speedObj_last;
    int pathDirection;
    int shiftPosReq;

    //test0223

    double crossSpeedBase;
    double crossSpeedBase_last;
    int objExistFlag;  //@pqg 0322


    sPoint motPoint;
    std::vector<double> accelSmooth;
    std::vector<double> deltaSpeedSmooth;
    std::vector<double> carAccel;
    std::vector<sPoint>  motPathPoints;

    std::vector<sObjFocus> objsFocus;
    std::vector<sObjFocus> objsHistory;
    std::vector<sObjFocus> trackObjs;
    // std::vector<sObjTrack> keyObjsTrack;
    sObjTrack keyObjsTrack[255];

    std::vector<sPoint> motionPlannerBackup;  //@pqg 0320

    doubleVector pathId; // 0: map; 1: genpath for lanechange or avoid
    sobjkeyInfo followingObj_last;


    ivpathplanner::motionpoint motionPoint;

    ivpredict::ivmsgpredict objsROI;

    ivmap::ivmapmsgvap ivvap;
    ivpredict::ivmsgpredict objPred;
    ivmap::ivmapmsguserfun objVirtual;
    ivmap::ivmapmsgobj objFusion;
    ivdecision::ivmsgdecision decision;
    ivpathplanner::ivmsgpathplanner optionPaths;

    mathtool matool;
    fuzzycontroller fuzzycontrol;

    lpf_1p_param param;


    OneDimKalman *pkalman;
    OneDimKalman *pkalman01;
    OneDimKalman *pkalman02;
    OneDimKalman *pkalman03;
    OneDimKalman *pkalman04;
    std::vector<double> v_filter;
    std::vector<double> v_filter01;
    std::vector<double> v_filter02;
    std::vector<double> v_filter03;
    std::vector<double> v_filter04;


    /****fu sheng***/
    vector<double> curveValue;;
    mean_filter_param mf1;
    mean_filter_param filter_uturn_size;
    Visualization *visualization;
    Visualization *visualization_long;
    bool lidarSegUpdate;



};

#endif //_MOTIONSPLANNER_H
