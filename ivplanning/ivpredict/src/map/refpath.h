//
// Created by idriver on 17-3-23.
//

#ifndef PREDICT_REFPATH_H
#define PREDICT_REFPATH_H

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "ivpathplanner/path.h"
#include "ivpathplanner/pathpoint.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivmap/ivmapmsgroad.h"
#include <geometry_msgs/Point32.h>
#include "../../../../avoslib/geotool.h"
#include "../../../../avoslib/iabasemaptool.h"
#include "../../../../avoslib/globalvariables.h"
#include "ivlocmsg/ivsensorgps.h"
#include "../globaldefine.h"
#include "geotoolfunc.h"
// #include "../premethods/predictrelline.h"
// #include "../avoidObsModel/cbp/functions.h"

//c++ lib
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/topic.h"
#include <sstream>
#include <iostream>
#include <fstream>

#define INVALID_SPEED -88
#define MIN_PATH_PTS 10

using namespace std;

namespace prediction{
typedef struct sMapInfo
{
    double lat;
    double lon;
    double xg;
    double yg;
    double heading;
    double angle;
    double velocity;
}sMapInfo;

typedef struct sMaps
{
    vector<sMapInfo> pathSeg;
    bool forwardBackwardFlag;
}sMaps;


class refpath
{
public:
    refpath();
    ~refpath();

    //Variables
    vector<sMapInfo> pathPoints;
    vector<sMapInfo> pathPointsRaw;
    vector<sMaps> allRoads;
    std::vector<int> pathSegsLeft;

    // Data Process
    void LocalRefpathBuild(ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos);
    void Rasterization(double disFront, double disBack, ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos);
    void VelocityUpdate(ivpathplanner::ivmsgpath motionPath);
    void MapRecovery();
    // void LocalPathToMap(ivmap::ivmapmsglocpos carPosition, ivpathplanner::ivmsgpath IAPath);
    void RawMapImprove(std::vector<sMapInfo> segment);

    //tool function
    double IncludeAngle(double angle1, double angle2);

    //connect paths to one smooth path
    void LoadAllMaps(int numPaths);
    vector<sMapInfo> LoadMap(string mapRoute, int roadId);
    void PathSegsToSmoothPath(vector< int > pathSegIds);
    vector<sMapInfo> SmoothTwoPaths(vector<sMapInfo> pathSeg1, vector<sMapInfo> pathSeg2);
    double GetSegDis(vector<sMapInfo> pathSeg, int startId, int endId);
    int GetPointIdBaseDis(vector<sMapInfo> pathSeg, int startId, double distance, int searchSequency);
    double DisToPath(sMapInfo pointOnPath, double xPointDis, double yPointDis);
    int FindNearestPointId(vector<sMapInfo> pathSegNearest, double xNear, double yNear, double angleNear, int searchSequency);
    vector<sMapInfo> PathMoveGenerate(vector<sMapInfo> pathSmoothSeg, double distanceMove, double distanceSmoothSeg, int startSmoothPointId, int endSmoothPointId);
    sMapInfo PointLeftRight(double xlr, double ylr, double anglelr, double dislr);
    vector<sMapInfo> AngleUpdate(vector<sMapInfo> pathMoveOut, int numCurveStartId, int numPointsCurve);
    bool PointMoveValid(sMapInfo pointOnPath1, sMapInfo pointMove2, double disMoveFinal, double disMoveLastPoint);
    double PathDirection(double roadPointDirection);
    int Signn(double bValue);
    double LineDirection(double xSecond, double ySecond, double xFirst, double yFirst);
    double UpdatePointAngle(vector<sMapInfo> pathAngleUpdate, int ist);
    // void LocalPathMap(ivmap::ivmapmsglocpos carPosition, ivpathplanner::path newPath);
	void mapOrigin();
    vector<sMapInfo> GetMorePoints(vector<sMapInfo> eachRoad);
    void UpdatePathPoints(vector<sMapInfo> newPathPoints);

    geotool gt;
    iabasemaptool mapTool;

    ivpredict::predictobj SmoothPathx_absline(ivpredict::predictobj initPrePath, int dataNumForSmooth);
    ivpredict::predictobj CalPathIncludeVelUseCurveBx_absline(ivpredict::predictobj initPrePath,int fitOrder,double objVel,double timeLength,double timeStep);

private:

    bool roadEnd;
    int iStartIndex;
    int iEndIndex;
    int lastIndex;
    geotoolfunc geo_function_path;
};
}

#endif //PROJECT_REFPATH_H
