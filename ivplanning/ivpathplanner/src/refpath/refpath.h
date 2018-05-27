//
// Created by idriver on 17-3-23.
//

#ifndef PROJECT_REFPATH_H
#define PROJECT_REFPATH_H

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

typedef struct sMapInfo
{
    double lat;
    double lon;
    double xg;
    double yg;
    double heading;
    double angle;
    double velocity;
    double a;
    double t;
    double curvature;
    int loncontrolmode;
    int latcontrolmode;
}sMapInfo;

typedef struct sSegRelationRefPath
{
    int segId;
    double segLength;
    double speed;
    std::vector<int> nextSegIds;
    int leftSegId;
    int rightSegId;
    int adaptiveSpeed;
    int backwardFlag;
}sSegRelationRefPath;

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
    void LocalRefpathBuild(ivmap::ivmapmsglocpos tempGps, double disFront, double disBack, ivpathplanner::path& localRef, double carSpeed);
    void Rasterization(ivmap::ivmapmsglocpos carPosition, double disFront, double disBack, ivpathplanner::path& matrixSend, double carSpeed);
    void VelocityUpdate(ivpathplanner::ivmsgpath motionPath);
    void MapRecovery();
    void LocalPathToMap(ivmap::ivmapmsglocpos carPosition, ivpathplanner::ivmsgpath IAPath);
    void RawMapImprove(std::vector<sMapInfo> segment);

    //tool function
    double IncludeAngle(double angle1, double angle2);

    //connect paths to one smooth path
    void LoadAllMaps(int numPaths);
    vector<sMapInfo> LoadMap(string mapRoute, int roadId, double segSpeedLimit);
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
    void LocalPathMap(ivmap::ivmapmsglocpos carPosition, ivpathplanner::path newPath);
	void mapOrigin();
    vector<sMapInfo> GetMorePoints(vector<sMapInfo> eachRoad);
    void UpdatePathPoints(vector<sMapInfo> newPathPoints);
	void LoadSegsRelation();
    void UpdateSegVelocity(double appSpeed);

    geotool gt;
    iabasemaptool mapTool;

private:

    bool roadEnd;
    int iStartIndex;
    int iEndIndex;
    int lastIndex;
    
    
public:
    int globalindexNearest;
    int globaliStartIndex;
    int globaliEndIndex;
    double globaldToVehicleMin;
    double globaldisFront;
    double globaldisFrontAll;
	std::vector<sSegRelationRefPath> segRelations;

};

#endif //PROJECT_REFPATH_H
