//
// Created by idriver on 17-3-23.
//

#include "refpath.h"

using namespace std;
namespace prediction{
refpath::refpath()
{
    ros::NodeHandle nh;
    lastIndex = 0;
    iStartIndex = 0;
    iEndIndex = 0;
    roadEnd = false;
}

refpath::~refpath()
{

}


void refpath::LocalRefpathBuild(ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos)
{
    if (pathPoints.size() > MIN_PATH_PTS )
    {
        Rasterization(50, 10, predict_result, pIvlocpos);
        predict_result->heading = pIvlocpos->heading;
        predict_result->xvrel = pIvlocpos->xg;
        predict_result->yvrel = pIvlocpos->yg;
        // *predict_result=CalPathIncludeVelUseCurveBx_absline(*predict_result,3,5,5,0.01);
        geo_function_path.PredictGCC2VCS_one(predict_result, pIvlocpos);       
    }
    // localRef.stopflag = false;
}

void refpath::Rasterization(double disFront, double disBack, ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos) //matrixSend
{
    double distance = 0;
    double delta = 0.1;
    // delta = disFront/50.0;

    double anglePointVehicle = 0;
    double dToVehicleMin = 88888.0;
    double dPointVehicle = 88888.0;
    int indexNearest = lastIndex;
    // find nearest point index
    for (long i = lastIndex; i < pathPoints.size(); ++i)
    {
        dPointVehicle = sqrt(pow(pIvlocpos->xg - pathPoints.at(i).xg, 2) + pow(pIvlocpos->yg - pathPoints.at(i).yg, 2));

        if (dPointVehicle > dToVehicleMin + 5)
        {
            break;
        }
        // anglePointVehicle = IncludeAngle(pathPoints.at(i).angle, carPosition->angle);
        if (dPointVehicle < dToVehicleMin && fabs(anglePointVehicle) < 90 && dPointVehicle < 20)
        {
            dToVehicleMin = dPointVehicle;
            indexNearest = i;
        }
    }
    // std::cout<<" lihui indexNearest: "<<indexNearest<<", dToVehicleMin: "<<dToVehicleMin<<", pathPoints.size(): "<<pathPoints.size()<<std::endl;
    // find all the points in the back within 'disBack' length
    double disBackAll = 0;
    iStartIndex = indexNearest;
    for (long i = indexNearest; i > 0; --i)
    {
        distance = sqrt(pow(pathPoints.at(i).xg - pathPoints.at(i - 1).xg, 2) + pow(pathPoints.at(i).yg - pathPoints.at(i - 1).yg, 2));
        disBackAll = disBackAll + distance;
        if (disBackAll >= disBack)
        {
            iStartIndex = i;
            break;
        }
    }
    if (iStartIndex > indexNearest)
    {
        iStartIndex = indexNearest;
    }
    predict_result->state = iStartIndex;
    // find all the points in the front within 'disFront' length
    // we will later consider the reverse case
    double disFrontAll = 0;
    iEndIndex = pathPoints.size() - 1;
    for (long i = indexNearest; i < pathPoints.size() - 1; i++)
    {
        distance = sqrt(pow(pathPoints.at(i+1).xg - pathPoints.at(i).xg, 2) + pow(pathPoints.at(i + 1).yg - pathPoints.at(i).yg, 2));
        disFrontAll = disFrontAll + distance;
        if (disFrontAll >= disFront)
        {
            iEndIndex = i;
            break;
        }
    }
    if (iEndIndex < indexNearest)
    {
        iEndIndex = indexNearest;
    }

    ivpredict::predictpos temp_point;

    float temp_dis = 0;
    // temp_point.xgmean = pathPoints.at(iStartIndex).xg;
    // temp_point.ygmean = pathPoints.at(iStartIndex).yg;
    // predict_result->positions.push_back(temp_point);
    predict_result->positions.clear();
    // cout << " lihui begin:" << iStartIndex << " end:" << iEndIndex << " dis:" << delta << endl;
    for (long i = iStartIndex; i <= iEndIndex; ++i)
    {
        temp_dis = sqrt(pow(pathPoints.at(i).xg - temp_point.xgmean, 2) + pow(pathPoints.at(i).yg - temp_point.ygmean, 2));
        // cout << " lihui dis target: " << delta << " dis calculate:" << temp_dis << endl;
        if (temp_dis < delta)
        {
            continue;
        }
        temp_point.xgmean = pathPoints.at(i).xg;
        temp_point.ygmean = pathPoints.at(i).yg;
        // matrixPoint.velocity = pathPoints.at(i).velocity;
        predict_result->positions.push_back(temp_point);
    }

    // cout << " lihui point input delete long: " << predict_result->positions.size() << endl;
    // ivpredict::predictobj predict_resultx=SmoothPathx_absline(*predict_result,3);
    // cout << " lihui point SmoothPathx_absline long: " << predict_resultx.positions.size() << endl;
    // *predict_result=CalPathIncludeVelUseCurveBx_absline(predict_resultx,3,delta*10,5,0.1);
}


void refpath::VelocityUpdate(ivpathplanner::ivmsgpath motionPath)
{
    if (iEndIndex - iStartIndex + 1 <= motionPath.points.size())
    {
        for (int i = iStartIndex; i <= iEndIndex; i++)
        {
            pathPoints[i].velocity = motionPath.points[i - iStartIndex].velocity;
        }
    }
    else
    {
        for (int i = iStartIndex; i <= iStartIndex + motionPath.points.size() - 1; i++)
        {
            pathPoints[i].velocity = motionPath.points[i - iStartIndex].velocity;
        }
    }
}

void refpath::MapRecovery()
{
    // pathPoints.clear();
    // pathPoints = pathPointsRaw;

    /*This may needs to be revised later!!*/
    if (iEndIndex > iStartIndex)
    {
        for(int i=0;i<iStartIndex-50;i++)
        {
            pathPoints[i].lon = pathPointsRaw[i].lon;
            pathPoints[i].lat = pathPointsRaw[i].lat;
            pathPoints[i].heading = pathPointsRaw[i].heading;
            pathPoints[i].xg = pathPointsRaw[i].xg;
            pathPoints[i].yg = pathPointsRaw[i].yg;
            pathPoints[i].angle = pathPointsRaw[i].angle;
            pathPoints[i].velocity = INVALID_SPEED;
        }
        for(int i=iEndIndex+80;i<pathPointsRaw.size();i++)
        {
            pathPoints[i].lon = pathPointsRaw[i].lon;
            pathPoints[i].lat = pathPointsRaw[i].lat;
            pathPoints[i].heading = pathPointsRaw[i].heading;
            pathPoints[i].xg = pathPointsRaw[i].xg;
            pathPoints[i].yg = pathPointsRaw[i].yg;
            pathPoints[i].angle = pathPointsRaw[i].angle;
            pathPoints[i].velocity = INVALID_SPEED;
        }
    }
    else
    {
        for(int i=iEndIndex+20;i<iStartIndex-10;i++)
        {
            pathPoints[i].lon = pathPointsRaw[i].lon;
            pathPoints[i].lat = pathPointsRaw[i].lat;
            pathPoints[i].heading = pathPointsRaw[i].heading;
            pathPoints[i].xg = pathPointsRaw[i].xg;
            pathPoints[i].yg = pathPointsRaw[i].yg;
            pathPoints[i].angle = pathPointsRaw[i].angle;
            pathPoints[i].velocity = INVALID_SPEED;
        }
    }

}

void refpath::RawMapImprove(std::vector<sMapInfo> newSegment)
{
	// pathPoints.clear();
	// pathPoints = newSegment;
	// lastIndex = 0;

    /*This may needs to be revised later!!!*/
    sMapInfo gpsStart;
    sMapInfo gpsEnd;
    if (newSegment.size() > 1)
    {
        gpsStart = newSegment[0];
        gpsEnd = newSegment[newSegment.size()-1];
    }
    double minDisStart = 88888.0;
    double minDisEnd = 88888.0;
    int indexStart = 0;
    int indexEnd = 0;
    double dis1 = 0;
    double dis2 = 0;

    for (long i = iStartIndex; i < pathPoints.size(); ++i)
    {
        dis1 = sqrt(pow(pathPoints[i].xg - gpsStart.xg, 2) + pow(pathPoints[i].yg - gpsStart.yg, 2));
        float angle = IncludeAngle(pathPoints[i].angle, gpsStart.angle);
        if (fabs(angle) >= 90.0 )
        {
            continue;
        }
        if(minDisStart > dis1)
        {
            minDisStart = dis1;
            indexStart = i;
        }
        if (dis1 > minDisStart + 5.0)
        {
            break;
        }
    }
    for (long i = iStartIndex ;i < pathPoints.size();i++)
    {
        dis2 = sqrt(pow(pathPoints[i].xg - gpsEnd.xg, 2) + pow(pathPoints[i].yg - gpsEnd.yg, 2));
        float angle = IncludeAngle(pathPoints[i].angle, gpsEnd.angle);
        if (fabs(angle) >= 90.0 )
        {
            continue;
        }
        if(minDisEnd > dis2)
        {
            minDisEnd = dis2;
            indexEnd = i;
        }
        if (dis2 > minDisEnd + 5.0)
        {
            break;
        }
    }

    if (indexEnd - indexStart > 0)
    {
        int newSize = indexEnd - indexStart + 1;
       int isegment = 0;
        double coff = double(newSegment.size()- 1)/(newSize -1);
        for (int i = indexStart; i<= indexEnd; ++i)
        {
            isegment = (int)((i - indexStart)*coff);
            if(isegment > newSegment.size() - 1)
            {
                pathPoints[i].xg = newSegment[newSegment.size() - 1].xg;
                pathPoints[i].yg = newSegment[newSegment.size() - 1].yg;
                pathPoints[i].angle = newSegment[newSegment.size() - 1].angle;
                pathPoints[i].velocity = newSegment[newSegment.size() - 1].velocity;
            }
            else
            {
                pathPoints[i].xg = newSegment[isegment].xg;
                pathPoints[i].yg = newSegment[isegment].yg;
                pathPoints[i].angle = newSegment[isegment].angle;
                pathPoints[i].velocity = newSegment[isegment].velocity;
            }
        }
    }
    else if (indexEnd - indexStart < 0)
    {
        int newSize = pathPointsRaw.size() - indexStart + indexEnd;
        int isegment = 0;
        double coff = double(newSegment.size() - 1)/(newSize - 1);;
        for(int i=indexStart;i<=pathPointsRaw.size() - 1; ++i)
        {
            isegment = (int)((i - indexStart)*coff);
            if(isegment > newSegment.size() - 1)
            {
                pathPoints[i].xg = newSegment[newSegment.size() - 1].xg;
                pathPoints[i].yg = newSegment[newSegment.size() - 1].yg;
                pathPoints[i].angle = newSegment[newSegment.size() - 1].angle;
                pathPoints[i].velocity = newSegment[newSegment.size() - 1].velocity;
            }
            else
            {
                pathPoints[i].xg = newSegment[isegment].xg;
                pathPoints[i].yg = newSegment[isegment].yg;
                pathPoints[i].angle = newSegment[isegment].angle;
                pathPoints[i].velocity = newSegment[isegment].velocity;
            }
        }
    }

}



double refpath::IncludeAngle(double angle1, double angle2)
{
    if (angle1 > angle2 + 180)
    {
        angle2 = angle2 + 360;
    }
    else if (angle2 > angle1 + 180)
    {
        angle1 = angle1 + 360;
    }
    double anglerr = angle1 - angle2;
    return anglerr;
}


void refpath::LoadAllMaps(int numPaths)
{
    //std::cout<<"numPaths: "<<numPaths<<std::endl;
    string routePath = "";
    ros::NodeHandle mh;
    mh.param("routemap", routePath, routePath);
    // routePath.append("/1-seg");
    //std::cout<<"routePath: "<<routePath<<std::endl;
    allRoads.clear();
    for(int i = 1; i <= numPaths; i++)
    {
        string temp_file_name = routePath;
        stringstream ss;
        ss<<i;
        string mapRoute = temp_file_name + "/"+ss.str()+"-seg";
        vector<sMapInfo> eachRoad = LoadMap(mapRoute, i);
        sMaps eachRoadDirection;
        eachRoadDirection.pathSeg = eachRoad;
        eachRoadDirection.forwardBackwardFlag = true;
        allRoads.push_back(eachRoadDirection);
    }


    ivlocmsg::ivsensorgps iabasemapTL;
    iabasemapTL.heading = 90;
    iabasemapTL.lon = 8888;
    iabasemapTL.lat = 8888;
    mh.param("iabasemaptllon",iabasemapTL.lon,iabasemapTL.lon);
    mh.param("iabasemaptllat",iabasemapTL.lat,iabasemapTL.lat);

    gt.originGpsOfGCCS = iabasemapTL;
}

vector<sMapInfo> refpath::LoadMap(string mapRoute, int roadId)
{
    vector<sMapInfo> eachRoad;
    sMapInfo rp;
    std::vector<sGeneralRoadPoint> segdata= mapTool.loadSegData(mapRoute);
    for (auto a : segdata)
    {
        rp.lat = a.ptOfGPS.lat;
        rp.lon = a.ptOfGPS.lon;
        rp.heading = a.ptOfGPS.heading;
        rp.angle = a.ptOfGCCS.angle;
        rp.xg = a.ptOfGCCS.xg;
        rp.yg = a.ptOfGCCS.yg;
        rp.velocity = INVALID_SPEED;
        eachRoad.push_back(rp);
    }
    vector<sMapInfo> eachRoadMorePoints = GetMorePoints(eachRoad);

    return eachRoadMorePoints;
}


void refpath::PathSegsToSmoothPath(vector< int > pathSegIds)
{
    // std::cout<<"pathSegIds.size() 0000000000000000  !!!!!!!!!!!!!!!!!!!!!: "<<pathSegIds.size()<<std::endl;
    vector<sMaps> pathSegs;
    for (int i = 0; i < pathSegIds.size(); i++)
    {
        // std::cout<<"allRoads.size() 0000000000000000  !!!!!!!!!!!!!!!!!!!!!: "<<allRoads.size()<<std::endl;
        // std::cout<<"allRoads[pathSegIds[i]] 0000000000000000  !!!!!!!!!!!!!!!!!!!!!: "<<pathSegIds[i]<<std::endl;
        pathSegs.push_back(allRoads[pathSegIds[i]]);
    }
    // std::cout<<"PathSegsToSmoothPath 11111111111111111111       !!!!!!!!!!!!!!!!!!!!!: "<<std::endl;
    vector<sMapInfo> smoothPath;
    if (pathSegs.size() >= 1)
    {
    // std::cout<<"PathSegsToSmoothPath  22222222222222222  !!!!!!!!!!!!!!!!!!!!!: "<<std::endl;
        bool firstPathDirection = allRoads[pathSegIds[0]].forwardBackwardFlag;
            // std::cout<<"pathSegIds: "<<pathSegIds.size()<<std::endl;

        smoothPath = pathSegs[0].pathSeg;
        // std::cout<<"smoothPath.size(): "<<smoothPath.size()<<std::endl;

        for (int i = 0; i < pathSegs.size() - 1; i++)
        {
                // std::cout<<"i: "<<i<<", pathSegs i: "<<pathSegs[i].pathSeg.size()<<std::endl;
            if (pathSegs[i + 1].forwardBackwardFlag == firstPathDirection || smoothPath.size() < 1)
            {
                if (smoothPath.size() > 0 && pathSegs[i + 1].pathSeg.size() > 0)
                {
                    // std::cout<<"smoothPath.size(): "<<smoothPath.size()<<", pathSegs[i + 1].pathSeg.size(): "<<pathSegs[i + 1].pathSeg.size()<<std::endl;
                    smoothPath = SmoothTwoPaths(smoothPath, pathSegs[i + 1].pathSeg);
                    // std::cout<<"smoothPath.size(): "<<smoothPath.size()<<std::endl;
                }
                else if (smoothPath.size() < 1 && pathSegs[i + 1].pathSeg.size() > 0)
                {
                    smoothPath = pathSegs[i + 1].pathSeg;
                    firstPathDirection = pathSegs[i + 1].forwardBackwardFlag;
                }
            }
        }
    }
    pathPoints.clear();
    pathPointsRaw.clear();
    pathPoints = smoothPath;
    pathPointsRaw = smoothPath;
    lastIndex = 0;
    // std::cout<<"pathPoints.size(): "<<pathPoints.size()<<std::endl;
}

vector<sMapInfo> refpath::SmoothTwoPaths(vector<sMapInfo> pathSeg1, vector<sMapInfo> pathSeg2)
{
    int numPointsPathSeg1 = pathSeg1.size() - 1;
    int nearestPointIdOnPathSeg2 = FindNearestPointId(pathSeg2, pathSeg1[numPointsPathSeg1].xg, pathSeg1[numPointsPathSeg1].yg, pathSeg1[numPointsPathSeg1].angle, 1);
    double disTwoPoints = sqrt(pow(pathSeg2[nearestPointIdOnPathSeg2].xg - pathSeg1[numPointsPathSeg1].xg, 2) + pow(pathSeg2[nearestPointIdOnPathSeg2].yg - pathSeg1[numPointsPathSeg1].yg, 2));
    vector<sMapInfo> pathSegSmooth;
    if (nearestPointIdOnPathSeg2 > 0 && disTwoPoints < 1.5 || (0 == nearestPointIdOnPathSeg2 && disTwoPoints < 0.8))
    {
        double disSegTwoPaths = GetSegDis(pathSeg2, nearestPointIdOnPathSeg2, 0);
        double disSegTwoPathsRest = GetSegDis(pathSeg2, nearestPointIdOnPathSeg2, pathSeg2.size() - 1);
        double distanceStandard = 30 * disTwoPoints / 3.7;
        if (distanceStandard < 2)
        {
            distanceStandard = 2;
        }
        // std::cout<<"disSegTwoPaths: "<<disSegTwoPaths<<", distanceStandard: "<<distanceStandard<<std::endl;
        int disFirstPointSeg2toSmoothPath = FindNearestPointId(pathSeg1, pathSeg2[0].xg, pathSeg2[0].yg, pathSeg2[0].angle, -1);
        double disTwoPoints2 = sqrt(pow(pathSeg2[0].xg - pathSeg1[disFirstPointSeg2toSmoothPath].xg, 2) + pow(pathSeg2[0].yg - pathSeg1[disFirstPointSeg2toSmoothPath].yg, 2));
        if (disSegTwoPathsRest < disSegTwoPaths)
        {
            return pathSeg1;
        }
        if (disSegTwoPaths >= distanceStandard && fabs(disTwoPoints2 - disTwoPoints) < 1.5)
        {
            int lastNearPointIdOnPathSeg1 = 0;
            for (int i = 0; i <= nearestPointIdOnPathSeg2; i++)
            {
                int nearestPointIdOnPathSeg1 = FindNearestPointId(pathSeg1, pathSeg2[i].xg, pathSeg2[i].yg, pathSeg2[i].angle, -1);
                if (nearestPointIdOnPathSeg1 >= lastNearPointIdOnPathSeg1)
                {
                    lastNearPointIdOnPathSeg1 = nearestPointIdOnPathSeg1;
                    double disSegEachPoint = GetSegDis(pathSeg2, 0, i);
                    double disCoe = disSegEachPoint / disSegTwoPaths;
                    if (disCoe > 1)
                    {
                        disCoe = 1;
                    }
                    double funcoe = 0.5 - 0.5 * cos(disCoe * M_PI);
                    if (0 == i)
                    {
                        for (int j = 0; j < nearestPointIdOnPathSeg1; j++)
                        {
                            pathSegSmooth.push_back(pathSeg1[j]);
                        }
                    }
                    sMapInfo smoothPoint = pathSeg1[nearestPointIdOnPathSeg1];
                    smoothPoint.xg = (1- funcoe) * pathSeg1[nearestPointIdOnPathSeg1].xg + funcoe * pathSeg2[i].xg;
                    smoothPoint.yg = (1- funcoe) * pathSeg1[nearestPointIdOnPathSeg1].yg + funcoe * pathSeg2[i].yg;
                    smoothPoint.velocity = -88;
                    pathSegSmooth.push_back(smoothPoint);
                }
            }
            // std::cout<<"nearestPointIdOnPathSeg2: "<<nearestPointIdOnPathSeg2<<", pathSeg2.size(): "<<pathSeg2.size()<<std::endl;
            for (int j = nearestPointIdOnPathSeg2 + 1; j < pathSeg2.size(); j++)
            {
                pathSegSmooth.push_back(pathSeg2[j]);
            }
        }
        else
        {
            // std::cout<<"stop 000: "<<std::endl;
            int startSmoothPointId = GetPointIdBaseDis(pathSeg1, numPointsPathSeg1, distanceStandard, -1);
            // std::cout<<"stop 1: "<<startSmoothPointId<<std::endl;
            // std::cout<<"stop 1.5: "<<pathSeg1.size()<<", pathSegSmooth.size(): "<<pathSegSmooth.size()<<std::endl;

            for (int j = 0; j < startSmoothPointId; j++)
            {
                if (j < 200)
                {
                    //std::cout<<"stop j: "<<j<<", xg: "<<pathSeg1[j].xg<<", yg: "<<pathSeg1[j].yg<<", angle: "<<pathSeg1[j].angle<<std::endl;

                }

                pathSegSmooth.push_back(pathSeg1[j]);
            }
            // std::cout<<"stop 2: "<<std::endl;

            double distanceMove = DisToPath(pathSeg2[nearestPointIdOnPathSeg2], pathSeg1[numPointsPathSeg1].xg, pathSeg1[numPointsPathSeg1].yg);
            // std::cout<<"stop 3: "<<distanceMove<<std::endl;

            sMapInfo pointLeftSide = PointLeftRight(pathSeg1[numPointsPathSeg1].xg, pathSeg1[numPointsPathSeg1].yg, pathSeg1[numPointsPathSeg1].angle, 1);
            sMapInfo pointRightSide = PointLeftRight(pathSeg1[numPointsPathSeg1].xg, pathSeg1[numPointsPathSeg1].yg, pathSeg1[numPointsPathSeg1].angle, -1);
            double disToLeft = sqrt(pow(pointLeftSide.xg - pathSeg2[nearestPointIdOnPathSeg2].xg, 2) + pow(pointLeftSide.yg - pathSeg2[nearestPointIdOnPathSeg2].yg, 2));
            double disToRight = sqrt(pow(pointRightSide.xg - pathSeg2[nearestPointIdOnPathSeg2].xg, 2) + pow(pointRightSide.yg - pathSeg2[nearestPointIdOnPathSeg2].yg, 2));
            if (disToLeft > disToRight)
            {
                distanceMove = -1 * distanceMove;
            }
            // std::cout<<"distanceMove: "<<distanceMove<<", distanceStandard: "<<distanceStandard<<", startSmoothPointId: "<<startSmoothPointId<<", numPointsPathSeg1: "<<numPointsPathSeg1<<std::endl;
            vector<sMapInfo> smoothPath = PathMoveGenerate(pathSeg1, distanceMove, distanceStandard, startSmoothPointId, numPointsPathSeg1);
            for (int j = 0; j < smoothPath.size(); j++)
            {
                pathSegSmooth.push_back(smoothPath[j]);
            }
            int k = nearestPointIdOnPathSeg2;
            // std::cout<<"pathSegSmooth[pathSegSmooth.size() - 1].angle: "<<pathSegSmooth[pathSegSmooth.size() - 1].angle<<std::endl;

            for (k = nearestPointIdOnPathSeg2; k < pathSeg2.size(); k++)
            {
                bool pointmoveflag = true; //PointMoveValid(pathSegSmooth[pathSegSmooth.size() - 1], pathSeg2[k], distanceMove, distanceMove);
                // std::cout<<"false k: "<<k<<std::endl;
                if (true == pointmoveflag)
                {
                    break;
                }
            }
            double disToPathSeg2 = sqrt(pow(pathSeg2[k].xg - pathSegSmooth[pathSegSmooth.size() - 1].xg, 2) + pow(pathSeg2[k].yg - pathSegSmooth[pathSegSmooth.size() - 1].yg, 2));

            int numIncrease = disToPathSeg2 * 10;
            for (int t = 1; t < numIncrease + 1; t++)
            {
                sMapInfo smoothPointConnect = pathSegSmooth[pathSegSmooth.size() - 1];
                smoothPointConnect.xg = t * 1.0 / (numIncrease + 1) * pathSeg2[k].xg + (numIncrease + 1 - t) * 1.0 / (numIncrease + 1) * pathSegSmooth[pathSegSmooth.size() - 1].xg;
                smoothPointConnect.yg = t * 1.0 / (numIncrease + 1) * pathSeg2[k].yg + (numIncrease + 1 - t) * 1.0 / (numIncrease + 1) * pathSegSmooth[pathSegSmooth.size() - 1].yg;
                smoothPointConnect.velocity = -88;
                pathSegSmooth.push_back(smoothPointConnect);
            }
            // std::cout<<"k: "<<k<<", pathSeg2.size(): "<<pathSeg2.size()<<std::endl;
            for (int m = k; m < pathSeg2.size(); m++)
            {
                pathSegSmooth.push_back(pathSeg2[m]);
            }
        }
    }
    else if (nearestPointIdOnPathSeg2 <= 2 && disTwoPoints < 10)
    {
        for (int j = 0; j < pathSeg1.size(); j++)
        {
            pathSegSmooth.push_back(pathSeg1[j]);
        }
        int numIncrease = disTwoPoints * 10;
        for (int t = 1; t < numIncrease + 1; t++)
        {
            sMapInfo smoothPointConnect = pathSeg1[pathSeg1.size() - 1];
            smoothPointConnect.xg = t * 1.0 / (numIncrease + 1) * pathSeg2[0].xg + (numIncrease + 1 - t) * 1.0 / (numIncrease + 1) * pathSeg1[pathSeg1.size() - 1].xg;
            smoothPointConnect.yg = t * 1.0 / (numIncrease + 1) * pathSeg2[0].yg + (numIncrease + 1 - t) * 1.0 / (numIncrease + 1) * pathSeg1[pathSeg1.size() - 1].yg;
            smoothPointConnect.velocity = -88;
            pathSegSmooth.push_back(smoothPointConnect);
            if (t > 100)
            {
                break;
            }
        }
        for (int m = 0; m < pathSeg2.size(); m++)
        {
            pathSegSmooth.push_back(pathSeg2[m]);
        }
    }
    else
    {
        pathSegSmooth = pathSeg1;
    }
    return pathSegSmooth;
}

double refpath::GetSegDis(vector<sMapInfo> pathSeg, int startId, int endId)
{
    double disSeg = 0;
    if (startId > endId)
    {
        for (int i = startId; i > endId; i--)
        {
            disSeg = disSeg + sqrt(pow(pathSeg[i].xg - pathSeg[i - 1].xg, 2) + pow(pathSeg[i].yg - pathSeg[i - 1].yg, 2));
        }
    }
    else
    {
        for (int i = startId; i < endId; i++)
        {
            disSeg = disSeg + sqrt(pow(pathSeg[i].xg - pathSeg[i + 1].xg, 2) + pow(pathSeg[i].yg - pathSeg[i + 1].yg, 2));
        }
    }
    return disSeg;
}

int refpath::GetPointIdBaseDis(vector<sMapInfo> pathSeg, int startId, double distance, int searchSequency)
{
    double disSeg = 0;
    int pointIdFinal = startId;
    if (searchSequency < 0)
    {
        for (int i = startId; i > 0; i--)
        {
            disSeg = disSeg + sqrt(pow(pathSeg[i].xg - pathSeg[i - 1].xg, 2) + pow(pathSeg[i].yg - pathSeg[i - 1].yg, 2));
            if (disSeg >= distance)
            {
                pointIdFinal = i;
                break;
            }
        }
    }
    else
    {
        for (int i = startId; i < pathSeg.size(); i++)
        {
            disSeg = disSeg + sqrt(pow(pathSeg[i].xg - pathSeg[i + 1].xg, 2) + pow(pathSeg[i].yg - pathSeg[i + 1].yg, 2));
            if (disSeg >= distance)
            {
                pointIdFinal = i;
                break;
            }
        }
    }
    return pointIdFinal;
}

double refpath::DisToPath(sMapInfo pointOnPath, double xPointDis, double yPointDis)
{
    double thetapath = PathDirection(pointOnPath.angle);
    double disobjectroad = 0;
    if (fabs(thetapath - 0.5 * M_PI) < 0.0000001 || fabs(thetapath + 0.5 * M_PI) < 0.00000001)
    {
        disobjectroad = fabs(pointOnPath.xg - xPointDis);
    }
    else
    {
        double klinederr = tan(thetapath);
        double blinederr = pointOnPath.yg - klinederr * pointOnPath.xg;
        disobjectroad = fabs((klinederr * xPointDis - yPointDis + blinederr) / sqrt(klinederr * klinederr + 1));
    }
    return disobjectroad;
}

int refpath::FindNearestPointId(vector<sMapInfo> pathSegNearest, double xNear, double yNear, double angleNear, int searchSequency)
{
    double disObjectToPathMin = 10000;
    int disminindex = -100;
    if (searchSequency >= 0)
    {
        for (int i = 0; i < pathSegNearest.size(); i++)
        {
            double disobjecttopath = sqrt(pow(pathSegNearest[i].xg - xNear, 2) + pow(pathSegNearest[i].yg - yNear, 2));
            if (disobjecttopath > disObjectToPathMin + 5)
            {
                break;
            }
            if (disobjecttopath < disObjectToPathMin)
            {
                disObjectToPathMin = disobjecttopath;
                disminindex = i;
            }
        }
    }
    else
    {

        for (int i = pathSegNearest.size() - 1; i >= 0; i--)
        {
            double disobjecttopath = sqrt(pow(pathSegNearest[i].xg - xNear, 2) + pow(pathSegNearest[i].yg - yNear, 2));
            if (disobjecttopath > disObjectToPathMin + 5)
            {
                break;
            }
            if (disobjecttopath < disObjectToPathMin)
            {
                disObjectToPathMin = disobjecttopath;
                disminindex = i;
            }
        }
    }

    return disminindex;
}

vector<sMapInfo> refpath::PathMoveGenerate(vector<sMapInfo> pathSmoothSeg, double distanceMove, double distanceSmoothSeg, int startSmoothPointId, int endSmoothPointId)
{

    vector<sMapInfo> pathMoveOut;
    double distanceSeg = 0;
    double disMoveFinal = distanceMove;
    sMapInfo pointmove;
    double disMoveLastPoint = 0;
    int numPointsCurve = 0;
    int numCurveStartId = -1;
    int ipushback = 0;
    int mumPathMove = pathSmoothSeg.size();
    for (int i = startSmoothPointId; i < endSmoothPointId; i++)
    {
        distanceSeg = distanceSeg + sqrt(pow(pathSmoothSeg[i].xg - pathSmoothSeg[i - 1].xg, 2) + pow(pathSmoothSeg[i].yg - pathSmoothSeg[i - 1].yg, 2));
        double discoe = distanceSeg / distanceSmoothSeg;
        if (discoe > 1)
        {
            discoe = 1;
        }
        double funcoe = 0.5 - 0.5 * cos(discoe * M_PI);
        disMoveFinal = funcoe * distanceMove;
        //std::cout<<"i: "<<i<<", nearestIdFirstPoint: "<<nearestIdFirstPoint<<", xi: "<<pathSmoothSeg[i].xg<<", xi-1: "<<pathSmoothSeg[i - 1].xg<<", yi: "<<pathSmoothSeg[i].yg<<", yi-1: "<<pathSmoothSeg[i - 1].yg<<std::endl;

        //std::cout<<"distanceSeg: "<<distanceSeg<<", distanceSmoothSeg: "<<distanceSmoothSeg<<std::endl;
        //std::cout<<"i: "<<i<<", discoe: "<<discoe<<", disMoveFinal: "<<disMoveFinal<<std::endl;


        if (fabs(disMoveFinal) < 0.01)
        {
            pointmove = pathSmoothSeg[i];
        }
        else
        {
            if (i < mumPathMove - 6 && i > 5)
            {
                pathSmoothSeg[i].angle = UpdatePointAngle(pathSmoothSeg, i);
            }
            pointmove = PointLeftRight(pathSmoothSeg[i].xg, pathSmoothSeg[i].yg, pathSmoothSeg[i].angle, disMoveFinal);
        }

        if (i < startSmoothPointId + 50)
        {
            //std::cout<<"pointmove.xg: "<<pointmove.xg<<", pointmove.yg: "<<pointmove.yg<<std::endl;
        }

        if (i > startSmoothPointId)
        {
            bool pointmoveflag = PointMoveValid(pathSmoothSeg[ipushback], pointmove, disMoveFinal, disMoveLastPoint);
            if (true == pointmoveflag)
            {
                ipushback = i;
                pathMoveOut.push_back(pointmove);
                numPointsCurve++;
                if (1 == numPointsCurve)
                {
                    numCurveStartId = pathMoveOut.size() - 1;
                }
            }
        }
        else
        {
            ipushback = i;
            pathMoveOut.push_back(pointmove);
        }
        disMoveLastPoint = disMoveFinal;

    }

    vector<sMapInfo> pathMoveAngleUpdateOut = AngleUpdate(pathMoveOut, numCurveStartId, numPointsCurve);

            //std::cout<<"pathMoveOut size: "<<pathMoveOut.size()<<std::endl;

    return pathMoveAngleUpdateOut;
}

sMapInfo refpath::PointLeftRight(double xlr, double ylr, double anglelr, double dislr)
{
    double pathpointdirection = PathDirection(anglelr);
    sMapInfo pointlr;
    pointlr.xg = xlr + fabs(dislr) * cos(pathpointdirection + 0.5 * M_PI * Signn(dislr));
    pointlr.yg = ylr + fabs(dislr) * sin(pathpointdirection + 0.5 * M_PI * Signn(dislr));
    pointlr.angle = anglelr;
    return pointlr;
}

vector<sMapInfo> refpath::AngleUpdate(vector<sMapInfo> pathMoveOut, int numCurveStartId, int numPointsCurve)
{
    vector<sMapInfo> pathMoveAngleUpdate = pathMoveOut;
    if (numCurveStartId >= 0 && numPointsCurve > 10)
    {
        for (int i = numCurveStartId + 2; i < numCurveStartId + numPointsCurve - 2; i++)
        {
            double angleUpdate = LineDirection(pathMoveAngleUpdate[i + 2].xg, pathMoveAngleUpdate[i + 2].yg, pathMoveAngleUpdate[i - 2].xg, pathMoveAngleUpdate[i - 2].yg);
            pathMoveAngleUpdate[i].angle = angleUpdate * 180 / M_PI;
        }
    }
    return pathMoveAngleUpdate;
}




bool refpath::PointMoveValid(sMapInfo pointOnPath1, sMapInfo pointMove2, double disMoveFinal, double disMoveLastPoint)
{
    bool currentmovevalid = false;
    if ((disMoveFinal > 0 && disMoveLastPoint < 0) || (disMoveFinal < 0 && disMoveLastPoint > 0) || disMoveFinal == 0 || disMoveLastPoint == 0)
    {
        currentmovevalid = true;
    }
    else if (disMoveFinal > 0)
    {
        sMapInfo pointleft = PointLeftRight(pointOnPath1.xg, pointOnPath1.yg, pointOnPath1.angle, 1);
        sMapInfo pointright = PointLeftRight(pointOnPath1.xg, pointOnPath1.yg, pointOnPath1.angle, -1); // + 90
        double distoleft = sqrt(pow(pointleft.xg - pointMove2.xg, 2) + pow(pointleft.yg - pointMove2.yg, 2));
        double distoright = sqrt(pow(pointright.xg - pointMove2.xg, 2) + pow(pointright.yg - pointMove2.yg, 2));
        if (distoleft > distoright)
        {
            currentmovevalid = true;
        }
                   // std::cout<<"left distoleft: "<<distoleft<<", distoright: "<<distoright<<std::endl;

    }
    else
    {
        sMapInfo pointleft = PointLeftRight(pointOnPath1.xg, pointOnPath1.yg, pointOnPath1.angle, 1);  // - 90
        sMapInfo pointright = PointLeftRight(pointOnPath1.xg, pointOnPath1.yg, pointOnPath1.angle, -1);
        double distoleft = sqrt(pow(pointleft.xg - pointMove2.xg, 2) + pow(pointleft.yg - pointMove2.yg, 2));
        double distoright = sqrt(pow(pointright.xg - pointMove2.xg, 2) + pow(pointright.yg - pointMove2.yg, 2));
        if (distoleft < distoright)
        {
            currentmovevalid = true;
        }
        //std::cout<<"right distoleft: "<<distoleft<<", distoright: "<<distoright<<std::endl;

    }
    return currentmovevalid;
}

double refpath::PathDirection(double roadPointDirection)
{
    double vehicleDirection = 0;
    double includedAngleVehicleRoad = roadPointDirection * M_PI / 180.0;
    return includedAngleVehicleRoad;
}

int refpath::Signn(double bValue)
{
    int bvaluesign = 1;
    if (bValue >= 0)
    {
        bvaluesign = 1;
    }
    else
    {
        bvaluesign = -1;
    }
    return bvaluesign;
}

double refpath::LineDirection(double xSecond, double ySecond, double xFirst, double yFirst)
{
    double alpha = 0.0;
    if ((xSecond>xFirst)&&(ySecond>=yFirst))
    {
        alpha=atan(fabs(ySecond-yFirst)/fabs(xSecond-xFirst));
    }
    else if ((xSecond<xFirst)&&(ySecond>=yFirst))
    {
        alpha=M_PI-atan(fabs(ySecond-yFirst)/fabs(xSecond-xFirst));
    }
    else if ((xSecond<xFirst)&&(ySecond<yFirst))
    {
        alpha=atan(fabs(ySecond-yFirst)/fabs(xSecond-xFirst)) - M_PI;
    }
    else if ((xSecond>xFirst)&&(ySecond<yFirst))
    {
        alpha=-atan(fabs(ySecond-yFirst)/fabs(xSecond-xFirst));
    }
    else if ((xSecond==xFirst)&&(ySecond>yFirst))
    {
        alpha = 0.5 * M_PI;
    }
    else if ((xSecond==xFirst)&&(ySecond<yFirst))
    {
        alpha = -0.5 * M_PI;
    }
    else if ((xSecond==xFirst)&&(ySecond==yFirst))
    {
        alpha=0;
    }
    return alpha;
}


double refpath::UpdatePointAngle(vector<sMapInfo> pathAngleUpdate, int ist)
{
    double pointAngle = LineDirection(pathAngleUpdate[ist + 5].xg, pathAngleUpdate[ist + 5].yg, pathAngleUpdate[ist - 5].xg, pathAngleUpdate[ist - 5].yg);
    pointAngle = pointAngle * 180 / M_PI;
    if (pointAngle - pathAngleUpdate[ist].angle > 180)
    {
        pointAngle = pointAngle - 360;
    }
    else if (pointAngle - pathAngleUpdate[ist].angle < -180)
    {
        pointAngle = pointAngle + 360;
    }
    if (fabs(pointAngle - pathAngleUpdate[ist].angle) > 10)
    {
        pathAngleUpdate[ist].angle = pointAngle + Signn(pathAngleUpdate[ist].angle - pointAngle) * 10;
    }
    return pathAngleUpdate[ist].angle;
}


// void refpath::LocalPathMap(ivmap::ivmapmsglocpos carPosition-> ivpathplanner::path newPath)
// {
//     sMapInfo NewRefPoint;
//     vector<sMapInfo> NewRefPath;

//     for (int i = 0; i < newPath.points.size();i++)
//     {
//         // local frame to global frame
//         sPointOfVCS localPosition;
//         sPointOfGCCS globalPosition;
//         localPosition.x = newPath.points.at(i).x;
//         localPosition.y = newPath.points.at(i).y;

//         sPointOfGCCS carPos;
//         carPos.xg = carPosition->xg;
//         carPos.yg = carPosition->yg;
//         carPos.angle = carPosition->angle;
//         globalPosition = gt.VCS2GCCS(carPos, localPosition);
//         globalPosition.angle = carPosition->angle - newPath.points.at(i).angle;
//         if (globalPosition.angle < 0)
//         {
//             globalPosition.angle += 360.0;
//         }
//         else if (globalPosition.angle >= 360)
//         {
//             globalPosition.angle -= 360.0;
//         }

//         NewRefPoint.angle = globalPosition.angle;
//         NewRefPoint.xg = globalPosition.xg;
//         NewRefPoint.yg = globalPosition.yg;
//         NewRefPoint.velocity = INVALID_SPEED;
//         NewRefPath.push_back(NewRefPoint);
//     }
//     if (NewRefPath.size() > 1)
//     {
//         pathPoints.clear();
//         pathPoints = NewRefPath;
//         lastIndex = 0;
//     }
// }

void refpath::mapOrigin()
{
	pathPoints = pathPointsRaw;
    lastIndex = 0;
}

void refpath::UpdatePathPoints(vector<sMapInfo> newPathPoints)
{
    pathPoints = newPathPoints;
    pathPointsRaw = newPathPoints;
    lastIndex = 0;
}


vector<sMapInfo> refpath::GetMorePoints(vector<sMapInfo> eachRoad)
{
    vector<sMapInfo> eachRoadMorePoints;
    int numRealPath = eachRoad.size();
    sMapInfo eachPoint;
    eachPoint.velocity = INVALID_SPEED;
    for (int irealpath = 0; irealpath < numRealPath - 1; irealpath++)
    {
        double dtwopoints = sqrt(pow(eachRoad[irealpath].xg - eachRoad[irealpath + 1].xg, 2) + pow((eachRoad[irealpath].yg - eachRoad[irealpath + 1].yg), 2));  
        int itwopoints = (int) (dtwopoints * 10);
        if (itwopoints >= 1)
        {
            for (int imore = 0; imore < itwopoints + 1; imore++)
            {
                double x = (1 - imore * 1.0 / (itwopoints + 1)) * eachRoad[irealpath].xg + imore * 1.0 / (itwopoints + 1) * eachRoad[irealpath + 1].xg;
                double y = (1 - imore * 1.0 / (itwopoints + 1)) * eachRoad[irealpath].yg + imore * 1.0 / (itwopoints + 1) * eachRoad[irealpath + 1].yg;
                eachPoint.xg = x;
                eachPoint.yg = y;
                if (fabs(eachRoad[irealpath].angle - eachRoad[irealpath + 1].angle) > 180)
                {
                    eachPoint.angle = eachRoad[irealpath + 1].angle;
                }
                else
                {
                    double angle = (1 - imore * 1.0 / (itwopoints + 1)) * eachRoad[irealpath].angle + imore * 1.0 / (itwopoints + 1) * eachRoad[irealpath + 1].angle;
                    eachPoint.angle = angle;
                }
                eachRoadMorePoints.push_back(eachPoint);
            }
        }
        else
        {
                eachRoadMorePoints.push_back(eachRoad[irealpath]);
        }
    }
    return eachRoadMorePoints;
}

ivpredict::predictobj refpath::SmoothPathx_absline(ivpredict::predictobj initPrePath, int dataNumForSmooth)
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
ivpredict::predictobj refpath::CalPathIncludeVelUseCurveBx_absline(ivpredict::predictobj initPrePath,int fitOrder,double objVel,double timeLength,double timeStep)
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
      double detaRate=0.01;
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
          if(minDetaRate < 0.001)
          {
            minDetaRate = 0.001;
          }
          
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
  switch(true)
  {
    case true:
      totalNum=round(timeLength/timeStep);
      break;
    case false:
      // totalNum=round(gPathDisSet/initPrePath.v_abs/timeStep);
      break;
    default:
      break;
  } 
  // cout << "lihui velPrePath size : " << velPrePath.positions.size() << endl;

  // if (velPrePath.positions.size()>totalNum)
  // {
  //   velPrePath.positions.erase(velPrePath.positions.begin()+totalNum,velPrePath.positions.end());
  // }

 //  if (velPrePath.positions.size()<totalNum)
 //  {
 //    for (int i = velPrePath.positions.size(); i < totalNum; i++)
 //    {
 //      ivpredict::predictpos tmpPoint;
 //      tmpPoint.xgmean=velPrePath.positions[velPrePath.positions.size()-1].xgmean;
 //      tmpPoint.ygmean=velPrePath.positions[velPrePath.positions.size()-1].ygmean;
 //      velPrePath.positions.push_back(tmpPoint);
 //    }
 // }



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
}