#include "motionplanner.h"

motionplanner::motionplanner()
{
    InitMotionplannerParam();
}

motionplanner::~motionplanner()
{
}

void motionplanner::InitMotionplannerParam()  // initilize variable
{
    ros::NodeHandle mh;
    mh.param("radiusTurnMin", radiusMin, radiusMin);
    xg_PointS1 = 494;
    yg_PointS1 = 382;
    xg_PointE1 = 476;
    yg_PointE1 = 406;
    xg_PointS2 = 72; 
    yg_PointS2 = 435;
    xg_PointE2 = 48; 
    yg_PointE2 = 417;
    xg_PointS3 = 118;
    yg_PointS3 = 75; 
    xg_PointE3 = 28; 
    yg_PointE3 = 44; 
    xg_PointS4 = 305;
    yg_PointS4 = 10; 
    xg_PointE4 = 327;
    yg_PointE4 = 22; 

    speed_upper = 30 / 3.6;
    speed_lower = 15 / 3.6;
    speedUpdate = false;
    accelPlusMax = 0.5;
    accelMinusMax = -0.8; //-1.0;
    lengthPreview = 50;
    pathPreview = 100;
    pathPreviewMax = 100; //For pathplanner length (realtime)
    pathEndStopNum = 5;
    startPointIndex = 1;
    pathID = 0;
    emergStatus = false;

    passWidthLower = 1.2;
    passWidthUpper = 2.0;
    carLength = 4.5;
    timePred = 3.0;
    blindsScope = 0.1;
    pathLength = 100;
    pathLengthLast = 100;
    pathLengthChange = false;
    followingObjChange = false;

    distPreObj = 5;//6;
    distPreObjMin = 2.5;
    distSafe = distPreObj;

    VelocityReplanObj = false;
    TrafficFreePlanningMode = 0;  //0:yanbo 1:fusheng;
    TrafficBasedPlanningMode = 0; //0:yanbo 1:fusheng;
    fuzzyAccelLast = ACCELERATION_INIT;
    fuzzyInitSpeed = 0.0;
    egoSpeedLast = 0;

    sbjSpeed_last_for_filter = 0;
    speedlost_cnt = 0;
    sbjSpeed = 0;
    vehSpd = 0; //0131
    vehSpd_last = 0;
    accelCar = 0;
    speedObj = 0;
    speedObj_last = 0;
    objExistFlag = 0;   //@pqg 0322

    crossSpeedBase = 0;    //@PQG 0223
    crossSpeedBase_last = 0;

    pathDirection = 0; //@pqg TODO 0327
    shiftPosReq = 3;
    
    /***Parameter of fusheng motionplan method***/
    g_start_v = 0;
    // mean_filter_init(&mf1, 3);
    // mean_filter_init(&filter_uturn_size, 13);
    planner.behavior_init();
    // visualization = new Visualization(mh);
    // visualization_long = new Visualization(mh);
    //0.0008
    
    /***Parameter of Kalman filter***/
    pkalman = new OneDimKalman(0.00003);//3
    pkalman01 = new OneDimKalman(0.0002);
    
    pkalman02 = new OneDimKalman(0.00008);
    pkalman03 = new OneDimKalman(0.00005);
    pkalman04 = new OneDimKalman(0.0002);

    /***Low-pass filter***/
    // lpf_1p_set_cutoff_freq(&param, 0.05, 0.9);

}


void motionplanner::MotionPlanning()
{
    //speedUpdate = true;
    TrafficFreePlanningMode = 1;
    TrafficBasedPlanningMode = 0;
    
    RawPathProcess(optionPaths);

    if ( pathDirection == 0 )
    {
        shiftPosReq = 3 ;
    }
    if ( pathDirection == 1 )
    {
        shiftPosReq = 1;
    }

    #if 1
    pathDirection = 0; 
    shiftPosReq = 3; 
    #endif
    

    if (( 0 == TrafficFreePlanningMode && pathDirection == DRIVE ) || (pathDirection == PARKING) )
    {
        TrafficFreePlanning(motOptionPathPoints);
    }
    else if (1 == TrafficFreePlanningMode && pathDirection == DRIVE)
    {
        FittingCurve(motOptionPathPoints);
        roadType = pathDecision.PathCurve(motOptionPathPoints, g_start_v, curveValue);
        auto behaviorState = planner.UpdateBehavior(motOptionPathPoints, objPred, curveValue[0], ivvap);
        objExistFlag = std::get<0>(behaviorState);
        behaviorState = std::make_tuple(0, 0, 0);  
        targetPosVel = planner.UpdateVelocity(g_start_v, behaviorState, curveValue, roadType);  
        g_start_v = velocityGeneration.SetVelocity(motOptionPathPoints, ivvap, targetPosVel, behaviorState, objExistFlag, accelCar); //TODO @pqg 0326 add accelCar msg
    }

    if (0 == TrafficBasedPlanningMode)
    {
        TrafficBasedPlanning(motOptionPathPoints, objPred, sbjSpeed); 
    }
    else if (1 == TrafficBasedPlanningMode)
    {

    }
    
    pathID = SelectPath(motOptionPathPoints);
    GeneratePath(motOptionPathPoints);
    
    int count_tmp = 3;
    if ( motOptionPathPoints.size() > 0 )
    {
        if ( motOptionPathPoints[pathID].size() > (startPointIndex + count_tmp) )
        {
            motionVars.speed = motOptionPathPoints[pathID][startPointIndex + count_tmp].pathpoints_in.velocity;
        }
        else if (motOptionPathPoints[pathID].size() > 0 )
        {
            double indexShow = motOptionPathPoints[pathID].size() - 1;
            motionVars.speed = motOptionPathPoints[pathID][indexShow].pathpoints_in.velocity;
        }
        else
        {
            motionVars.speed = VELOCITY_INVALID;
        }
    }
    else
    {
        motionVars.speed = VELOCITY_INVALID;
    }
    //motionVars.testVar = fuzzycontrol.accel_mode;
    //motionPlannerBackup = motOptionPathPoints[pathID];  //TODO @pqg 0320 
}

/******************************************************
 * Function: RawPathProcess()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Add the length and curvature info for raw optionpath;
 * Input: optionPathsRaw
 * Output: None
 * Return: None
 * Others: None
 *******************************************************/
void motionplanner::RawPathProcess(const ivpathplanner::ivmsgpathplanner &optionPathsRaw)
{
    motOptionPathPoints.clear();
    pathId.clear();

    if (optionPathsRaw.paths.size() < 1)
    {
        motPathPoints.clear();
        int virPointsNum = 20;

        pathId.push_back(PATH_MAP);
        for (int i = 0; i < virPointsNum; i++)
        {
            motPoint.curvature = 0;
            motPoint.pathpoints_in.x = 0.1 * i;
            motPoint.pathpoints_in.y = 0.0;
            motPoint.pathpoints_in.xc = 0.088;
            motPoint.pathpoints_in.yc = 0;
            motPoint.pathpoints_in.xg = 0;
            motPoint.pathpoints_in.yg = 0;
            motPoint.pathpoints_in.angle = 0.0;
            motPoint.pathpoints_in.velocity = SPEED_STOP;
            motPoint.pathpoints_in.a = SPEED_STOP;
            motPoint.length = 0.1 * i;
            motPathPoints.push_back(motPoint);
        }
        motOptionPathPoints.push_back(motPathPoints);
    }
    else
    {
        for (int i = 0; i < optionPathsRaw.paths.size(); i++)
        {
            motPathPoints.clear();
            pathId.push_back(optionPathsRaw.paths[i].pathid);

            if (optionPathsRaw.paths[i].points.size() <= startPointIndex)
            {
                return;
            }
            else
            {
                double dis2Pts = 0.0;
                double segLength = 0.0;
                double nearDist = INVALIDDOUBLE;

                for (int j = 0; j < optionPathsRaw.paths[i].points.size(); j++)
                {
                    double distPoint = matool.GetDist(optionPathsRaw.paths[i].points[j].x,
                                                      optionPathsRaw.paths[i].points[j].y, 0, 0);
                    if (nearDist > distPoint)
                    {
                        nearDist = distPoint;
                        startPointIndex = j + 1;// TODO @yanbo 20180104;
                    }
                }
                for (int j = 0; j < optionPathsRaw.paths[i].points.size(); j++)
                {
                    motPoint.curvature = CARVE_INIT;
                    motPoint.pathpoints_in.x = optionPathsRaw.paths[i].points[j].x;
                    motPoint.pathpoints_in.y = optionPathsRaw.paths[i].points[j].y;
                    motPoint.pathpoints_in.xc = optionPathsRaw.paths[i].points[j].xc;
                    motPoint.pathpoints_in.yc = optionPathsRaw.paths[i].points[j].yc;
                    motPoint.pathpoints_in.xg = optionPathsRaw.paths[i].points[j].xg;
                    motPoint.pathpoints_in.yg = optionPathsRaw.paths[i].points[j].yg;
                    motPoint.pathpoints_in.angle = optionPathsRaw.paths[i].points[j].angle;
                    motPoint.pathpoints_in.velocity = optionPathsRaw.paths[i].points[j].velocity;
                    motPoint.pathpoints_in.a = optionPathsRaw.paths[i].points[j].a;
                    if (j <= startPointIndex)
                    {
                        motPoint.length = 0.01;
                    }
                    else
                    {
                        dis2Pts = matool.GetDist(optionPathsRaw.paths[i].points[j].x,
                                                 optionPathsRaw.paths[i].points[j].y,
                                                 optionPathsRaw.paths[i].points[j - 1].x,
                                                 optionPathsRaw.paths[i].points[j - 1].y);
                        motPoint.length = segLength + dis2Pts;
                    }
                    segLength = motPoint.length;
                    motPathPoints.push_back(motPoint);
                }
            }
            motOptionPathPoints.push_back(motPathPoints);
        }
    }

}

/******************************************************
 * Function: TrafficFreePlanning()
 * Author: Bo Yan
 * Date: 2017-10-16
 * Description: Genetare velocity for Traffic-free condition;
 * Input: optionPathPoints
 * Output: None
 * Return: None
 * Others: None
 *******************************************************/
void motionplanner::TrafficFreePlanning(std::vector<std::vector<sPoint> > &optionPathPoints)
{
    for (int i = 0; i < optionPathPoints.size(); i++)
    {
        // 1. VelocityUpdate flag
        double a = -0.8;
        double b = 1.0;
        double c = 10;

        pathPreview = MIN(pathPreviewMax, -0.5 * pow(sbjSpeed, 2) / a + b * sbjSpeed + c);
        pathLength = optionPathPoints[i][optionPathPoints[i].size() - 1].length;

        if (fabs(pathLength - pathLengthLast) > 10 || pathLength / pathLengthLast < 0.8 || pathLength / pathLengthLast > 1.2)
        {
            pathLengthChange = true;
        }

        if (fabs(optionPathPoints[i][startPointIndex].pathpoints_in.velocity - ivvap.v) > 3 / 3.6) // 5 || ivvap.v <= 0.3
        {
            speedUpdate = true;
        }

        
        if (PATH_MAP == pathId[i])  //TODO @  yanbo define PATH_MAP 0 in motionplanner.h
        {
            if (PATH_MARK && 0)  // define PATH_MARK 1 in motionplanner.h
            {
                GenVelocityBasedRoadMark(optionPathPoints[i]);
            }
            else
            {
                FittingCurve(optionPathPoints[i]);
                //SmoothCurve(optionPathPoints[i]);   //TODO @pqg 0322
                AnalysisCurve(optionPathPoints[i]);

                GenVelocityBasedRoadOld(optionPathPoints[i]);
                //GenVelocityBasedRoad(optionPathPoints[i]);
            }

        }
        else
        {
            FittingCurve(optionPathPoints[i]);
            //SmoothCurve(optionPathPoints[i]);      //TODO @pqg 0322
            AnalysisCurve(optionPathPoints[i]);
            GenVelocityBasedRoadOld(optionPathPoints[i]);
            //GenVelocityBasedRoad(optionPathPoints[i]);  //empty function
        }


        pathLengthLast = pathLength;
    }

}

void motionplanner::TrafficBasedPlanning(std::vector<std::vector<sPoint> > &optionPathPoints, ivpredict::ivmsgpredict &objPredict, double egoVelocity)
{
    double a = -1.0;
    double b = 1.5;
    double c = 10;
    double lengthPreview  = MIN(pathPreviewMax, -0.5 * pow(egoVelocity, 2) / a + b * egoVelocity + c);
    double passWidthLower = 1.2;
    double passWidthUpper = 2.0; //Params
    double carLength = 4.5;
    
    if(optionPathPoints.size() > 0)
    {
        for(int i = 0; i < optionPathPoints.size(); i++)
        {
            FocusingObjs(optionPathPoints[i], objPredict, egoVelocity, pathId[i]);
            GenVelocityTrafficBased(optionPathPoints[i], trackObjs, egoVelocity, pathId[i]);
            //GenVelocityVirtualObj(optionPathPoints[i], objVirtual, egoVelocity);
        }
    }
}

void motionplanner::GenVelocityTrafficBased(std::vector<sPoint> &PathPoints, std::vector<sObjFocus> &keyObjs, double egoSpeed, int pathNum)
{
    // 1. status switching
    double distEmergency = 8;
    double a = -3;
    double b = 0.5;
    double c = 5;
    double collisionInterval = 0;
    double distLon = 0;
    double distLat = 0;
    double dist2AccuracyControl = MAX(-0.5 * pow(egoSpeed, 2) / a + b * egoSpeed + c , distEmergency);
    sObjPos collisionPos;

    sobjkeyInfo followingObj;
    followingObj.targetAcceleration = ACCELERATION_INIT;
    followingObj.targetIndex = INVALIDINT;
    followingObj.targetPreIndex = INVALIDINT;
    followingObj.id = 99;
    followingObj.objType = 99;
    
    if (keyObjs.size() > 0)
    {
        cout<<"Test0311debug03:There are some OBJ"<<endl;

        for (int i = 0; i < keyObjs.size(); i++)
        {
            double objPredTimeInterval = keyObjs[i].objfocus.time_interval;
            double objPredStep = keyObjs[i].objfocus.steps;
            objPredTimeInterval = MIN(1, objPredTimeInterval);
            objPredTimeInterval = MAX(0.01, objPredTimeInterval);
            objPredStep = MIN(100, objPredStep);
            objPredStep = MAX(0, objPredStep);
            int objStepsPred = int(timePred / objPredTimeInterval);
            objStepsPred = MIN(objStepsPred, objPredStep);
            
            if (keyObjs[i].roi_id == 1)
            {
                for (int j = 0; j < objStepsPred; j++)
                {
                    collisionPos = CalcuObjPos(keyObjs[i].objfocus.positions[j].xmean, keyObjs[i].objfocus.positions[j].ymean, PathPoints);
                    if (collisionPos.dist <= passWidthLower && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                    {
                        double timeCollisionEgo = GetTHW(PathPoints, collisionPos.posid);
                        double timeCollisionObj = j * keyObjs[i].objfocus.time_interval;
                        double collisionInterval = timeCollisionEgo - timeCollisionObj;
                        double distLon = PathPoints[collisionPos.posid].length;
                        distLat = collisionPos.dist;
                        break;
                    }
                }
            }
            else
            {
                collisionPos = CalcuObjPos(keyObjs[i].objfocus.positions[0].xmean, keyObjs[i].objfocus.positions[0].ymean, PathPoints);
                double timeCollisionEgo = GetTHW(PathPoints, collisionPos.posid);
                double timeCollisionObj = 0;
                collisionInterval = timeCollisionEgo - timeCollisionObj;
                distLon = PathPoints[collisionPos.posid].length;
                distLat = collisionPos.dist;
            }
            
            genAccelMode = GenAccelModeSwitch(collisionInterval, distLon, dist2AccuracyControl);
            sobjkeyInfo keyobj;
           
            genAccelMode = 0;     // TODO @PQG 
            if (genAccelMode == 0)
            {
                keyobj = GenAccelerationFC(PathPoints, keyObjs[i], egoSpeed, pathNum);
            }
            else if (genAccelMode == 1)
            {
                keyobj = GenAccelerationAC(PathPoints, keyObjs[i], egoSpeed, pathNum);
            }

            if (followingObj.targetAcceleration > keyobj.targetAcceleration)
            {
                followingObj = keyobj;
            }
            motionVars.genAccelMode = genAccelMode;

        }
    }
    
    GenVelocityByAccel(PathPoints, followingObj);

    if (fabs(followingObj_last. id - followingObj.id) > 0.5)
    {
        followingObjChange = true;
        VelocityReplanObj = true;
    }
    
    followingObj_last = followingObj;
    egoSpeedLast = egoSpeed;
    speedObj_last = speedObj; //TODO @pqg
}

void motionplanner::GenVelocityByAccel(std::vector<sPoint> &PathPoints, sobjkeyInfo objInfo)
{
    int debugNUM1 = 10;
    if ( PathPoints.size() < 10)
    {
        debugNUM1 = PathPoints.size();
    }
    // for (int debug_i = 0; debug_i < debugNUM1; debug_i++) 
    // {
    //    // cout << "TEST0315+++OriginSpeed: " << debug_i << " :" << PathPoints[debug_i].pathpoints_in.velocity <<  endl;
    // } 
    
    //cout << "TEST0315+++targetPreIndex: " << objInfo.targetPreIndex <<  endl;
    // cout << "Test0311++++++targetIndex: " << objInfo.targetIndex <<  endl;
    // cout << "Test0311+++++++++ObjSpeed: " << speedObj <<  endl;
    // cout << "Test0311+decisionvellimit: " << decision.vellimit <<  endl;

    std::vector<sPoint> PathPointsOriginal;
     PathPointsOriginal= PathPoints ;

    if (fabs(objInfo.targetAcceleration - ACCELERATION_INIT) > 0.1  && fabs(objInfo.targetIndex - INVALIDINT) > 1)
    {
        double targetAccel = SmoothAcceleration(6, objInfo.targetAcceleration);
        targetAccel = MIN(accelPlusMax, targetAccel);
        speedUpdate = true;
        VelocityReplanObj = true;
        targetAccel = targetAccel;
        motionVars.Acceltarget = targetAccel;

        cout<<"TEST0315++targetAccel:"<< targetAccel <<endl;
        
        /*************TODO @yanbo for real car test 20180104*****************/ 
        CalFuzzyInitSpeed(targetAccel, PathPoints[startPointIndex].pathpoints_in.velocity);
        
        cout << "TEST0315+++startPointIndex: " << startPointIndex <<  endl;
        cout << "TEST0315+++fuzzyInitSpeed: " << fuzzyInitSpeed <<  endl;
        
        for (int i = 0; i <= startPointIndex; i++)
        {
            double speedTemp = fuzzyInitSpeed;
            
            // if (PathPoints.size() >= 3)//to make sure that PathPoints[2] exists. 
            // {
            //    if (PathPointsOriginal[1].pathpoints_in.velocity - PathPointsOriginal[2].pathpoints_in.velocity > 0.01)//means deceleration
            //    {
            //        speedTemp = MIN(PathPoints[i].pathpoints_in.velocity , speedTemp);
            //    } 
            // }
            
            if (PathPoints[i].pathpoints_in.velocity > 5/3.6) 
            {
                PathPoints[i].pathpoints_in.velocity = speedTemp;
            }
        }
        //cout << "TEST0315+++Points[0]spd: " << PathPoints[0].pathpoints_in.velocity <<  endl;
        /***************************/

        motionVars.egospeedinit = fuzzyInitSpeed;
        motionVars.THW_FC = startPointIndex;
        
        for (int j = MAX(startPointIndex, 1); j <= objInfo.targetIndex; j++) // +1
        {
            double velocityTemp = 0.01;

            if ( pow(PathPoints[j-1].pathpoints_in.velocity, 2) + 2 * targetAccel * (PathPoints[j].length - PathPoints[j - 1].length) < 0.01 )
            {
                velocityTemp = SPEED_STOP;
            }
            else
            {
                velocityTemp = sqrt(pow(PathPoints[j - 1].pathpoints_in.velocity, 2) + 2 * targetAccel * (PathPoints[j].length - PathPoints[j - 1].length));
            }

            if (velocityTemp < 0.01 || targetAccel <= -5.0)
            {
                velocityTemp = SPEED_STOP;
            }
            
            // if (fuzzyInitSpeed > sbjSpeed)   //It means that acceleration is needed in this moment
            // {
            //     PathPoints[j].pathpoints_in.velocity = velocityTemp;
            // }
            // else 
            // {
            //    PathPoints[j].pathpoints_in.velocity = MIN(PathPoints[j].pathpoints_in.velocity,velocityTemp);
            // }
            // //before 0307
           
            if (j >= objInfo.targetPreIndex)
            {  
                //PathPoints[j].pathpoints_in.velocity = PathPoints[objInfo.targetPreIndex - 1].pathpoints_in.velocity; 
                velocityTemp = PathPoints[objInfo.targetPreIndex - 1].pathpoints_in.velocity; 
                
                if (velocityTemp > speedObj)
                {
                    velocityTemp = speedObj;
                }

                if (objInfo.objType == 2)   //when there are some objs besides ego-car
                {
                    velocityTemp = speedObj ; 
                }

            }
            
            //TODO @PQG 0307  
            if (velocityTemp >= decision.vellimit)
            {
                velocityTemp = decision.vellimit;
            }
            
            if ( PathPointsOriginal[j-1].pathpoints_in.velocity - PathPointsOriginal[j].pathpoints_in.velocity > 0.001) //means deceleration originally
            {
                PathPoints[j].pathpoints_in.velocity = MIN(velocityTemp,PathPoints[j].pathpoints_in.velocity);
            }
            else
            {
                if ( j > objInfo.targetPreIndex + 10 && fuzzyInitSpeed > sbjSpeed )   //It means that acceleration is needed in this moment
                {
                    PathPoints[j].pathpoints_in.velocity = velocityTemp;     //but should not apply in Safe Region 
                }
                else 
                {
                   PathPoints[j].pathpoints_in.velocity = MIN(PathPoints[j].pathpoints_in.velocity,velocityTemp);
                }
            }

            #if 1 //To be tested @pqg 0316  aim to make preview speed !=0 when it need not to stop
            if ( PathPoints[j].pathpoints_in.velocity <= 0.2 && speedObj > 0.2 &&  j < objInfo.targetPreIndex)
            {
                PathPoints[j].pathpoints_in.velocity = PathPoints[j - 1].pathpoints_in.velocity;  //TODO @pqg 0319
            }
            #endif

            #if 1
            if ( speedObj < 0.2 )
            {
               if (objInfo.targetPreIndex *0.1 > 7)
               {
                   if ( PathPoints[j].pathpoints_in.velocity <= 0.2 )
                   {
                      PathPoints[j].pathpoints_in.velocity = PathPoints[j - 1].pathpoints_in.velocity;  //TODO @pqg 0324
                   }
               } 
            }
            
            #endif 

        }
        
        for (int j = objInfo.targetIndex - 1; j < PathPoints.size() - 1; j++)
        {
            double velocityPlus = sqrt(pow(PathPoints[j].pathpoints_in.velocity, 2) + 2 * accelPlusMax *
                                       (PathPoints[j + 1].length - PathPoints[j].length));
            double velocityMinus2 = pow(PathPoints[j].pathpoints_in.velocity, 2) + 2 * accelMinusMax *
                                    (PathPoints[j + 1].length - PathPoints[j].length);

            double velocityMinus = 0;

            if (velocityMinus2 > 0)   //else velocityMinus = 0
            {
                velocityMinus = sqrt(velocityMinus2);
            }

            if( targetAccel >= 0 )
            {
                if (PathPoints[j + 1].pathpoints_in.velocity < velocityMinus)
                {
                    PathPoints[j + 1].pathpoints_in.velocity = velocityMinus;
                }
            }
            else
            {
                if(PathPoints[j + 1].pathpoints_in.velocity > velocityPlus)
                {
                    PathPoints[j + 1].pathpoints_in.velocity = velocityPlus;
                }
            }
        }
    }
    else
    {
        accelSmooth.clear();
        fuzzyInitSpeed = sbjSpeed;
        fuzzyAccelLast = ACCELERATION_INIT;
        speedObj = VELOCITY_INVALID;
    }

    //motionPlannerBackup =  PathPoints;
    
    int debugNUM2 = 10;
    if ( PathPoints.size() < 10)
    {
        debugNUM2 = PathPoints.size();
    }
    // for (int debug_j = 0; debug_j < debugNUM2; debug_j++) 
    // {
    //     cout << "TEST0315+++FinalSpeed: " << debug_j << " :" << PathPoints[debug_j].pathpoints_in.velocity <<  endl;
    // }
}

int motionplanner::GenAccelModeSwitch(double collosionTime, double dist, double distPreview)
{
    int genAccelMethod = 0;
    double THW2AccuracyControl = 1.5;
    double timeWindow = 0.5;
    double distWindow = 5.0;

    if (collosionTime < THW2AccuracyControl || dist < distPreview)  // || sbjSpeed < 1.5) {
    {
        genAccelMethod = 1;
    }
    else if (collosionTime > THW2AccuracyControl + timeWindow && dist > distPreview + distWindow)
    {
        genAccelMethod = 0;
    }

    return genAccelMethod;
}

sobjkeyInfo motionplanner::GenAccelerationFC(std::vector<sPoint> &PathPoints, sObjFocus &Obj, double egoSpeed, int pathNum)
{
    sobjkeyInfo keyObj;
    sObjPos objPos;
    keyObj.targetAcceleration = ACCELERATION_INIT;
    keyObj.targetIndex = INVALIDINT;
    keyObj.targetPreIndex = INVALIDINT;
    keyObj.id = 99;
    keyObj.objType = 99;
    double accelerationFC = ACCELERATION_INIT;
    int objTypeJudge = 99;
    
    if (PathPoints.size() - startPointIndex > 1)
    {

        double keyObjVrelLon = Obj.objfocus.xvrel;
        double keyObjVLon = Obj.objfocus.xvrel + egoSpeed;
        double objDistLon = INVALIDDOUBLE;
        double objDistLat = INVALIDDOUBLE;
        double THW = 0.7 * GetFollowingTime(keyObjVrelLon);
        THW = MIN(1.8, THW);
        THW = MAX(2.0, THW);
        int objPosIndex = PathPoints.size() - 1;
        int objPosPreIndex = PathPoints.size() - 1;
        double ObjPosYNearSide = 0;
        double ObjPosXNearSide = 0;
        double widthRight = 0;
        double widthLeft = 0;

        widthRight = 0.5 * Obj.objfocus.width;
        widthLeft = 0.5 * Obj.objfocus.width;
        keyObjVrelLon = Obj.objfocus.xvrel; //TODO @ yanbo test 1110
        keyObjVLon = Obj.objfocus.xvrel + egoSpeed;
        distSafe = CalcuDistSafe(distPreObj, distPreObjMin, egoSpeed, keyObjVrelLon);
        
        #if 0    //test TODO @pqg 0312
        double accTemp = 0.5*(pow(keyObjVLon,2) -pow(sbjSpeed,2))/ (objDistLon - distSafe);
        double decAcclimit = -1;
        if (accTemp < decAcclimit)
        {
            distSafe = distPreObjMin;
        }

        #endif
        
        //distSafe = 5; //TODO @pqg

        if (Obj.objfocus.positions[0].ymean >= 0)
        {
            ObjPosYNearSide = Obj.objfocus.positions[0].ymean - widthRight;
        }
        else
        {
            ObjPosYNearSide = Obj.objfocus.positions[0].ymean + widthLeft;
        }
        if (ObjPosYNearSide * Obj.objfocus.positions[0].ymean <= 0)
        {
            ObjPosYNearSide = 0;
        }

        ObjPosXNearSide = Obj.objfocus.positions[0].xmean;

        objPos = CalcuObjPos(ObjPosXNearSide, ObjPosYNearSide, PathPoints);
        objDistLat = objPos.dist;
        objPosIndex = objPos.posid;
        objDistLon = PathPoints[objPos.posid].length;
        
        #if 0
        for(int i = 0; i < Obj.objfocus.cell.size(); i++)
        {
            objPos = CalcuObjPos(Obj.objfocus.cell[i].x, Obj.objfocus.cell[i].y, PathPoints);
            if (objDistLat < objPos.dist)
            {
                objDistLat = objPos.dist;
                objPosIndex = objPos.posid;
                objDistLon = PathPoints[objPos.posid].length;
            }
        }
        #endif
        double dist_following = THW * egoSpeed + distSafe;
        double passWidth = 1.2;//CalcuPassWidth(objDistLon, keyObjVrelLon);
        
        motionVars.testVar1 = keyObjVLon;
        
        if (objDistLat > passWidth) 
        {
            objTypeJudge = 2;  //Determine the scene type

            if (fabs(followingObj_last. id - Obj.objfocus.id) > 0.5)
            {
                crossSpeedBase = PathPoints[objPos.posid].pathpoints_in.velocity;
                
            }
            //Determine the base speed for calculating the passing speed 
            //The base speed remains unchanged until the obstacle ID changed
        
            motionVars.testVar = crossSpeedBase;
            
            keyObjVrelLon = CalcuVelocityObjS(objDistLat, keyObjVLon, crossSpeedBase) - egoSpeed;
  
            if (objDistLon < dist_following) 
            {
                objDistLon = dist_following + 0.21;//"0.21" just in order to make distPercentage != -1;
            }

            keyObjVLon = keyObjVrelLon + egoSpeed;

            cout<<"TEST0315:passSpeed:"<<keyObjVLon<<endl;
        }
        else
        {
            objTypeJudge = 1;
        }
        
        //*****emergencyflagCalculation*****// TODO @pqg 0312
        double accEmergency = -4;
        if ( objDistLat < passWidthLower )
        {
            double accTmp = 0.5 * (pow(keyObjVLon,2)-pow(egoSpeed,2)) / objDistLon;
            
            if ( accTmp < accEmergency )
            {
                emergStatus = true ;
            }
            else
            {
                emergStatus = false ;
            }
        }
        else
        {
            emergStatus = false ;
        }
        //*****emergencyflagCalculation*****//
        
        objPosPreIndex = GetPrePointIndex(objPosIndex, startPointIndex, distSafe, PathPoints);

        double distPercentage = 1;
        if (objDistLon - distSafe > 0.2)
        {
            distPercentage = (objDistLon - dist_following) / (objDistLon - distSafe);
        }
        else
        {
            distPercentage = -1;
        }
        
        if ((distPercentage <= 0.7 || objDistLon - distSafe < 2) && keyObjVrelLon <= 20)   //0.7
        {
            accelerationFC = 1.0*fuzzycontrol.realize(keyObjVrelLon, distPercentage, accelCar, keyObjVrelLon);
        }
        else
        {
            accelerationFC=ACCELERATION_INIT;//0201 add @pqg
        }

        // if (objDistLon - distSafe < 0.5 && objDistLat <= passWidth )   //2.0
        // if (objDistLon - distSafe < 0.5 && objDistLat <= passWidth && sbjSpeed > (keyObjVLon + 1/3.6) )   //TODO @PQG 0316
        // {
        //      accelerationFC = -5;
        // }
        if ( keyObjVLon < 0.1)
        {
            if (objDistLon - distSafe < 0.5 && objDistLat <= passWidth )
            {
                accelerationFC = -5;
            }
        }
        else
        {
            if (objDistLon - distSafe < 0.5 && objDistLat <= passWidth && sbjSpeed > (keyObjVLon + 3/3.6) ) 
            {
                accelerationFC = -5;
            }
        }//@pqg TODO 0324
        
        if(keyObj.targetAcceleration >= accelerationFC)
        {
            keyObj.targetAcceleration = accelerationFC;
            keyObj.targetIndex = objPosIndex;
            keyObj.targetPreIndex = objPosPreIndex;
            keyObj.id = Obj.objfocus.id;
            keyObj.objType = objTypeJudge;

            motionVars.accelFollowingFC = keyObj.targetAcceleration;
            motionVars.relspeedFC = keyObjVrelLon;
            motionVars.objspeedFC = keyObjVLon;
            motionVars.distxobj = objDistLon;
            motionVars.distPercentage = distPercentage;
            motionVars.distSafe = distSafe;
            motionVars.distyobj = objDistLat;

        }

        if ( fabs(followingObj_last. id - Obj.objfocus.id) > 0.5 )
        {
            speedObj = keyObjVLon;
        }
        else if ( followingObj_last. id == Obj.objfocus.id )
        {
            if ( fabs(keyObjVLon - speedObj_last) > 0.5 )
            {
                speedObj = keyObjVLon;
            }
            else
            {
                speedObj = speedObj_last;
            }
        }   //0316 TODO @pqg
        
        motionVars.testVar1=speedObj;
        //cout << "test0227:speedObj:" << speedObj << endl;
        //cout << "========" << "test0227" << "========" << endl;
        
    }
    return keyObj;
}

sobjkeyInfo motionplanner::GenAccelerationAC(std::vector<sPoint> &PathPoints, sObjFocus &Obj, double egoSpeed, int pathNum)
{
    sobjkeyInfo keyObj;
    keyObj.targetAcceleration = ACCELERATION_INIT;
    keyObj.targetIndex = INVALIDINT;
    keyObj.targetPreIndex = INVALIDINT;
    keyObj.id = 99;

    sObjPos collisionPos;
    if (PathPoints.size() - startPointIndex > 1)
    {
        double ObjPosYNearSide = 0;
        double ObjPosXNearSide = 0;
        double widthRight = 0;
        double widthLeft = 0;
        double keyObjVLon = 0;
        double keyObjVrelLon = 0;
        double followingTime = INVALIDDOUBLE;
        double collisionInterval = INVALIDDOUBLE;
        double crossPreIndex = INVALIDINT;
        double crossAccel = ACCELERATION_INIT;
        double timeCollisionEgo = INVALIDDOUBLE;
        double timeCollisionObj = INVALIDDOUBLE;

        widthRight = 0.5 * Obj.objfocus.width;
        widthLeft = 0.5 * Obj.objfocus.width;
        keyObjVrelLon = Obj.objfocus.xvrel; //TODO @ yanbo test 1110
        keyObjVLon = Obj.objfocus.xvrel + egoSpeed;
        distSafe = CalcuDistSafe(distPreObj, distPreObjMin, egoSpeed, keyObjVrelLon);

        if (Obj.objfocus.positions[0].ymean >= 0)
        {
            ObjPosYNearSide = Obj.objfocus.positions[0].ymean - widthRight;
        }
        else
        {
            ObjPosYNearSide = Obj.objfocus.positions[0].ymean + widthLeft;
        }
        if (ObjPosYNearSide * Obj.objfocus.positions[0].ymean <= 0)
        {
            ObjPosYNearSide = 0;
        }

        ObjPosXNearSide = Obj.objfocus.positions[0].xmean;

        followingTime = GetFollowingTime(keyObjVrelLon);

        if (Obj.path_id == pathNum && (Obj.roi_id == ROI_FRONTL || Obj.roi_id == ROI_FRONTR))
        {

            collisionPos = CalcuObjPos(ObjPosXNearSide, ObjPosYNearSide, PathPoints);
            if ((collisionPos.dist >= passWidthLower && collisionPos.dist <= passWidthUpper) && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
            {

                double keyPointSpeed = CalcuVelocityObjS(collisionPos.dist, keyObjVLon, egoSpeed);
                crossPreIndex = GetPrePointIndex(collisionPos.posid, startPointIndex, distSafe, PathPoints);

                if (fabs(keyPointSpeed - PathPoints[crossPreIndex].pathpoints_in.velocity) >= 0.8 || keyPointSpeed <= 0.1)
                {
                    double crossAccel = -0.5 * pow(egoSpeed, 2) - pow(keyPointSpeed, 2) / (PathPoints[crossPreIndex].length - PathPoints[startPointIndex].length);
                }
                // crossAccel = MIN(accelPlusMax, crossAccel);
                if (keyObj.targetAcceleration > crossAccel)
                {
                    keyObj.targetAcceleration = crossAccel;
                    keyObj.targetIndex = collisionPos.posid;
                    keyObj.targetPreIndex = crossPreIndex;
                    keyObj.id = Obj.objfocus.id;
                }

            }
            else if (collisionPos.dist < passWidthLower && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
            {
                // double keyPointSpeed = CalcuVelocityObjS(collisionPos.dist, keyObjVLon, egoSpeed);
                crossPreIndex = GetPrePointIndex(collisionPos.posid, startPointIndex, distSafe, PathPoints);
                timeCollisionEgo = GetTHW(PathPoints, crossPreIndex);
                timeCollisionObj = 0;
                collisionInterval = timeCollisionEgo - timeCollisionObj;
                collisionInterval = MAX(0, timeCollisionEgo - timeCollisionObj);

                if (collisionInterval - followingTime < 4.5)
                {
                    double accelTemp = 0.0;
                    double safeTimeInterval = timeCollisionObj + followingTime;
                    double safeTimeIntervalMin = timeCollisionObj + followingTime - 0;
                    double safeTimeIntervalMax = timeCollisionObj + followingTime + 1.5;
                    double accelMin = 0;
                    double accelMax = 0;
                    if (crossPreIndex - startPointIndex > 2)
                    {

                        accelTemp = CalcuAccel2Points(safeTimeInterval, crossPreIndex, egoSpeed, PathPoints);
                        accelMin = CalcuAccel2Points(safeTimeIntervalMin, crossPreIndex, egoSpeed, PathPoints);

                        accelMax = CalcuAccel2Points(safeTimeIntervalMax, crossPreIndex, egoSpeed, PathPoints);

                        if (accelMax * accelMin <= 0)
                        {
                            accelTemp = 0;
                        }
                        else
                        {
                            if(accelMin < 0)
                            {
                                accelTemp = 0.5 * (accelMin + accelTemp);
                            }
                            else
                            {
                                accelTemp = 0.5 * (accelMax + accelTemp);
                            }
                        }

                        accelTemp = MIN(accelMax, accelTemp);

                        accelTemp = MAX(-5, accelTemp);

                    }
                    else
                    {
                        accelTemp = -5.0;
                    }
                    // accelTemp = MIN(accelPlusMax, accelTemp);

                    if (keyObj.targetAcceleration > accelTemp)
                    {
                        keyObj.targetAcceleration = accelTemp;
                        keyObj.targetIndex = collisionPos.posid;
                        keyObj.targetPreIndex = crossPreIndex;
                        keyObj.id = Obj.objfocus.id;

                        motionVars.distSafe = distSafe;
                        motionVars.distxobj = PathPoints[collisionPos.posid].length;
                        motionVars.followingTime = followingTime;
                        motionVars.accelfollowingMaxAC = accelMax;
                        motionVars.accelfollowingMinAC = accelMin;
                        motionVars.safeTimeInterval = safeTimeInterval;
                        motionVars.timeCollisionEgo = timeCollisionEgo;
                        motionVars.timeCollisionObj = timeCollisionObj;
                        motionVars.accelfollowingAC = keyObj.targetAcceleration;
                    }
                }
            }

        }
        else if (Obj.path_id == pathNum && (Obj.roi_id == ROI_FRONT || Obj.roi_id == ROI_FRONTPRED))
        {
            double objPredTimeInterval = Obj.objfocus.time_interval;
            double objPredStep = Obj.objfocus.steps;
            objPredTimeInterval = MIN(1, objPredTimeInterval);
            objPredTimeInterval = MAX(0.01, objPredTimeInterval);
            objPredStep = MIN(100, objPredStep);
            objPredStep = MAX(0, objPredStep);
            int objStepsPred = int(timePred / objPredTimeInterval);
            objStepsPred = MIN(objStepsPred, objPredStep);
            //TODO @yanbo test 1128
            // for (int j = 0; j < objStepsPred; j+=5) {
            for (int j = 0; j < objStepsPred; j += 1)
            {
                collisionPos = CalcuObjPos(Obj.objfocus.positions[j].xmean, Obj.objfocus.positions[j].ymean, PathPoints);

                if (collisionPos.dist < passWidthLower && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                {
                    crossPreIndex = GetPrePointIndex(collisionPos.posid, startPointIndex, distSafe, PathPoints);
                    timeCollisionEgo = GetTHW(PathPoints, crossPreIndex);
                    
                    timeCollisionObj = j * Obj.objfocus.time_interval;
                    
                    collisionInterval = MAX(0, timeCollisionEgo - timeCollisionObj);

                    if (collisionInterval - followingTime < 4.5)
                    {
                        double accelTemp = 0.0;
                        double safeTimeInterval = timeCollisionObj + followingTime;
                        double safeTimeIntervalMin = timeCollisionObj + followingTime - 0.0;
                        double safeTimeIntervalMax = timeCollisionObj + followingTime + 1.5;
                        double accelMin = 0;
                        double accelMax = 0;
                        if (crossPreIndex - startPointIndex > 2)
                        {

                            accelTemp = CalcuAccel2Points(safeTimeInterval, crossPreIndex, egoSpeed, PathPoints);

                            accelMin = CalcuAccel2Points(safeTimeIntervalMin, crossPreIndex, egoSpeed, PathPoints);

                            accelMax = CalcuAccel2Points(safeTimeIntervalMax, crossPreIndex, egoSpeed, PathPoints);

                            // if (accelMax * accelMin <= 0 || (collisionInterval >= safeTimeIntervalMin && collisionInterval <= safeTimeIntervalMax)) {
                            if (accelMax * accelMin <= 0)
                            {
                                accelTemp = 0;
                            }
                            else
                            {
                                if(accelMin < 0)
                                {
                                    accelTemp = 0.5 * (accelMin + accelTemp);
                                }
                                else
                                {
                                    accelTemp = 0.5 * (accelMax + accelTemp);

                                }
                            }

                            //accelTemp = MIN(accelMax, accelTemp);
                            // if (accelTemp >= 0.5 && keyObjVrelLon <= -1.0) {
                            // 	accelTemp = 0.2;
                            // }
                            accelTemp = MAX(-5, accelTemp);

                        }
                        else
                        {
                            accelTemp = -5.0;
                        }

                        // accelTemp = MIN(accelPlusMax, accelTemp);

                        if (keyObj.targetAcceleration > accelTemp)
                        {
                            keyObj.targetAcceleration = accelTemp;
                            keyObj.targetIndex = collisionPos.posid;
                            keyObj.targetPreIndex = crossPreIndex;
                            keyObj.id = Obj.objfocus.id;

                            motionVars.distSafe = distSafe;
                            //motionVars.distxobj = PathPoints[collisionPos.posid].length;
                            motionVars.followingTime = followingTime;
                            motionVars.accelfollowingMaxAC = accelMax;
                            motionVars.accelfollowingMinAC = accelMin;
                            motionVars.safeTimeInterval = safeTimeInterval;
                            motionVars.timeCollisionEgo = timeCollisionEgo;
                            motionVars.timeCollisionObj = timeCollisionObj;
                            motionVars.accelfollowingAC = keyObj.targetAcceleration;
                        }
                        
                    }
                }
            }

        }
    
        return keyObj;
    }

}

void motionplanner::FocusingObjs(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict &objsP, double egoSpeed, int pathNum)
{
    objsFocus.clear();
    objsROI.objects.clear();
    // don't distinguish objs type TODO @ yanbo
    // 1. Focus objs in ROI;
    CollectObjsROI(PathPoints, objsP, pathNum);

    // 2. Select key objs in ROI;
    SelectKeyObjs(PathPoints, objsROI, pathNum);

    // 3. Key Objs track
    TrackingKeyObjs(PathPoints, objsFocus);

}
void motionplanner::CollectObjsROI(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict objs, int pathNumber)
{

    if (objs.objects.size() > 0)
    {
        ivpredict::predictobj objROI;
        sObjPos collisionPos;

        for (int i = 0; i < objs.objects.size(); i++)
        {

            double objPredTimeInterval = objs.objects[i].time_interval;
            double objPredStep = objs.objects[i].steps;
            objPredTimeInterval = MIN(1, objPredTimeInterval);
            objPredTimeInterval = MAX(0.01, objPredTimeInterval);
            objPredStep = MIN(100, objPredStep);
            objPredStep = MAX(0, objPredStep);
            int objStepsPred = int(timePred / objPredTimeInterval);

            objStepsPred = MIN(objStepsPred, objPredStep);
            // TODO @yanbo 20180108 objStepsPred limit
            double rearWidth = passWidthUpper;

            if(pathNumber == PATH_MAP)//
            {
                rearWidth = passWidthLower;
            }
            if(objs.objects[i].positions[0].xmean + carLength < 0 && (fabs(objs.objects[i].positions[0].ymean) < rearWidth || objs.objects[i].vrel < 0))  //rear_relspeed +- TBD
            {
                continue;
            }
            else
            {
                if(objStepsPred > 0)
                {
                    for (int j = 0; j < objStepsPred; j++)
                    {
                        collisionPos = CalcuObjPos(objs.objects[i].positions[j].xmean, objs.objects[i].positions[j].ymean, PathPoints);
                        if (collisionPos.dist <= passWidthUpper && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                        {
                            // double timeCollisionEgo = GetTHW(PathPoints, collisionPos.posid);
                            // double timeCollisionObj = j * objs.objects[i].time_interval;

                            objROI = objs.objects[i];
                            break;
                        }
                    }
                }
            }
            objsROI.objects.push_back(objROI);
        }
    }
}

void motionplanner::SelectKeyObjs(std::vector<sPoint> &PathPoints, ivpredict::ivmsgpredict objs, int pathNumber)
{
    sObjFocus objTemp;
    sObjPos collisionPos;
    double costKeyObjsROI01 = INVALIDDOUBLE;
    int indexKeyObjROI01 = INVALIDINT;
    double costKeyObjsROI02 = INVALIDDOUBLE;
    int indexKeyObjROI02 = INVALIDINT;
    double costKeyObjsROI03 = INVALIDDOUBLE;
    int indexKeyObjROI03 = INVALIDINT;
    double costKeyObjsROI04 = INVALIDDOUBLE;
    int indexKeyObjROI04 = INVALIDINT;
    double deltaTimeInterval = FosusTimeInterval(); //TODO

    if (objs.objects.size() > 0)
    {

        for (int i = 0; i < objs.objects.size(); i++)
        {

            double objPredTimeInterval = objs.objects[i].time_interval;
            double objPredStep = objs.objects[i].steps;
            objPredTimeInterval = MIN(1, objPredTimeInterval);
            objPredTimeInterval = MAX(0.01, objPredTimeInterval);
            objPredStep = MIN(100, objPredStep);
            objPredStep = MAX(0, objPredStep);
            int objStepsPred = int(timePred / objPredTimeInterval);
            objStepsPred = MIN(objStepsPred, objPredStep);

            double collisionInterval = 0;
            double distLon = 0;
            double distLat = 0;
            double gainCollisionInterval = 0;
            double gainDistLon = 0;
            double gainDistLat = 0;

            if (objStepsPred > 0)
            {

                for(int j = 0; j < objStepsPred; j++)
                {
                    collisionPos = CalcuObjPos(objs.objects[i].positions[j].xmean, objs.objects[i].positions[j].ymean, PathPoints);
                    if (0 == j)
                    {
                        double timeCollisionEgo = GetTHW(PathPoints, collisionPos.posid);
                        double timeCollisionObj = j * objs.objects[i].time_interval;
                        collisionInterval = timeCollisionEgo - timeCollisionObj;
                        distLon = PathPoints[collisionPos.posid].length;
                        distLat = collisionPos.dist;

                        // 1. ROI-1
                        if (collisionPos.dist <= passWidthLower && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                        {
                            gainCollisionInterval = 1;
                            gainDistLon = 1;
                            gainDistLat = 0.3;
                            double costKeyObjsTemp = gainCollisionInterval * collisionInterval + gainDistLon * distLon + gainDistLat * distLat;
                            
                            if (costKeyObjsROI01 > costKeyObjsTemp)
                            {
                                costKeyObjsROI01 = costKeyObjsTemp;
                                indexKeyObjROI01 = i;
                            }
                            break;
                        }
                        // 2. ROI-3:4
                        else if ((collisionPos.dist > passWidthLower && collisionPos.dist <= passWidthUpper) && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                        {
                            if (objs.objects[i].positions[j].ymean - PathPoints[collisionPos.posid].pathpoints_in.y >= 0)
                            {
                                gainCollisionInterval = 0.1;
                                gainDistLon = 0.1;
                                gainDistLat = 3;
                                double costKeyObjsTemp = gainCollisionInterval * collisionInterval + gainDistLon * distLon + gainDistLat * distLat;
                                if (costKeyObjsROI03 > costKeyObjsTemp)
                                {
                                    costKeyObjsROI03 = costKeyObjsTemp;
                                    indexKeyObjROI03 = i;
                                }
                                break;
                            }
                            else
                            {
                                gainCollisionInterval = 0.1;
                                gainDistLon = 0.1;
                                gainDistLat = 3;
                                double costKeyObjsTemp = gainCollisionInterval * collisionInterval + gainDistLon * distLon + gainDistLat * distLat;
                                if (costKeyObjsROI04 > costKeyObjsTemp)
                                {
                                    costKeyObjsROI04 = costKeyObjsTemp;
                                    indexKeyObjROI04 = i;
                                }
                                break;
                            }
                        }
                        else
                        {

                        }

                    }
                    else
                    {
                        if (collisionPos.dist <= passWidthLower && (PathPoints[collisionPos.posid].length < lengthPreview && PathPoints[collisionPos.posid].length > blindsScope))
                        {
                            double timeCollisionEgo = GetTHW(PathPoints, collisionPos.posid);
                            double timeCollisionObj = j * objs.objects[i].time_interval;
                            collisionInterval = timeCollisionEgo - timeCollisionObj;
                            distLon = PathPoints[collisionPos.posid].length;
                            distLat = collisionPos.dist;
                            if (collisionInterval > deltaTimeInterval)
                            {
                                gainCollisionInterval = 1;
                                gainDistLon = 1;
                                gainDistLat = 0.1;
                                double costKeyObjsTemp = gainCollisionInterval * collisionInterval + gainDistLon * distLon + gainDistLat * distLat;
                                if (costKeyObjsROI02 > costKeyObjsTemp)
                                {
                                    costKeyObjsROI02 = costKeyObjsTemp;
                                    indexKeyObjROI02 = i;
                                }
                                break;
                            }

                        }
                    }
                }
            }
        }

        if (fabs(indexKeyObjROI01 - INVALIDINT) > 1)
        {
            objTemp.objfocus = objs.objects[indexKeyObjROI01];
            objTemp.path_id = pathNumber;
            objTemp.roi_id = ROI_FRONT;
            objsFocus.push_back(objTemp);
        }
        else if (fabs(indexKeyObjROI02 - INVALIDINT) > 1)
        {
            objTemp.objfocus = objs.objects[indexKeyObjROI02];
            objTemp.path_id = pathNumber;
            objTemp.roi_id = ROI_FRONTPRED;
            objsFocus.push_back(objTemp);
        }
        else if (fabs(indexKeyObjROI03 - INVALIDINT) > 1)
        {
            objTemp.objfocus = objs.objects[indexKeyObjROI03];
            objTemp.path_id = pathNumber;
            objTemp.roi_id = ROI_FRONTL;
            objsFocus.push_back(objTemp);
        }
        else if (fabs(indexKeyObjROI04 - INVALIDINT) > 1)
        {
            objTemp.objfocus = objs.objects[indexKeyObjROI04];
            objTemp.path_id = pathNumber;
            objTemp.roi_id = ROI_FRONTR;
            objsFocus.push_back(objTemp);
        }
    }
}

void motionplanner::TrackingKeyObjs(std::vector<sPoint> &PathPoints, std::vector<sObjFocus> &keyObjs)
{
    int lostTimer = 10;
    int holdTimer = 5;
    // smooth filter and objs' life cycle management;
    // keyObjsTrack save final keyObjs;
    trackObjs.clear();

    if (keyObjs.size() > 0)
    {
        for (int i = 0; i < keyObjs.size(); i++)
        {
            keyObjsTrack[keyObjs[i].objfocus.id].ObjpreTrack.push_back(keyObjs[i]);
        }
    }

    for (int i = 0; i <= 255; i++)
    {
        if (keyObjsTrack[i].ObjpreTrack.size() > 0)
        {
            keyObjsTrack[i].cnt_lost++;
            if (keyObjsTrack[i].cnt_lost > lostTimer)
            {
                keyObjsTrack[i].ObjpreTrack.clear();
                keyObjsTrack[i].cnt_lost = 0;
            }
        }
        if (keyObjsTrack[i].ObjpreTrack.size() > 1)
        {
            if (fabs(keyObjsTrack[i].ObjpreTrack[0].objfocus.positions[0].xmean - keyObjsTrack[i].ObjpreTrack[1].objfocus.positions[0].xmean) < 2 &&
                    fabs(keyObjsTrack[i].ObjpreTrack[0].objfocus.positions[0].ymean - keyObjsTrack[i].ObjpreTrack[1].objfocus.positions[0].ymean) < 2 &&
                    fabs(keyObjsTrack[i].ObjpreTrack[0].objfocus.v_abs - keyObjsTrack[i].ObjpreTrack[1].objfocus.v_abs) < 2)
            {
                keyObjsTrack[i].ObjpreTrack.erase(keyObjsTrack[i].ObjpreTrack.begin());
                keyObjsTrack[i].cnt_lost = 0;
            }
            else
            {
                if (keyObjsTrack[i].cnt_lost < holdTimer)
                {
                    keyObjsTrack[i].ObjpreTrack.pop_back();
                }
                else
                {
                    keyObjsTrack[i].ObjpreTrack.erase(keyObjsTrack[i].ObjpreTrack.begin());
                    keyObjsTrack[i].cnt_lost = 0;
                }
            }
        }
        if (keyObjsTrack[i].ObjpreTrack.size() > 0)
        {
            trackObjs.push_back(keyObjsTrack[i].ObjpreTrack[0]);
        }

    }
}

double motionplanner::GetTHW(std::vector<sPoint> &PathPoints, int posId)
{
    double timeCollisionEgo = 0;
    if (posId > startPointIndex && posId < PathPoints.size())
    {
        for(int i = startPointIndex + 1; i <= posId; i++)
        {
            if (PathPoints[i].pathpoints_in.velocity < 0.1)
            {
                //timeCollisionEgo = INVALIDDOUBLE;
                // break;
            }
            else
            {
                timeCollisionEgo += 2 * (PathPoints[i].length - PathPoints[i - 1].length) / (PathPoints[i].pathpoints_in.velocity + PathPoints[i - 1].pathpoints_in.velocity);
            }
        }
    }
    else if(posId <= startPointIndex)
    {
        timeCollisionEgo = 0;
    }
    else
    {
        //timeCollisionEgo = INVALIDDOUBLE;
    }

    return timeCollisionEgo;
}

void motionplanner::GenVelocityBasedRoadMark(std::vector<sPoint> &PathPoints)
{
    double velocityTemp = 0;

    if (PathPoints.size() - startPointIndex > 1)
    {
        for (int i = 0; i < PathPoints.size(); i++)
        {
            if ((PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - i < pathEndStopNum) || fabs(PathPoints[i].pathpoints_in.velocity - VELOCITY_INVALID) <= 0.01 || speedUpdate || pathLengthChange || followingObjChange)
            {
                if (PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - i < pathEndStopNum)
                {
                    velocityTemp = SPEED_STOP;
                }
                else
                {
                    if(((PathPoints[i].pathpoints_in.xg - xg_PointS1) * (PathPoints[i].pathpoints_in.xg - xg_PointE1) <= 0 && (PathPoints[i].pathpoints_in.yg - yg_PointS1) * (PathPoints[i].pathpoints_in.yg - yg_PointE1) <= 0) ||
                            ((PathPoints[i].pathpoints_in.xg - xg_PointS2) * (PathPoints[i].pathpoints_in.xg - xg_PointE2) <= 0 && (PathPoints[i].pathpoints_in.yg - yg_PointS2) * (PathPoints[i].pathpoints_in.yg - yg_PointE2) <= 0) ||
                            ((PathPoints[i].pathpoints_in.xg - xg_PointS3) * (PathPoints[i].pathpoints_in.xg - xg_PointE3) <= 0 && (PathPoints[i].pathpoints_in.yg - yg_PointS3) * (PathPoints[i].pathpoints_in.yg - yg_PointE3) <= 0) ||
                            ((PathPoints[i].pathpoints_in.xg - xg_PointS4) * (PathPoints[i].pathpoints_in.xg - xg_PointE4) <= 0 && (PathPoints[i].pathpoints_in.yg - yg_PointS4) * (PathPoints[i].pathpoints_in.yg - yg_PointE4) <= 0))
                    {
                        velocityTemp = speed_lower;
                    }
                    else
                    {
                        velocityTemp = speed_upper;
                    }
                }
                if (i <= startPointIndex)
                {
                    if (sbjSpeed < 0.1)
                    {
                        velocityTemp = 0.1;
                    }
                    else
                    {
                        velocityTemp = sbjSpeed;
                    }
                }
                else
                {
                    velocityTemp = MIN(velocityTemp, decision.vellimit);

                }
                
                PathPoints[i].pathpoints_in.velocity = velocityTemp;
            }

        }
        speedUpdate = false;
        pathLengthChange = false;
        followingObjChange = false;
        VelocityReplanObj = false;

        SmoothVelocity(PathPoints);
    }
    else
    {
        if (PathPoints.size() > 0)
        {
            for (int i = 0; i < PathPoints.size(); i++)
            {
                PathPoints[i].pathpoints_in.velocity = SPEED_STOP;
            }
        }
    }
}

void motionplanner::GenVelocityBasedRoadOld(std::vector<sPoint> &PathPoints)
{
    double velocityTemp = 0.0;

    if (PathPoints.size() - startPointIndex > 1)
    {
        motionVars.curvature = PathPoints[startPointIndex].curvature * 100;
        if ( pathDirection == DRIVE )
        {
            for (int i = 0; i < PathPoints.size(); i++)
            {
                if ((PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - i < pathEndStopNum) ||
                        (fabs(PathPoints[i].pathpoints_in.velocity - VELOCITY_INVALID) <= 0.01 && PathPoints[i].curvature >= 0.0) || speedUpdate || pathLengthChange || followingObjChange)
                {
    
                    if (PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - i < pathEndStopNum)
                    {
                        velocityTemp = SPEED_STOP;
                    }
                    else
                    {
                        
                        for (int k = 0; k < 9; k++)    //(n-1)
                        {
                            if (PathPoints[i].curvature >= tableCurvature1[k] && PathPoints[i].curvature < tableCurvature1[k + 1])
                            {
                                velocityTemp = ((tableVelocity1[k + 1] - tableVelocity1[k]) /
                                                (tableCurvature1[k + 1] - tableCurvature1[k]) *
                                                (PathPoints[i].curvature - tableCurvature1[k]) + tableVelocity1[k]) / 3.6; // km/h to m/s;
                            }
                        }
                    }
                    if (i <= startPointIndex)
                    {
                        if (sbjSpeed < 0.1)
                        {
                            velocityTemp = 0.1;
                        }
                        else
                        {
                            velocityTemp = sbjSpeed;
                        }
    
                    }
                    velocityTemp = MIN(velocityTemp, decision.vellimit);
    
                    PathPoints[i].pathpoints_in.velocity = velocityTemp;
                }
    
            }
        }
        
        
        if (pathDirection == PARKING)  //@pqg TODO 0327
        {
           for (int j = 0; j < PathPoints.size(); j++)
           {
            if ((PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - j < pathEndStopNum) ||
                    (fabs(PathPoints[j].pathpoints_in.velocity - VELOCITY_INVALID) <= 0.01 && PathPoints[j].curvature >= 0.0) || speedUpdate || pathLengthChange || followingObjChange)
            {

                if (PathPoints[PathPoints.size() - 1].length < pathPreview && PathPoints.size() - j < pathEndStopNum)
                {
                    velocityTemp = SPEED_STOP;
                }
                else
                {
                    velocityTemp = 5/3.6;
                }

                if (j <= startPointIndex)
                {
                    if (sbjSpeed < 0.1)
                    {
                        velocityTemp = 0.1;
                    }
                    else
                    {
                        velocityTemp = sbjSpeed;
                    }

                }
                velocityTemp = MIN(velocityTemp, decision.vellimit);

                PathPoints[j].pathpoints_in.velocity = velocityTemp;
            }

           } 
        }

        speedUpdate = false;
        pathLengthChange = false;
        followingObjChange = false;

        SmoothVelocity(PathPoints);
    }
    else
    {
        if (PathPoints.size() > 0)
        {
            for (int i = 0; i < PathPoints.size(); i++)
            {
                PathPoints[i].pathpoints_in.velocity = SPEED_STOP;
            }
        }
    }

}

void motionplanner::GenVelocityBasedRoad(std::vector<sPoint> &PathPoints)
{
    
}

void motionplanner::GetMsg(const ivpathplanner::ivmsgpathplanner &optionPathsIn,
                           const ivpredict::ivmsgpredict &objPredIn,
                           const ivmap::ivmapmsguserfun &objVirtualIn,
                           const ivmap::ivmapmsgobj &objFusionIn,
                           const ivdecision::ivmsgdecision &decisionIn,
                           const ivmap::ivmapmsgvap &ivvapIn,
                           int currentPathDirection)
{
    ivvap = ivvapIn;
    pathDirection = currentPathDirection;

    if (fabs(ivvap.v - VELOCITY_INVALID) < 0.001 || ivvap.v < 0.0)
    {
        ivvap.v = 0.0;
    }
    double sbjspeed_temp = 0;
    
    sbjspeed_temp = SbjspeedFilter(ivvap.v);  //180117 add PQG
    if(pkalman != NULL)
    {
        v_filter = pkalman->getMessureMent(sbjspeed_temp);
    }

    if(pkalman01 != NULL)
    {
        v_filter01 = pkalman01->getMessureMent(sbjspeed_temp);
    }

    sbjSpeed = v_filter[0];     //ivvap.v;
    sbjSpeed = MAX(0, sbjSpeed);
    
    vehSpd=v_filter01[0];          //0131
    accelCar=CalculationAccelCar(vehSpd,vehSpd_last);//0201
    vehSpd_last=vehSpd;
    motionVars.accelCar = accelCar;
    
    /*****Just for test StoreSpd && StoreAccel function @pqg******/
    // double spd_test=StoreSpd(vehSpd);
    // motionVars.testVar = spd_test;
    // double spd_test1=StoreAccel(accelCar);
    // motionVars.testVar1 = spd_test1;
    /***********/

    decision = decisionIn;
    decision.vellimit = 100 / 3.6; //TODO @yanbo
    objPred = objPredIn;
    objVirtual = objVirtualIn;
    objFusion = objFusionIn;
    optionPaths = optionPathsIn;
    motionVars.egospeed = sbjSpeed;
}

void motionplanner::FittingCurve(std::vector<sPoint> &PathPoints)
{
    int fitting_order = 2;
    int fittingpoints_step = 80;
    double curvatureTemp = -0.1;
    std::vector<sPt> sample;
    sample.clear();
    sPt temp;

    if(PathPoints.size() > 0)
    {
        for(int j = 0; j < PathPoints.size(); j++)
        {
            double curvatureFitingEnd = 0;
            // if (PathPoints[PathPoints.size()-1].length > pathPreview || PathPoints.size() - j >= fittingpoints_step)
            if (PathPoints[PathPoints.size() - 1].length > pathPreview)
            {
                temp.x = PathPoints[j].pathpoints_in.x;
                temp.y = PathPoints[j].pathpoints_in.y;
                sample.push_back(temp);
                if (1 == sample.size())
                {
                }
                if(sample.size() == fittingpoints_step)
                {
                    doubleVector Coef;
                    Coef = matool.GetCoeff(sample, fitting_order);
                    for(int n = 0; n < sample.size(); n++)
                    {
                        double y_fit = 0;
                        double dy_fit = 0;
                        double ddy_fit = 0;
                        for(int m = 0; m <= fitting_order; m++)
                        {
                            y_fit += Coef[m] * pow(sample[n].x, m);
                        }
                        for(int m = 1; m <= fitting_order; m++)
                        {
                            dy_fit += m * Coef[m] * pow(sample[n].x, m - 1);
                        }
                        for(int m = 2; m <= fitting_order; m++)
                        {
                            ddy_fit += (m - 1) * m * Coef[m] * pow(sample[n].x, m - 2);
                        }

                        PathPoints[j - sample.size() + n + 1].y_fitting = y_fit;
                        curvatureTemp = fabs(ddy_fit) / sqrt(fabs((1 + dy_fit * dy_fit) * (1 + dy_fit * dy_fit) * (1 + dy_fit * dy_fit)));
                        if (curvatureTemp >= 0.5)
                        {
                            curvatureTemp = 0.5;
                        }
                        if (curvatureTemp <= 0.0)
                        {
                            curvatureTemp = 0.0;
                        }

                        PathPoints[j - sample.size() + n + 1].curvature = 0.8 * curvatureTemp;
                        curvatureFitingEnd = PathPoints[j - sample.size() + n + 1].curvature;
                    }
                    sample.clear();
                }
                else
                {

                }
            }
            else if (PathPoints.size() - j > pathEndStopNum)
            {
                PathPoints[j].curvature = curvatureFitingEnd;
                //cout << "fitting-error" << endl;
            }
            else
            {
                PathPoints[j].curvature = 1;
            }
        }
    }
    else
    {

    }

}

void motionplanner::AnalysisCurve(std::vector<sPoint> &PathPoints)
{

}

void motionplanner::SmoothVelocity(std::vector<sPoint> &PathPoints)
{

    if(PathPoints.size() > startPointIndex + 1)
    {
        double accelMinusTemp = accelMinusMax;
        
        if (PathPoints[PathPoints.size() - 1].length - PathPoints[startPointIndex].length > 0.1)
        {
            accelMinusTemp = -0.5 * pow(sbjSpeed, 2) / (PathPoints[PathPoints.size() - 1].length - PathPoints[startPointIndex].length);
        }
        else
        {
            accelMinusTemp = -5.0;
        }
        
        accelMinusTemp = MIN(accelMinusMax, accelMinusTemp);
        
        for (int i = startPointIndex + 1; i < PathPoints.size(); i++)
        {
            double diffVelocityMax = 0.01;
            double diffVelocity = PathPoints[i].pathpoints_in.velocity - PathPoints[i - 1].pathpoints_in.velocity;

            if(diffVelocity > diffVelocityMax)
            {
                double velocityTemp = sqrt(pow(PathPoints[i - 1].pathpoints_in.velocity, 2) + 2 * accelPlusMax * (PathPoints[i].length - PathPoints[i - 1].length));
                PathPoints[i].pathpoints_in.velocity = MIN(PathPoints[i].pathpoints_in.velocity, velocityTemp);
            }
        }

        for (int i = PathPoints.size() - 1; i > startPointIndex; i--)
        {
            double diffVelocityMax = 0.01;
            double diffVelocity = PathPoints[i - 1].pathpoints_in.velocity - PathPoints[i].pathpoints_in.velocity;

            if(diffVelocity > diffVelocityMax)
            {
                double velocityTemp = sqrt(pow(PathPoints[i].pathpoints_in.velocity, 2) - 2 * accelMinusTemp * (PathPoints[i].length - PathPoints[i - 1].length));
                PathPoints[i - 1].pathpoints_in.velocity = MIN(PathPoints[i - 1].pathpoints_in.velocity, velocityTemp);
            }
        }
    }
}

int motionplanner::SelectPath(const std::vector<std::vector<sPoint> > &optionPathPoints)
{

    if(0 == optionPathPoints.size())
    {
        return 1;
    }
    else
    {
        doubleVector priorityScore = {0.0};
        doubleVector costLength = {0.0};
        doubleVector costTime = {0.0};
        doubleVector costCurvature = {0.0};
        doubleVector costCurvatureMax = {0.0};
        doubleVector costVelociy = {0.0};
        double gainLength = 1.0;
        double gainTime = 1.0;
        double gainCurvature = 2.0;
        double gainVelocity = 3.0;
        double curvatureMax = 1 / radiusMin;
        double priorityScoreMax = 0.0;

        for (int i = 0; i < optionPathPoints.size(); i++)
        {
            for (int j = 1; j < optionPathPoints[i].size(); j++)
            {
                if (optionPathPoints[i][j].pathpoints_in.velocity > 0.1)
                {
                    costTime[i] += (optionPathPoints[i][j].length - optionPathPoints[i][j - 1].length) /
                                   optionPathPoints[i][j].pathpoints_in.velocity;
                }

                costCurvature[i] += optionPathPoints[i][j].curvature;
                costLength[i] += optionPathPoints[i][j].length;

                if (costCurvatureMax[i] > optionPathPoints[i][j].curvature)
                {
                    costCurvatureMax[i] = optionPathPoints[i][j].curvature;
                }
                costVelociy[i] += fabs(optionPathPoints[i][j].pathpoints_in.velocity -
                                       optionPathPoints[i][j - 1].pathpoints_in.velocity);
            }

            priorityScore[i] = 1 / (gainLength * costLength[i] + gainTime * costTime[i] +
                                    gainCurvature * costCurvature[i] + gainVelocity * costVelociy[i]);

            if (costCurvatureMax[i] >= curvatureMax)
            {
                priorityScore[i] = 0;
            }
            if (priorityScoreMax < priorityScore[i])
            {
                priorityScoreMax = priorityScore[i];
                pathID = i;
            }
            pathID = i;
        }
        if (optionPathPoints.size() >= 2)
        {
            pathID = 1;
        }
        else
        {
            pathID = 0;
        }
        return pathID;
    }
}

void motionplanner::GeneratePath(std::vector<std::vector<sPoint> > &optionPathPoints)
{

    finalPath.points.clear();

    #if 1
    ivvap.shiftlvlposition = 3;  //test
    #endif

    if ( ivvap.shiftlvlposition == shiftPosReq)   
    {
       if ((0 == optionPathPoints.size()) || (optionPathPoints[pathID].size() <= 1))
       {
           return;
       }
       for (int i = 0; i < optionPathPoints[pathID].size(); i++)
       {
           motionPoint.x = optionPathPoints[pathID][i].pathpoints_in.x;
           motionPoint.y = optionPathPoints[pathID][i].pathpoints_in.y;
           motionPoint.xc = optionPathPoints[pathID][i].pathpoints_in.xc;
           motionPoint.yc = optionPathPoints[pathID][i].pathpoints_in.yc;
           motionPoint.xg = optionPathPoints[pathID][i].pathpoints_in.xg;
           motionPoint.yg = optionPathPoints[pathID][i].pathpoints_in.yg;
           motionPoint.angle = optionPathPoints[pathID][i].pathpoints_in.angle;
           motionPoint.velocity = optionPathPoints[pathID][i].pathpoints_in.velocity;
           motionPoint.a = optionPathPoints[pathID][i].pathpoints_in.a;
           motionPoint.horn = 0;
           motionPoint.turnlight = 0;
           motionPoint.alertlight = 0;
           motionPoint.emergency_flag = emergStatus;
           if (fabs(optionPathPoints[pathID][i].y_fitting) < 20000)
           {
               motionPoint.y_fit = optionPathPoints[pathID][i].y_fitting;
           }
           else
           {
               motionPoint.y_fit = optionPathPoints[pathID][i - 1].y_fitting;
           }
   
           finalPath.points.push_back(motionPoint);
       }
    }
    else //real shiftposition != shiftposreq
    {
       if ((0 == optionPathPoints.size()) || (optionPathPoints[pathID].size() <= 1))
       {
           return;
       }
       for (int i = 0; i < optionPathPoints[pathID].size(); i++)
       {
           motionPoint.x = optionPathPoints[pathID][i].pathpoints_in.x;
           motionPoint.y = optionPathPoints[pathID][i].pathpoints_in.y;
           motionPoint.xc = optionPathPoints[pathID][i].pathpoints_in.xc;
           motionPoint.yc = optionPathPoints[pathID][i].pathpoints_in.yc;
           motionPoint.xg = optionPathPoints[pathID][i].pathpoints_in.xg;
           motionPoint.yg = optionPathPoints[pathID][i].pathpoints_in.yg;
           motionPoint.angle = optionPathPoints[pathID][i].pathpoints_in.angle;
           motionPoint.velocity = 0;   //@pqg 0327 when real shiftposition != shiftposreq , then set all points 0. 
           motionPoint.a = 0;
           motionPoint.horn = 0;
           motionPoint.turnlight = 0;
           motionPoint.alertlight = 0;
           motionPoint.emergency_flag = emergStatus;
           if (fabs(optionPathPoints[pathID][i].y_fitting) < 20000)
           {
               motionPoint.y_fit = optionPathPoints[pathID][i].y_fitting;
           }
           else
           {
               motionPoint.y_fit = optionPathPoints[pathID][i - 1].y_fitting;
           }
   
           finalPath.points.push_back(motionPoint);
       }
    }
    
    
}

sObjPos motionplanner::CalcuObjPos(double x, double y, std::vector<sPoint> &pathPointss)
{
    sObjPos posInfo;
    double disThre = INVALIDDOUBLE;
    int indexStamp = INVALIDINT;

    for (int i = 0; i < pathPointss.size(); i++)
    {
        double ptX = pathPointss[i].pathpoints_in.x;
        double ptY = pathPointss[i].pathpoints_in.y;
        double dis = sqrt((x - ptX) * (x - ptX) + (y - ptY) * (y - ptY));

        if (disThre > dis)
        {
            disThre = dis;
            indexStamp = i;
        }
        posInfo.dist = disThre;
        posInfo.posid  = indexStamp;
    }
    return posInfo;

}

double  motionplanner::CalcuVelocityObjS(double dist, double objSpeed, double egoSpeed)
{
    // According the obj's classification and  Power field Gaussian distribution
    //double crossSpeed = 0;
    double crossSpeed =egoSpeed;
    objSpeed = MAX(0, objSpeed);

    if (egoSpeed > objSpeed)
    {
        if (dist < passWidthLower)//1.4
        {
            crossSpeed = objSpeed;
        }
        else if (dist < passWidthLower + 0.3)
        {
            crossSpeed = objSpeed + MIN(2, (egoSpeed - objSpeed) * (dist - passWidthLower) / (passWidthUpper - passWidthLower));     
        }
        else if (dist < passWidthUpper)  //2
        {
            crossSpeed = objSpeed + (egoSpeed - objSpeed) * (dist - passWidthLower) / (passWidthUpper - passWidthLower);
        }
    }
    crossSpeed = MIN(crossSpeed, egoSpeed);

    return crossSpeed;
}

int motionplanner::GetPrePointIndex(int collisionIndex, int startIndex, double distPre, std::vector<sPoint> &PathPoints)
{
    int prePointIndex = collisionIndex;

    if(collisionIndex > startIndex + 1)
    {
        for (int i = collisionIndex - 1; i > startIndex; i--)
        {
            double dist = PathPoints[collisionIndex].length - PathPoints[i].length;
            if (dist >= distPre)
            {
                prePointIndex = i;
                break;
            }
            else if (i <= startIndex + 1)
            {
                prePointIndex = startIndex + 1;
            }
        }
    }
    else
    {
        prePointIndex = startIndex + 1;
    }
    return prePointIndex;
}

double motionplanner::GetFollowingTime(double targetRelspeed)
{
    double followingTime = INVALIDDOUBLE;

    if (targetRelspeed < -13)
    {
        followingTime = 5;
    }
    else if(targetRelspeed < -1)
    {
        followingTime = 5 - ( 13 + targetRelspeed) / 4;
    }
    else if(targetRelspeed < 0.5)
    {
        followingTime = 2;
    }
    else if(targetRelspeed < 2)
    {
        followingTime = 2 - (targetRelspeed - 0.5);
    }
    else
    {
        followingTime = 0.5;
    }
    // if (targetRelspeed < -10){
    // 	followingTime = 5;
    // }
    // else if(targetRelspeed < -1){
    // 	followingTime = 5 - (10 + targetRelspeed)/3;
    // }
    // else if(targetRelspeed < 0.5) {
    // 	followingTime = 2;
    // }
    // else if(targetRelspeed < 2) {
    // 	followingTime = 2 - (targetRelspeed - 0.5);
    // }
    // else {
    // 	followingTime = 0.5;
    // }
    return followingTime;
}

double motionplanner::CalcuDistSafe(double distParam, double distMinParam, double egoSpeed, double targetRelspeed)
{

    double dSafe = distParam;
    if (egoSpeed + targetRelspeed < 1)
    {
        dSafe = distParam;
    }
    else
    {
        dSafe = distParam - 2.0 * targetRelspeed;  // in order to make deceleration lower when front vehicle speed higher than sbjspeed but too near to our car  
    }
    
    if (dSafe >= distParam)
    {
        dSafe = distParam;
    }
    if (dSafe <= distMinParam)
    {
        dSafe = distMinParam;
    }
    if (fabs(egoSpeed - SPEED_STOP) <= 0.1)
    {
        dSafe = distParam + 1.0;
    }
    
    #if 0     //TODO @pqg 0320
    if (motionPlannerBackup.size()>0)
    {
       double distanceDrive = (sbjSpeed + egoSpeedLast)/0.05;
       double spd_index = 0;
       for ( int spd_i = 0 ;spd_i< motionPlannerBackup.size(); spd_i++ )
       {
           if ( distanceDrive > (spd_i*0.1) )
           {
               spd_index = spd_i;
               break;
           }
       }
       double spd_temp = 0;
       if ( spd_index < (motionPlannerBackup.size()-1))
       {
          spd_temp = motionPlannerBackup[spd_index].pathpoints_in.velocity + (motionPlannerBackup[spd_index+1].pathpoints_in.velocity - motionPlannerBackup[spd_index].pathpoints_in.velocity)*(distanceDrive-spd_index*0.1)/(0.1);
       }
       else
       {
          spd_temp = motionPlannerBackup[spd_index].pathpoints_in.velocity;
       }
   
       //double error_speed = sbjSpeed - motionPlannerBackup[spd_index].pathpoints_in.velocity;
       double error_speed = sbjSpeed - spd_temp;
       
       if (error_speed > 3/3.6)
       {
           dSafe = dSafe - 0.5*error_speed;
       }
       dSafe = MIN(dSafe,3);
    }

    
    #endif 

    return dSafe;
}

double motionplanner::CalcuAccel2Points(double deltaTime, int PointIndex, double speedInit, std::vector<sPoint> &PathPoints)
{
    double acceleraion = 0.0;

    acceleraion = 2 * (PathPoints[PointIndex].length - speedInit * deltaTime) / pow(deltaTime, 2);

    if ((PathPoints[PointIndex].length - speedInit * deltaTime < 0.0) && (acceleraion > 0.0 || speedInit + acceleraion * deltaTime < 0.0))
    {
        acceleraion = -0.5 * pow(speedInit, 2) / PathPoints[PointIndex].length;
    }
    return acceleraion;

}


double motionplanner::CalcuPassWidth(double distanceObj, double relspeedObj)
{
    double passWidth = 1.9;
    double passWidthDist = 1.9;
    double passWidthVrel = 1.9;
    double passWidthLower = 1.2;
    double passWidthUpper = 1.9;
    double passWidthMin = 1.2;

    if (distanceObj > 10)
    {
        passWidthDist = passWidthLower + 0.03 * (distanceObj - 10);
    }
    else
    {
        passWidthDist = passWidthLower;
    }
    passWidthDist = MAX(passWidthDist, passWidthLower);
    passWidthDist = MIN(passWidthDist, passWidthUpper);

    passWidth = passWidthDist;

    return passWidth;
}

double motionplanner::THW_following(double dist, double egospeed, double distSafe, double THW0, double relspeed)
{
    /********THW_SET*******/
    /*  Adaptive THW for different conditions.  For closing (ccrb or relspeed minus) condition,set buffer area for pre-smooth dec,and to enhance dec for emergency;
     For obs far away EGO condition, variable THW is allowed, if THW is bigger than set threshold ,THW_set will reduce and avoid dist influence;
    */
    double THW_upper = 0.5;
    double THW_lower = 0.5;
    double THW_inc = 0.3;
    double Vrel_THW_inc = -1.5;
    double THW_real = THW0;
    double THW_set = THW0;

    THW_real = (dist - distSafe) / egospeed;

    if(relspeed < Vrel_THW_inc)
    {
        if(THW_real <= THW0 - THW_lower)
        {
            THW_set = 2 * THW0 - 2 * THW_lower + THW_inc - THW_real;
        }
        else
        {
            THW_set = THW_real + THW_inc;
        }
    }
    else
    {
        if(THW_real > THW0 + THW_upper)
        {
            THW_set = 2 * THW0 + 2 * THW_upper - THW_real;
            if(THW_set <= THW0 - THW_upper)
            {
                THW_set = THW0 - THW_upper;
            }
        }
        else if(THW_real > THW0)
        {
            THW_set = THW_real;
        }
        else
        {
            THW_set = THW0;
        }
    }

    if(THW_set >= 2.5)
    {
        THW_set = 2.5;
    }
    if(THW_set <= 1.1)
    {
        THW_set = 1.1;
    }

    return THW_set;
}

double motionplanner::SmoothAcceleration(int filterCnt, double newAcceleration)
{
    double accelTemp = 0.0;
    double accelSum = 0.0;
    accelSmooth.push_back(newAcceleration);
    if (accelSmooth.size() > filterCnt)
    {
        accelSmooth.erase(accelSmooth.begin());
    }
    if (accelSmooth.size() > 0)
    {

        for (int i = 0; i < accelSmooth.size(); i++)
        {
            accelTemp += accelSmooth[i];
        }
        accelTemp = accelTemp / (accelSmooth.size());
    }

    return accelTemp;
}

double motionplanner::SmoothAcceleration1(int filterCnt, double newAcceleration)
{
    double accelTemp = 0.0;
    double accelSum = 0.0;
    deltaSpeedSmooth.push_back(newAcceleration);
    if (deltaSpeedSmooth.size() > filterCnt)
    {
        deltaSpeedSmooth.erase(deltaSpeedSmooth.begin());
    }
    if (deltaSpeedSmooth.size() > 0)
    {

        for (int i = 0; i < deltaSpeedSmooth.size(); i++)
        {
            accelTemp += deltaSpeedSmooth[i];
        }
        accelTemp = accelTemp / (deltaSpeedSmooth.size());
    }

    return accelTemp;
}

/******************************************************
 * Function: AddPathInfo()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Add the length and curvature info for raw optionpath;
 * Input: optionPathsRaw
 * Output: None
 * Return: None
 * Others: None
 *******************************************************/
void motionplanner::AddPathInfo(const ivpathplanner::ivmsgpathplanner &optionPathsRaw)
{
    motOptionPathPoints.clear();
    pathStopFlag.clear();
    if (0 == optionPathsRaw.paths.size())
    {
        return;
    }
    for (int i = 0; i < optionPathsRaw.paths.size(); i++)
    {
        motPathPoints.clear();
        pathStopFlag.push_back(optionPathsRaw.paths[i].stopflag);
        if (1 >= optionPathsRaw.paths[i].points.size())
        {
            return;
        }
        else
        {
            
            float32 dis2Pts = 0.0;
            float32 segLength = 0.0;
            for (int j = 0; j < optionPathsRaw.paths[i].points.size(); j++)
            {
                motPoint.curvature = CARVE_INIT;
                motPoint.pathpoints_in.x = optionPathsRaw.paths[i].points[j].x;
                motPoint.pathpoints_in.y = optionPathsRaw.paths[i].points[j].y;
                motPoint.pathpoints_in.xc = optionPathsRaw.paths[i].points[j].xc;
                motPoint.pathpoints_in.yc = optionPathsRaw.paths[i].points[j].yc;
                motPoint.pathpoints_in.xg = optionPathsRaw.paths[i].points[j].xg;
                motPoint.pathpoints_in.yg = optionPathsRaw.paths[i].points[j].yg;
                motPoint.pathpoints_in.angle = optionPathsRaw.paths[i].points[j].angle;
                motPoint.pathpoints_in.velocity = optionPathsRaw.paths[i].points[j].velocity;
                motPoint.pathpoints_in.a = optionPathsRaw.paths[i].points[j].a;
                if (0 == j)
                {
                    motPoint.length = 0;//GetDist(optionPathsRaw.paths[i].points[j].x, optionPathsRaw.paths[i].points[j].y, 0, 0);
                }
                else
                {
                    dis2Pts = GetDist(optionPathsRaw.paths[i].points[j].x,
                                      optionPathsRaw.paths[i].points[j].y,
                                      optionPathsRaw.paths[i].points[j - 1].x,
                                      optionPathsRaw.paths[i].points[j - 1].y);
                    motPoint.length = segLength + dis2Pts;
                }
                segLength = motPoint.length;
                motPathPoints.push_back(motPoint);
            }
        }
        motOptionPathPoints.push_back(motPathPoints);
    }
}

/******************************************************
 * Function: FittingCurve()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Fitting the path using the least square method;
 * Input: optionPathPoints
 * Output: None
 * Return: None
 * Others: None
 *******************************************************/
void motionplanner::FittingCurve(std::vector<std::vector<sPoint> >  &optionPathPoints)
{

#if 1

    int fitting_order = 2;
    //TODO (@yanbo) : Adaptive method for the Segments'length( or point)
    int fittingpoints_step = 150;
    int fittingInitIndex = 0;
    double fittingLengthMax = 20;
    double fittingLengthMin = 10;
    double fittingRatioThreshold = 1.1;
    int fittingReset = 0;
    std::vector<sPt> sample;
    sample.clear();
    sPt temp;
    if (0 == optionPathPoints.size())
    {
        return;
    }
    for(int i = 0; i < optionPathPoints.size(); i++)
    {
        sample.clear();
        double curvatureTemp = -0.1;
        for(int j = 0; j < optionPathPoints[i].size(); j++)
        {
            if (3 >= optionPathPoints[i].size())
            {
                return;
            }
            temp.x = optionPathPoints[i][j].pathpoints_in.x;
            temp.y = optionPathPoints[i][j].pathpoints_in.y;
            sample.push_back(temp);
            if (1 == sample.size())
            {
                fittingInitIndex = j;
                fittingReset = 1;
            }

            fittingpoints_step = 150;
            if(sample.size() == fittingpoints_step)
            {
                doubleVector Coef;
                Coef = GetCoeff(sample, fitting_order);
                for(int n = 0; n < sample.size(); n++)
                {
                    double y_fit = 0;
                    double dy_fit = 0;
                    double ddy_fit = 0;
                    for(int m = 0; m <= fitting_order; m++)
                    {
                        y_fit += Coef[m] * pow(sample[n].x, m);
                    }
                    for(int m = 1; m <= fitting_order; m++)
                    {
                        dy_fit += m * Coef[m] * pow(sample[n].x, m - 1);
                    }
                    for(int m = 2; m <= fitting_order; m++)
                    {
                        ddy_fit += (m - 1) * m * Coef[m] * pow(sample[n].x, m - 2);
                    }

                    optionPathPoints[i][j - sample.size() + n + 1].y_fitting = y_fit;
                    curvatureTemp = fabs(ddy_fit) / sqrt(fabs((1 + dy_fit * dy_fit) * (1 + dy_fit * dy_fit) * (1 + dy_fit * dy_fit)));
                    if (curvatureTemp >= 0.5)
                    {
                        curvatureTemp = 0.5;
                    }
                    if (curvatureTemp <= 0.0)
                    {
                        curvatureTemp = 0.0;
                    }

                    optionPathPoints[i][j - sample.size() + n + 1].curvature = 1.0 * curvatureTemp;
                }
                sample.clear();
            }
        }
    }
#endif

}


doubleVector motionplanner::GetCoeff(const vector<sPt> &sample, int n)
{
    vector<doubleVector> matFunX;  //矩阵方程
    vector<doubleVector> matFunY;  //矩阵方程
    doubleVector temp;
    double sum;
    int i, j, k;

    //正规方程X
    for (i = 0; i <= n; i++)
    {
        temp.clear();
        for (j = 0; j <= n; j++)
        {
            sum = 0;
            for(k = 0; k < sample.size(); k++)
                sum += pow(sample[k].x, j + i);
            temp.push_back(sum);
        }
        matFunX.push_back(temp);
    }

    //正规方程Y
    for (i = 0; i <= n; i++)
    {
        temp.clear();
        sum = 0;
        for(k = 0; k < sample.size(); k++)
            sum += sample[k].y * pow(sample[k].x, i);
        temp.push_back(sum);
        matFunY.push_back(temp);
    }

    //矩阵行列式变换
    double num1, num2, ratio;
    for (i = 0; i < matFunX.size() - 1; i++)
    {
        num1 = matFunX[i][i];
        for (j = i + 1; j < matFunX.size(); j++)
        {
            num2 = matFunX[j][i];
            ratio = num2 / num1;
            for (k = 0; k < matFunX.size(); k++)
                matFunX[j][k] = matFunX[j][k] - matFunX[i][k] * ratio;
            matFunY[j][0] = matFunY[j][0] - matFunY[i][0] * ratio;
        }
    }

    //计算拟合曲线的系数
    doubleVector coeff(matFunX.size(), 0);
    for (i = matFunX.size() - 1; i >= 0; i--)
    {
        if (i == matFunX.size() - 1)
            coeff[i] = matFunY[i][0] / matFunX[i][i];
        else
        {
            for (j = i + 1; j < matFunX.size(); j++)
                matFunY[i][0] = matFunY[i][0] - coeff[j] * matFunX[i][j];
            coeff[i] = matFunY[i][0] / matFunX[i][i];
        }
    }
    return coeff;
}

/******************************************************
 * Function: GetDist()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Calculate the distance of two points;
 * Input: x0 ,y0, x1, y1
 * Output: None
 * Return: distance
 * Others: None
 *******************************************************/
double motionplanner::GetDist(double x0, double y0, double x1, double y1)
{
    double dist2Pts = sqrt((x0 - x1 ) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    return dist2Pts;
}


/******************************************************
 * Function: GetBasesignals()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Get the vehicle's signal;
 * Input: carStatus
 * Output: None
 * Return: egoVelocity
 * Others: None
 *******************************************************/
// void motionplanner::GetBasesignals(ivmap::ivmapmsgvap &carStatus) {

// 	egoVelocity = carStatus.v;
// 	if (fabs(egoVelocity - VELOCITY_INVALID) < 0.001 || egoVelocity < 0.0) {
// 		egoVelocity = 0.0;
// 	}
// 	//egoVelocity = 4.0; //test
// }


/******************************************************
 * Function: GetDecision()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Get the decision's signal;
 * Input: decision
 * Output: None
 * Return: None
 * Others: None
 *******************************************************/
// void motionplanner::GetDecision(const ivdecision::ivmsgdecision &decision, bool &lidarPchange) {
// 	decisions = decision;
// 	decisions.vellimit = 100/3.6; //TODO : test
// 	lidarSegUpdate = lidarPchange;
// 	if (1 == lidarSegUpdate) {
// 		// cout<<KRED<<"=========$$$$$$========lidarSegUpdate:"<<lidarSegUpdate<<endl;
// 	}
// }
//
void motionplanner::CalFuzzyInitSpeed(double accelFuzzy, double egoSpeed)
{

    if (fabs(fuzzyAccelLast - accelFuzzy) < 0.4 && egoSpeed > 5/3.6)
    {
        if (accelFuzzy * fuzzyAccelLast <= 0.0)
        {
            fuzzyInitSpeed = egoSpeed;
        }
        double deltaSpeed = egoSpeed - fuzzyInitSpeed;
        if (deltaSpeed * accelFuzzy >= 0.0)
        {
            fuzzyInitSpeed = egoSpeed;
        }
    }
    else
    {
        fuzzyInitSpeed = egoSpeed;
    }

    fuzzyAccelLast = accelFuzzy;

}

double motionplanner::FosusTimeInterval()
{
    //TBD
    double deltaTimeInterval = -1;

    return deltaTimeInterval;
}

double motionplanner::SbjspeedFilter(double egospeed)
{
  if (((fabs(egospeed - sbjSpeed_last_for_filter) >= 2) || (sbjSpeed_last_for_filter >= 1 && egospeed == 0)) && speedlost_cnt < 20) 
  {
    egospeed = sbjSpeed_last_for_filter;
    ++speedlost_cnt;
    if (speedlost_cnt >= 200) 
    {
      speedlost_cnt = 200;
    }
  }
  else 
  {
    sbjSpeed_last_for_filter = egospeed;
    speedlost_cnt = 0;
  }
  return egospeed;
}

double motionplanner::StoreAccel(double Accel)
{
    static double accelStore[10]={0};
    accelStore[9]=Accel;
    for (int i=0;i<9;i++)
    {
        accelStore[i]=accelStore[i+1];
    }
    return accelStore[0];
}

double motionplanner::StoreSpd(double spd)
{
    static double spdstore[10]={0};
    spdstore[9]=spd;
    for (int i=0;i<9;i++)
    {
        spdstore[i]=spdstore[i+1];
    } 
    return spdstore[0];
}

double motionplanner::CalculationAccelCar(double spd,double spd_last)
{
    double accelTemp = 20 * (spd - spd_last);
    double accelerationCar = 0;
    //accelerationCar = SmoothAcceleration1(10, accelerationCar);
    //motionVars.Dsafe = accelTemp;
    int filterCnt = 30;
    int fitting_order = 2;
    carAccel.push_back(accelTemp);

    if (carAccel.size() > filterCnt) 
    {
        carAccel.erase(carAccel.begin());
    }
    std::vector<sPt> sample;
    sample.clear();
    std::vector<double> accels_fit;
    accels_fit.clear();

    if (carAccel.size() > 0) 
    {
      sPt temp;
      for (int i = 0; i < carAccel.size(); i++) 
      {
        temp.x = i;
        temp.y = carAccel[i];
        sample.push_back(temp);
      }
    }

    if (sample.size() > 10) 
    {
      doubleVector Coef;
      Coef = GetCoeff(sample, fitting_order);

      for (int i = 0; i < sample.size(); i++) 
      {
        double accel_fit = 0;
        for (int j = 0; j <= fitting_order; j++) 
        {
          accel_fit += Coef[j] * pow(sample[i].x, j);
        }
        accels_fit.push_back(accel_fit);
      }
      accelerationCar = accels_fit[accels_fit.size() - 1];
    }
    else 
    {
      accelerationCar = accelTemp;
    }
    
    return accelerationCar; 
}

void motionplanner::GenVelocityVirtualObj(std::vector<sPoint> &PathPoints, const ivmap::ivmapmsguserfun &objVirtual, double egospeed) 
{
  double distXVirtualMin = INVALIDDOUBLE;
  double posidVirtualMin = INVALIDDOUBLE;
  double distXVirtualPark = INVALIDDOUBLE;
  double distXVirtualStop = INVALIDDOUBLE;
  int posidVirtualPark = INVALIDINT;
  int posidVirtualStop = INVALIDINT;
  // double accelParking = 0;
  // double accelStop = 0;
  // double accelVirtual = 0;
  double accelParking = INVALIDDOUBLE;
  double accelStop = INVALIDDOUBLE;
  double accelVirtual = INVALIDDOUBLE;
  //double distXObjVirtualScope = egospeed * 6 + 20; //TODO (@yanbo):Param for hSV 60km/h
  double distXObjVirtualScope = egospeed * egospeed /2 + 20; //TODO  
  
  if (objVirtual.obj.size() > 0 && PathPoints.size() > 1) 
  {
    for (int k = 0; k < objVirtual.obj.size(); k++) 
    {
      if (4 == objVirtual.obj[k].type) 
      {
        sObjPos posInfo;
        double x = objVirtual.obj[k].x + 5;
        double y = objVirtual.obj[k].y;
        posInfo = CalcuObjPos(x, y, PathPoints);

        motionVars.distxobj = x;

        if (distXVirtualPark > PathPoints[posInfo.posid].length && posInfo.dist <= 4 && PathPoints[posInfo.posid].length < distXObjVirtualScope) 
        {
          distXVirtualPark = PathPoints[posInfo.posid].length;
          posidVirtualPark = posInfo.posid;

          //motionVars.distxobj = distXVirtualPark;

          if (distXVirtualPark > 5.2) 
          {
            int accelCalswitch = 0;

            if (accelCalswitch == 0)
            {
                
                double relSpdVirt = 0 - egospeed;
                double THWVirt = 0.7 * GetFollowingTime(relSpdVirt);
                THWVirt = MIN(1.8, THWVirt);
                THWVirt = MAX(2.0, THWVirt); 
                double distFollowing = THWVirt * egospeed + 5;
                double relDist = (distXVirtualPark - distFollowing) / (distXVirtualPark - 5);
                
                motionVars.distPercentage = relDist;
                
                accelParking = 1.0*fuzzycontrol.realize(relSpdVirt, relDist, accelCar, relSpdVirt);
                
                motionVars.accelFollowingFC = accelParking;

                //cout << "test0227:distPercentage:" << relDist<< endl;
                //cout << "test0227:accelFollowingFC:" <<accelParking<< endl;
            }
            else if (accelCalswitch == 1)
            {
                accelParking = -0.5 * egospeed * egospeed /(distXVirtualPark - 5);
            }
            
          }
          else 
          {
            accelParking = -2;
          }
        }
        //cout << "test0227:egospeed:" << egospeed << endl;
        //cout << "test0227:objVirtDist:" << x << endl;

      }
      
      if (6 == objVirtual.obj[k].type) 
      {
        sObjPos posInfo;
        double accelStop = -0.6;
        double x = -0.5 * egospeed * egospeed / accelStop + 5;
        double y = 0;
        posInfo = CalcuObjPos(x, y, PathPoints);

        if (distXVirtualStop > PathPoints[posInfo.posid].length && PathPoints[posInfo.posid].length < distXObjVirtualScope) 
        {
          distXVirtualStop = PathPoints[posInfo.posid].length;
          posidVirtualStop = posInfo.posid;
        }
      }
    }
    
    if (accelParking < accelStop) 
    {
      distXVirtualMin = distXVirtualPark;
      posidVirtualMin = posidVirtualPark;
      accelVirtual = accelParking;
    }
    else 
    {
      distXVirtualMin = distXVirtualStop;
      posidVirtualMin = posidVirtualStop;
      accelVirtual = accelStop;
    }
    
    //cout << "test0227:accelVirtualRaw:" << accelVirtual<< endl;
    if ( fabs(accelVirtual - INVALIDDOUBLE)>0.01 )  //TODO 0227 @PQG 
    {
        if (0)
        {
            accelVirtual = MIN(accelVirtual, -0.6);//accelMinusMax);
        }
        
        accelVirtual = MAX(accelVirtual, -2);
    }
    
    // accelVirtual = MIN(accelVirtual, accelMinusMax);
    // accelVirtual = MAX(accelVirtual, -2);

    //cout << "test0227:accelVirtual:" << accelVirtual<< endl;

    //if (distXVirtualMin < distXObjVirtualScope)
    if (distXVirtualMin < distXObjVirtualScope && fabs(accelVirtual - INVALIDDOUBLE)>0.01 ) //TODO 0227 @PQG 
    {
      // VelocityReplanObj = true;
      PathPoints[posidVirtualMin].pathpoints_in.velocity = SPEED_STOP;
      int posid_preobs = 0;
      double distPreObjThred = 5;//distPreObjDecr;
      
      for (int h = posidVirtualMin; h > 0; h--) 
      {
        double dist = GetDist(PathPoints[posidVirtualMin].pathpoints_in.x, 
                              PathPoints[posidVirtualMin].pathpoints_in.y, 
                              PathPoints[h].pathpoints_in.x, 
                              PathPoints[h].pathpoints_in.y);
        if (dist >= distPreObjThred || h <= startPointIndex) 
        {
          posid_preobs = h;
          break;
        }
      }
      
      for (int n = posid_preobs; n <= posidVirtualMin; n++) 
      {
        if (PathPoints[n].pathpoints_in.velocity > PathPoints[posidVirtualMin].pathpoints_in.velocity) 
        {
            PathPoints[n].pathpoints_in.velocity = PathPoints[posidVirtualMin].pathpoints_in.velocity;
        }
      } 
      // for (int j = PathPoints.size() - 1; j > 0; j--) 
      // {
      //   double diffVelocityMax = 0.01;
      //   double diffVelocity = PathPoints[j-1].pathpoints_in.velocity - PathPoints[j].pathpoints_in.velocity;

      //   if (diffVelocity > diffVelocityMax) 
      //   {
      //       double velocityTemp = sqrt(PathPoints[j].pathpoints_in.velocity * 
      //       PathPoints[j].pathpoints_in.velocity - 2 * accelVirtual *
      //       (PathPoints[j].length - PathPoints[j-1].length));
      //       if (PathPoints[j-1].pathpoints_in.velocity > velocityTemp) 
      //       {
      //           PathPoints[j-1].pathpoints_in.velocity = velocityTemp;
      //       }
      //   }
      // }
      #if 0
      for (int j = posid_preobs; j > 0; j--) 
      {
        double diffVelocityMax = 0.01;
        double diffVelocity = PathPoints[j-1].pathpoints_in.velocity - PathPoints[j].pathpoints_in.velocity;

        if (diffVelocity > diffVelocityMax) 
        {
            double velocityTemp = sqrt(PathPoints[j].pathpoints_in.velocity * 
            PathPoints[j].pathpoints_in.velocity - 2 * accelVirtual *
            (PathPoints[j].length - PathPoints[j-1].length));
            if (PathPoints[j-1].pathpoints_in.velocity > velocityTemp) 
            {
                PathPoints[j-1].pathpoints_in.velocity = velocityTemp;
            }
        } 
        
      }
      #endif
       
      PathPoints[0].pathpoints_in.velocity = egospeed;

      for (int j = 1; j < posid_preobs; j++)
      {
         
         double velocityTemp = PathPoints[j-1].pathpoints_in.velocity * 
            PathPoints[j-1].pathpoints_in.velocity + 2 * accelVirtual *
            (PathPoints[j].length - PathPoints[j-1].length);
        if (velocityTemp < 0.1)
        {
            velocityTemp = SPEED_STOP;
        }
        else
        {
            velocityTemp = sqrt(velocityTemp);   
        }

         PathPoints[j].pathpoints_in.velocity = velocityTemp;
         PathPoints[j].pathpoints_in.velocity = MAX(PathPoints[j].pathpoints_in.velocity,SPEED_STOP);

     }





      for (int j = posidVirtualMin; j < PathPoints.size() - 1; j++) 
      {
        double diffVelocityMax = 0.01;
        double diffVelocity = PathPoints[j+1].pathpoints_in.velocity - PathPoints[j].pathpoints_in.velocity;

        if (diffVelocity > diffVelocityMax) 
        {
            double velocityTemp = sqrt(PathPoints[j].pathpoints_in.velocity * 
            PathPoints[j].pathpoints_in.velocity + 2 * accelPlusMax *
            (PathPoints[j+1].length - PathPoints[j].length));
            if (PathPoints[j+1].pathpoints_in.velocity > velocityTemp) 
            {
                PathPoints[j+1].pathpoints_in.velocity = velocityTemp;
            }
        }
      }

      for (int i_test = 0; i_test <10; i_test ++ )
      {
         //cout << "test0227:i:" << i_test << "+++++speed:" << PathPoints[i_test].pathpoints_in.velocity << endl;
      }
      
    }
  }

  //cout << "========" << "test0227loop_end" << "========" << endl;
}




/******************************************************
* Function: SmoothCurve()
* Author: Bo Yan
* Date: 2017-6-6
* Description: Smooth the curvatures;
* Input: optionPathPoints
* Output: None
* Return: None
* Others: None
*******************************************************/
void motionplanner::SmoothCurve(std::vector<sPoint> &PathPoints) 
{
    double diffCurvatureMax = 0.002;
    // cout << FYEL("optionPathPoints[0][5].velocity") << optionPathPoints[0][5].pathpoints_in.velocity << endl;
    if (3 >= PathPoints.size()) 
    {
        return;
    }
    for (int j = 1; j < PathPoints.size()-1; j++) 
    {
        if (PathPoints[j].curvature >= 0.0) 
        {
            if ((fabs(PathPoints[j].curvature - PathPoints[j-1].curvature) >= diffCurvatureMax &&
                fabs(PathPoints[j].curvature - PathPoints[j+1].curvature) >= diffCurvatureMax) ||
                PathPoints[j].curvature >= 0.2) 
            {
               PathPoints[j].curvature = PathPoints[j-1].curvature;
            }
            if (PathPoints[j].curvature -PathPoints[j-1].curvature >= 0.01) 
            {
                PathPoints[j].curvature = PathPoints[j-1].curvature + 0.01;
                // cout<< "ivmotionplanner*******Diff_curvature:" << optionPathPoints[i][j].curvature - optionPathPoints[i][j-1].curvature <<endl;
            }
            if (PathPoints[j].curvature - PathPoints[j-1].curvature <= -0.01) 
            {
                PathPoints[j].curvature = PathPoints[j-1].curvature - 0.01;
            }
        }
        
    }
    int smoothNumber = 30;
    if (PathPoints.size() > smoothNumber + 1) 
    {
        for (int m =PathPoints.size()-1; m > 0; m--) 
        {
            double sumCurvatures = 0.0;
            if (m > smoothNumber && PathPoints[m].curvature >= 0.0) 
            {
                for (int k = m; k > (m - smoothNumber); k--) 
                {
                   sumCurvatures += PathPoints[k].curvature;
                }
                PathPoints[m].curvature = sumCurvatures / smoothNumber;             
            }
            else 
            {
               
            }         
        }
    }
        
}