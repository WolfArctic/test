/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: PredictManager.h
Description: 
1. library define
2. static varrible difine
3. PredictManager class declear

History:
<author>    <time>      <version>    <description> 
hui li      17/06/06    1.0.1       modified function names (delete _ )  

************************************************************/

#ifndef PREDICT_MANAGER
#define PREDICT_MANAGER
#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

//self library
#include "glovariable.h"
#include "globaldefine.h"
#include "premethods/motionbased.h"
#include "premethods/predictuserfun.h"
#include "errorcheck/errorcheck.h"
#include "errorcheck/monitor.h"
#include "container/buffer.h" 
#include "premethods/predictuserfun.h"
// #include "premethods/relationMap.h"
// #include "predictmethods/matchpath.h"

//predictor methods
#include "premethods/predictrelline.h"

//evaluator methods
#include "evaluator/pedestrian/direction_decide.h"

//map
#include "map/geotoolfunc.h"
#include "map/refpath.h"
   

using namespace std;
using namespace prediction;

#define SUB_TOPIC_OBJ "ivmapobj"
#define SUB_TOPIC_ROAD "ivmaproad"
#define SUB_TOPIC_VSP "ivmapvsp"
#define SUB_TOPIC_VAP "ivmapvap"
#define SUB_TOPIC_APP "ivmapapp"
#define SUB_TOPIC_USER "ivmapuserfun"
#define SUB_TOPIC_TRAFFIC "ivmaptraffic"
#define SUB_TOPIC_LOCPOS "ivmaplocpos"
#define SUB_TOPIC_LOCTAG "ivmaploctag"
#define PUB_TOPIC_IAPREDICT "ivpredict"

class PredictManager
{
    // Buffer
    friend class buffer;

  public:
    PredictManager(ros::NodeHandle mh);
    ~PredictManager();
    void run();
    int PredictOmega();
    void PredictAbsoluteShort();
    void PredictYuyan(); //0823
    void PredictUserFunc();
    void InitParams();

  public:
    void chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg);
    void chatterCallbackRoad(const ivmap::ivmapmsgroad::ConstPtr &msg);
    void chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg);
    void chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg);
    void chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg);
    void chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg);
    void chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg);
    void chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg);
    void chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg);
    void subCallback_motionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg);
    
    ros::Publisher pub1_;
    ros::Publisher pub_predictDebug;

    // sub
    ros::Subscriber 		sub1_;
    ros::Subscriber 		sub2_;
    ros::Subscriber 		sub3_;
    ros::Subscriber 		sub4_;
    ros::Subscriber 		sub5_;
    ros::Subscriber 		sub6_;
    ros::Subscriber 		sub7_;
    ros::Subscriber 		sub8_;
    ros::Subscriber 		sub9_;
    ros::Subscriber sub_motionplanner;
    
    
    class PredictPedestrian
    {
    public:
        PredictPedestrian();
        ~PredictPedestrian();
        int oemga(ivpredict::objbuffer *pObj,ivmap::ivmapmsglocpos locpos,ivpredict::predictobj* predict_result, ivpredict::predictobj* self_predict);
        ivpredict::predictobj RelShort(ivpredict::objbuffer *pObj,ivmap::ivmapmsglocpos locpos);
        ivpredict::predictobj AbsShort(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap);
    private:
        direction_decide EvaluateStates;
    };

     //added lihui 0616
    class PredictCar
    {
    public:
        PredictCar();
        ~PredictCar();
        ivpredict::predictobj AbsLong(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap);
    };

    class PredictUnknown
    {
    public:
        PredictUnknown();
        ~PredictUnknown();
        ivpredict::predictobj run(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap);
    };

    // Variables
    sIAMap iamap;
    buffer Buffer;
    errorcheck Check;
    ivpathplanner::ivmsgpath Reference_path; //0823
    ivpredict::ivmsgpredict IAPredict_omega;
    ivpredict::ivmsgpredict IAPredict_shortAbs;
    ivpredict::ivmsgpredict IAPredict_shortLong;	//used by long term path predicting
    ivpredict::ivmsgpredict IAPredict_shortForProbal;	//used by long term path predicting
    ivpredict::ivmsgpredict IAPredict_LineRel;   //0823
    geotoolfunc geo_function;
    refpath RefPathRaw;

private:
    double time1,time2;
    ePredictMethod predictmethod;
    double timeObj;
    double timeRoad;
    double timeVsp;
    double timeVap;
    double timeApp;
    double timeUser;
    double timeTraffic;
    double timeLocpos;
    double timeLoctag;
    void MonitorCheck(double TimeNow);
    monitor mainMonitor;

    refpath RefPath;
    int numPathSegs;
    int test_segid;
    bool test_enable;
    bool navigationPlanEnable;
    std::vector<int> segSequency;
    ivpredict::predictobj car_pose;
    // void checkRun(double dif_time);
    // void checkWarningObj();
    // void checkWarningRoad();
    // void checkWarningVsp();
    // void checkWarningVap();
    // void checkWarningApp();
    // void checkWarningUser();
    // void checkWarningTraffic();
    // void checkWarningLocpos();
    // void checkWarningLoctag();
};


#endif
