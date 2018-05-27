//
// Created by idriver on 17-3-10.
//

#include "omegadecision.h"

omegadecision::omegadecision(ros::NodeHandle mh)
{
    //sub
    sub1_ =  mh.subscribe(SUB_TOPIC_OBJ,10,&omegadecision::chatterCallbackObj,this);
    sub2_ =  mh.subscribe(SUB_TOPIC_ROAD,10,&omegadecision::chatterCallbackRoad,this);
    sub3_ =  mh.subscribe(SUB_TOPIC_VSP,10,&omegadecision::chatterCallbackVsp,this);
    sub4_ =  mh.subscribe(SUB_TOPIC_VAP,10,&omegadecision::chatterCallbackVap,this);
    sub5_ =  mh.subscribe(SUB_TOPIC_APP,10,&omegadecision::chatterCallbackApp,this);
    sub6_ =  mh.subscribe(SUB_TOPIC_USER,10,&omegadecision::chatterCallbackUser,this);
    sub7_ =  mh.subscribe(SUB_TOPIC_TRAFFIC,10,&omegadecision::chatterCallbackTraffic,this);
    sub8_ =  mh.subscribe(SUB_TOPIC_LOCPOS,10,&omegadecision::chatterCallbackLocpos,this);
    sub9_ =  mh.subscribe(SUB_TOPIC_LOCTAG,10,&omegadecision::chatterCallbackLoctag,this);
    sub11_ = mh.subscribe(SUB_TOPIC_BOUNDARYCELL,10,&omegadecision::chatterCallbackBoundarycell,this);
    sub13_ = mh.subscribe(SUB_TOPIC_GPS,10,&omegadecision::chatterCallbackGps,this);
    
    //pub
    pub2_ = mh.advertise<ivdecision::ivmsgdecision>(PUB_TOPIC_DECISION,10);

    // Parameters initialize

    Initial_Param();

    int node_id = 18;
    mcuMonitor = new Monitor(node_id);
}

omegadecision::~omegadecision()
{
    delete mcuMonitor;
    mcuMonitor = NULL;
}

void omegadecision::Initial_Param()
{
    // Initialise the geotool class
    ros::NodeHandle mh;
    ivlocmsg::ivsensorgps iabasemapTL;
    iabasemapTL.heading = 90;
    iabasemapTL.lon = 8888;
    iabasemapTL.lat = 8888;
    mh.param("iabasemaptllon",iabasemapTL.lon,iabasemapTL.lon);
    mh.param("iabasemaptllat",iabasemapTL.lat,iabasemapTL.lat);
    gt.originGpsOfGCCS = iabasemapTL;

    bavoidance = false;
    mh.param("bavoidance",bavoidance,bavoidance);
    test_enable = false;
    mh.param("test_enable",test_enable,test_enable);

    predict_flag = false;
    ivmap_flag = false;
    pathplanning_flag = false;
    ego_time = ros::Time::now();

}

void omegadecision::VelCompute()
{
    decision.vellimit = 3.0;
    decision.velsuggest = 3.0;
}

void omegadecision::RefPathSource()
{
    decision.refpathsource = 1;
    return;
}

void omegadecision::PubMessage()
{
    if (false == test_enable)
    {
        pub2_.publish(decision);
    }
    rule_based.pubMsg();
}

void omegadecision::run()
{
    ros::Rate rate(HZ);
    while(ros::ok())
    {
        ros::spinOnce();
        ego_time = ros::Time::now();
        VelCompute();
        RefPathSource();
        rule_based.AvoidObsModel(bavoidance);
        PubMessage();
        FailSafeMode();
        rate.sleep();
    }
}

void omegadecision::FailSafeMode()
{
    float time_threshold = 0.05;
    ros::Time current_time = ros::Time::now();
    ros::Duration duration = ros::Duration(0.0);
    if (false == ivmap_flag)
    {
        mcuMonitor->sendWarnning(2, 0);
    }

    current_time = ros::Time::now();
    duration = current_time - ivmap_time;
    if (duration.toSec() > time_threshold + 0.1)
    {
        mcuMonitor->sendWarnning(3, 3);
    }
    else if (duration.toSec() > time_threshold + 0.05)
    {
        mcuMonitor->sendWarnning(3, 2);
    }
    else if (duration.toSec() > time_threshold + 0.02)
    {
        mcuMonitor->sendWarnning(3, 1);
    }
    else if (duration.toSec() > time_threshold + 0.01)
    {
        mcuMonitor->sendWarnning(3, 0);
    }

    if (duration.toSec() > time_threshold + 0.5)
    {
    	ivmap_flag = false;
        mcuMonitor->sendWarnning(2, 0);
    }

    current_time = ros::Time::now();
    duration = current_time - ego_time;
    if (duration.toSec() > time_threshold + 0.1)
    {
        mcuMonitor->sendWarnning(5, 3);
    }
    else if (duration.toSec() > time_threshold + 0.05)
    {
        mcuMonitor->sendWarnning(5, 2);
    }
    else if (duration.toSec() > time_threshold + 0.02)
    {
        mcuMonitor->sendWarnning(5, 1);
    }
    else if (duration.toSec() > time_threshold + 0.01)
    {
        mcuMonitor->sendWarnning(5, 0);
    }

}

/**********************************************part 0: callback functions*****************************************/

void omegadecision::chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivobj = *msg;
}

void omegadecision::chatterCallbackRoad(const ivmap::ivmapmsgroad::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivroad = *msg;
}

void omegadecision::chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivvsp = *msg;
}

void omegadecision::chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivvap = *msg;
}

void omegadecision::chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivapp = *msg;
}

void omegadecision::chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivuserfun = *msg;
}

void omegadecision::chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivtraffic = *msg;
}

void omegadecision::chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivlocpos = *msg;
}

void omegadecision::chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivloctag = *msg;
}

void omegadecision::chatterCallbackBoundarycell(const ivmap::ivmapmsgbdcell::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivboundarycell = *msg;
}

void omegadecision::chatterCallbackGps(const ivlocmsg::ivsensorgps::ConstPtr &msg)
{
    ivgps = *msg;
}
