//
// Created by idriver on 17-3-10.
//

#ifndef OMEGA_DECISION_H
#define OMEGA_DECISION_H
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/ivmapmsgroad.h"
#include "ivmap/ivmapmsgvsp.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/ivmapmsgapp.h"
#include "ivmap/ivmapmsguserfun.h"
#include "ivmap/ivmapmsgtraffic.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgloctag.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/ivmsgpathplanner.h"
#include "ivdecision/ivmsgdecision.h"
#include "ivlocmsg/ivsensorgps.h"

#include "ivcloud/ivcloudgetmapid.h"

#include "../../../avoslib/geotool.h"

#include "rulebased.h"

#include "monitor_client.h"   


//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>

using namespace std;


/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#define SUB_TOPIC_OBJ "ivmapobj"
#define SUB_TOPIC_ROAD "ivmaproad"
#define SUB_TOPIC_VSP "ivmapvsp"
#define SUB_TOPIC_VAP "ivmapvap"
#define SUB_TOPIC_APP "ivmapapp"
#define SUB_TOPIC_USER "ivmapuserfun"
#define SUB_TOPIC_TRAFFIC "ivmaptraffic"
#define SUB_TOPIC_LOCPOS "ivmaplocpos"
#define SUB_TOPIC_LOCTAG "ivmaploctag"
#define SUB_TOPIC_BOUNDARYCELL "ivmapboundarycell"
#define SUB_TOPIC_GETMAPID "ivcloudgetmapid"

#define PUB_TOPIC_DECISION "ivdecision"

#define SUB_TOPIC_GPS "ivsensorgps"

#define HZ 20

struct mapId
{
	ivcloud::ivcloudgetmapid data;
	bool isvalid;
};

class omegadecision
{
public:
    //! Constructor.
    omegadecision(ros::NodeHandle mh);
    ~omegadecision();
    void run();
private:
    
    void chatterCallbackMapId(const ivcloud::ivcloudgetmapid::ConstPtr &msg);
    void chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg);
    void chatterCallbackRoad(const ivmap::ivmapmsgroad::ConstPtr &msg);
    void chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg);
    void chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg);
    void chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg);
    void chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg);
    void chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg);
    void chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg);
    void chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg);
    void chatterCallbackBoundarycell(const ivmap::ivmapmsgbdcell::ConstPtr &msg);
    

    void chatterCallbackGps(const ivlocmsg::ivsensorgps::ConstPtr &msg);

    // Variables
    ivmap::ivmapmsgobj ivobj;
    ivmap::ivmapmsgroad ivroad;
    ivmap::ivmapmsgvsp ivvsp;
    ivmap::ivmapmsgvap ivvap;
    ivmap::ivmapmsgapp ivapp;
    ivmap::ivmapmsguserfun ivuserfun;
    ivmap::ivmapmsgtraffic ivtraffic;
    ivmap::ivmapmsglocpos ivlocpos;
    ivmap::ivmapmsgloctag ivloctag;
    ivmap::ivmapmsgbdcell ivboundarycell;
    
    //Launch parameter
    bool bavoidance;
    bool test_enable;
    
    bool predict_flag;
    bool ivmap_flag;
    bool pathplanning_flag;

    ros::Time ivmap_time;
    ros::Time predict_time;
    ros::Time pathplanning_time;
    ros::Time ego_time;
    
    ivdecision::ivmsgdecision decision;

    ivlocmsg::ivsensorgps ivgps;

    //avoslib variables;
    geotool gt;
    rulebased rule_based;

    Monitor *mcuMonitor;
    
    //sub
    ros::Subscriber 		sub1_;
    ros::Subscriber 		sub2_;
    ros::Subscriber 		sub3_;
    ros::Subscriber 		sub4_;
    ros::Subscriber 		sub5_;
    ros::Subscriber 		sub6_;
    ros::Subscriber 		sub7_;
    ros::Subscriber 		sub8_;
    ros::Subscriber 		sub9_;
    ros::Subscriber 		sub11_;
    ros::Subscriber         sub13_;

    //pub
    ros::Publisher 		pub2_;

    // functions
    void Initial_Param();
    void VelCompute();
    void PubMessage();
    void AvoidObsModel();
    void RefPathSource();

    void FailSafeMode();
};

#endif //PROJECT_PATHPLANNER_H
