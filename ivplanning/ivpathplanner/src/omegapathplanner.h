//
// Created by idriver on 17-3-10.
//

#ifndef OMEGA_PATHPLANNER_H
#define OMEGA_PATHPLANNER_H
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/topic.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point32.h"
#include <ctime>

#include "ivmap/ivmapmsgobj.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivmap/ivmapmsgstcell.h"
#include "ivmap/ivmapmsgroad.h"
#include "ivmap/ivmapmsgvsp.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/ivmapmsgapp.h"
#include "ivmap/ivmapmsguserfun.h"
#include "ivmap/ivmapmsgtraffic.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgloctag.h"
#include "ivmap/ivmapmsgmapid.h"
#include "ivmap/ivmapmsgnavigation.h"

#include "ivapp/ivappjustmove.h"

#include "ivpredict/ivmsgpredict.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/ivmsgpathplanner.h"
#include "ivpathplanner/ivmsgpathfeedback.h"
#include "ivpathplanner/deliverinfo.h"
#include "ivpathplanner/carstateshow.h"

#include "ivdecision/ivmsgobsavoid.h"
#include "ivdecision/ivmsgdecision.h"

#include "ivlocmsg/ivsensorgps.h"

#include "refpath.h"
#include "motionplannerWBD.h"
#include "../../../avoslib/geotool.h"
#include "../../../avoslib/globalvariables.h"
#include "ivpathplanner/motiondebug.h"

#include "dwaplanner.h"
#include "functions.h"
#include "visualize.h"
#include "monitor_client.h"

#include "sehs.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <bitset>
#include <string>
#include <utility>

using namespace std;

#define SUB_TOPIC_OBJ "ivmapobj"
#define SUB_TOPIC_ROAD "ivmaproad"
#define SUB_TOPIC_VSP "ivmapvsp"
#define SUB_TOPIC_VAP "ivmapvap"
#define SUB_TOPIC_APP "ivmapapp"
#define SUB_TOPIC_USER "ivmapuserfun"
#define SUB_TOPIC_TRAFFIC "ivmaptraffic"
#define SUB_TOPIC_LOCPOS "ivmaplocpos"
#define SUB_TOPIC_LOCTAG "ivmaploctag"
#define SUB_TOPIC_OBSAVOID "ivobsavoid"
#define SUB_TOPIC_DECISION "ivdecision"
#define SUB_TOPIC_PREDICT "ivpredict"
#define SUB_TOPIC_MOVE "ivappjustmove"

#define SUB_TOPIC_NAVIGATION "ivmapnavigation"

#define PUB_TOPIC_IAPATH "ivmotionplanner"
#define PUB_TOPIC_MOTIONPLAN "ivmotionplanner"
#define PUB_TOPIC_FEEDBACK "ivpathfeedback"
#define PUB_TOPIC_MOTIONDEBUG "motiondebug"
#define PUB_TOPIC_DELIVERINFO "ivdeliverinfo"
#define HZ 20
#define SUB_TOPIC_OBJST "ivmapmsgstcell"

using LocPos = ivmap::ivmapmsglocpos;
using LocArea = std::pair<LocPos, LocPos>;

static const int MAX_SIZE = 4;

class omegapathplanner
{
  public:
    //! Constructor.
    omegapathplanner(ros::NodeHandle mh);
    ~omegapathplanner();
    const char *cmdArray[9] = {"restart", "TurnLight_Left", "TurnLight_Right",
                               "HeadLight", "BrakeLight", "ReversingLight",
                               "DoubleFlashLight", "TailLight", "Horn"};
    void run();

  private:
    void chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg);
    void chatterCallbackRoad(const ivmap::ivmapmsgroad::ConstPtr &msg);
    void chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg);
    void chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg);
    void chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg);
    void chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg);
    void chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg);
    void chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg);
    void chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg);
    void chatterCallbackObsavoid(const ivdecision::ivmsgobsavoid::ConstPtr &msg);
    void chatterCallbackDecision(const ivdecision::ivmsgdecision::ConstPtr &msg);
    void chatterCallbcalPredict(const ivpredict::ivmsgpredict::ConstPtr &msg);
    void chatterCallbackGps(const ivlocmsg::ivsensorgps::ConstPtr &msg);
    void chatterCallbackObjstcell(const ivmap::ivmapmsgstcell::ConstPtr &msg);
    void chatterCallbackNavigation(const ivmap::ivmapmsgnavigation::ConstPtr &msg);
    void chatterCallbackMove(const ivapp::ivappjustmove::ConstPtr &msg);

    // Variables
    ivmap::ivmapmsgobj ivobj;
    ivmap::ivmapmsgvsp ivvsp;
    ivmap::ivmapmsgvap ivvap;
    ivmap::ivmapmsgapp ivapp;
    ivmap::ivmapmsguserfun ivuserfun;
    ivmap::ivmapmsgtraffic ivtraffic;
    ivmap::ivmapmsglocpos ivlocpos;
    ivmap::ivmapmsgloctag ivloctag;
    ivmap::ivmapmsgstcell ivstobj;

    ivapp::ivappjustmove ivjustmove;

    ivpathplanner::ivmsgpathplanner optionPaths;
    ivpathplanner::ivmsgpath IAPath;

    ivpathplanner::path LocalRefPath;
    ivpathplanner::path LocalRefPathRaw;
    ivpathplanner::path LocalMeRoad;
    ivpathplanner::path reversePath;

    ivpathplanner::ivmsgpathfeedback pathfeedbackmsg;
    ivpathplanner::deliverinfo delivermsg;

    ivdecision::ivmsgobsavoid obsavoid;
    ivdecision::ivmsgdecision decision;
    ivdecision::ivmsgdecision tmp_decision;
    ivpredict::ivmsgpredict ivpredict;
    ivpathplanner::motiondebug motionvars;

    refpath refCheck;
    refpath RefPath;
    refpath RefPathRaw;
    motionplannerWBD MotionPlan;

    Monitor *mcuMonitor;

    std::vector<int> segSequency;

    int ID_SelectPath;
    int numPathSegs;
    int test_segid;

    int limit_count;
    int reversing_count;
    int failure_count;
    int current_seg;

    bitset<MAX_SIZE> limit_set;

    bool navigationPlanEnable;
    bool test_enable;
    bool reachable_flag;
    bool predict_flag;
    bool ivmap_flag;
    bool decision_flag;
    bool reversing_flag;
    bool ifchange_flag;
    bool emergencyFlag;

    float offset_distance;

    ivpathplanner::path stopping_point;

    std::pair<int, std::string> previous_state;

    ros::Time ivmap_time;
    ros::Time predict_time;
    ros::Time decision_time;
    ros::Time ego_time;
    ros::Time current_time;

    //sub
    ros::Subscriber sub_Obj;
    ros::Subscriber sub_Bdline;
    ros::Subscriber sub_Vsp;
    ros::Subscriber sub_Vap;
    ros::Subscriber sub_App;
    ros::Subscriber sub_User;
    ros::Subscriber sub_Traffic;
    ros::Subscriber sub_Pos;
    ros::Subscriber sub_Tag;
    ros::Subscriber sub_Obs;
    ros::Subscriber sub_Dec;
    ros::Subscriber sub_Pred;
    ros::Subscriber sub_Stcell;
    ros::Subscriber sub_Navigation;
    ros::Subscriber sub_Move;

    //pub
    ros::Publisher pub_Path;
    ros::Publisher pub_Feedback;
    ros::Publisher pub_motiondebug;
    ros::Publisher pub_deliver;
    ros::Publisher pub_carstateshow;

    std::unique_ptr<dwaplanner> dwa;
    std::unique_ptr<sehs> heuristic;
    std::unique_ptr<geotool> gt;
    sorted_tree emergencyObsTree, rawObsTree, dilatedObsTree;
    sorted_tree collid_obstacles;

    blind_tree blind_area;

    std::vector<LocArea> no_avoid_area_;
    bool area_avoid_;

    // functions
    void InitParams();
    void LoadLocalRefPath();
    void EditMap();
    void PubMessage();
    void FreeDriveCheck();
    void NavigationPlan();
    void FailSafeMode();
    void GivewayProcess();
    void ReversingProcess();
    void HMIProcess(bool avoiding_flag, int turning);
    void GetNoAvoidArea(std::string &routemap);
    void OptionAvoid();

    int reachedGoal();
    bool emergencyStop(bool backward);

    int sendCarStateShowToIVACTUTER(const char *cmdString, bool showCmd, int ledcmd);
    void InteractionWindow(std::pair<int, std::string> previous, std::map<int, std::string, greater<int>> current);
    double keepDecimal(const double source, const int reserved = 1) const;
    Site tfBaseLinkToMap(const ivmap::ivmapmsglocpos &ivlocpos, const Site &obj_under_vcs) const;
    Site tfMapToBaseLink(const ivmap::ivmapmsglocpos &ivlocpos, const Site &obj_under_gccs) const;
    void updateBlindTree();
};

#endif
