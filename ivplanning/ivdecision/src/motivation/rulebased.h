//
// Created by idriver on 17-3-10.
//

#ifndef RULEBASED_H
#define RULEBASED_H
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/ivmsgpathfeedback.h"
#include "ivpathplanner/ivmsgpathplanner.h"
#include "ivdecision/ivmsgobsavoid.h"
#include "ivdecision/ivmsgdecision.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>

using namespace std;


class rulebased
{
public:
    //! Constructor.
    rulebased();
    ~rulebased();
    void AvoidObsModel(int avoidFlag);
    void pubMsg();
    void InitParam(); 
    void chatterCallbackPathFeedback(const ivpathplanner::ivmsgpathfeedback::ConstPtr &msg);

private:
    ros::Time time_start_occupy, pred_time;
    ros::Duration freeTime;

    ivpathplanner::ivmsgpathfeedback ivpathfeedback;
    ivdecision::ivmsgobsavoid obsavoid;

    ros::Subscriber sub_;

    ros::Publisher pub_;

    bool estimating;

};

#endif //PROJECT_PATHPLANNER_H
