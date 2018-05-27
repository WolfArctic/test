//
// Created by idriver on 17-3-10.
//

#include "rulebased.h"

rulebased::rulebased()
{
    InitParam();
    ros::NodeHandle nh;
    sub_ = nh.subscribe("ivpathfeedback",10,&rulebased::chatterCallbackPathFeedback,this);
    pub_ = nh.advertise<ivdecision::ivmsgobsavoid>("ivobsavoid",10);
}

rulebased::~rulebased()
{

}

void rulebased::InitParam()
{
    time_start_occupy = ros::Time::now();
    pred_time = ros::Time::now();
    freeTime = ros::Duration(0.0);
    obsavoid.enable = 0;
    obsavoid.obsid = 0;
    ivpathfeedback.freedrive = 1;
    estimating = false;

}


void rulebased::AvoidObsModel(int avoidFlag)
{
    if (0 == avoidFlag)
    {
        obsavoid.enable = 0;
        return;
    }

#if 1
    if (ivpathfeedback.freedrive == 1 && false == estimating)
    {
        time_start_occupy = ros::Time::now();
        obsavoid.enable = 0;
    }
    else
    {
        estimating = true;
        ros::Time Now = ros::Time::now();
        ros::Duration duration = Now - time_start_occupy;
        if (1 == ivpathfeedback.freedrive)
        {
            freeTime += Now - pred_time;
        } 
        if (duration.toSec() > 0.3)
        {
            if (freeTime.toSec() / duration.toSec() < 0.5)
            {
                obsavoid.enable = 1 ;
                if (obsavoid.obsid == ivpathfeedback.avoidobs)
                {
                    obsavoid.obsid ++;
                    freeTime = ros::Duration(0.0);
                    time_start_occupy = ros::Time::now();
                    estimating = false;
                }                
            }
            else
            {
                obsavoid.enable = 0;
                freeTime = ros::Duration(0.0);
            }     
        }
    }
    pred_time = ros::Time::now();
#endif

}

void rulebased::pubMsg()
{
    pub_.publish(obsavoid);
}

void rulebased::chatterCallbackPathFeedback(const ivpathplanner::ivmsgpathfeedback::ConstPtr &msg)
{
    ivpathfeedback = *msg;
}