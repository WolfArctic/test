//
// Created by idriver on 17-3-14.
//
#pragma once
#ifndef _MOTIONPLANNER_WBD_H
#define _MOTIONPLANNER_WBD_H

/****Add****/
//c++ lib

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "ivpredict/ivmsgpredict.h"
#include "ivpredict/predictpos.h"
#include "ivpredict/predictobj.h"
#include "ivpathplanner/pathpoint.h"
#include "ivpathplanner/path.h"
#include "ivpathplanner/ivmsgpathplanner.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivpathplanner/motionpoint.h"
#include "ivdecision/ivmsgdecision.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/mapobject.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivpathplanner/motiondebug.h"
#include "ivmap/ivmapmsgvap.h"
#include "../../../../avoslib/globalvariables.h"
#include "motionplannercom.h"


using namespace std;


#define INVALIDDOUBLE 8888.8888
#define PASSWIDTH_UPPER 0.8
#define PASSWIDTH_LOWER 0.5 //0.45
#define PASSWIDTH_LOWER_DY 0.7//0.6
#define PASSWIDTH_UPPER_DY 1.0
#define TSAFE_BEF_COLLISATION_WBD 1
#define TSAFE_AFT_COLLISATION_WBD 2
#define TSAFE_LANE_COLLISATION_WBD 1
#define SPEED_STOP 0.0
#define DIST_PREOBS 3.0//6
#define DIST_PREOBS_MIN 2.5
#define DIST_PREOBS_S 1.3
//#define PI 3.14159265
#define VELOCITY_ISVALID -88.0
#define VELOCITY_CURVE_MIN 1.5/3.6
#define VELOCITY_STRAIGHT_MAX 4/3.6
#define ACCELERATION_INIT_WBD 2.22
#define REALVELOCITY 1
#define VELOCITY_LIDARSTOP -66.0
#define CURVATURE_INIT -1

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

// typedef struct sPoint
// {
//     float32 length;
//     float32 curvature;
//     float32 y_fitting;
//     ivpathplanner::pathpoint pathpoints_in;
// }sPoint;

typedef struct objpos
{
    int32 posid;
    float32 dist;
    float32 time_inter;
}objpos;

typedef struct  pt
{
    float64 x;
    float64 y;
}pt;

typedef std::vector<double> doubleVector;

class motionplannerWBD
{
public:
      float32 velocity_limit;
      float32 velocity_suggest;
      float32 length_segment;
      float32 T_crossin_car;
      float32 T_collision_car[200];
      float32 Aplus_max;
      float32 Aminus_max;
      int32 PathID;
      int32 cnt_test;
      int speed_reset_obs;
      int speed_reset_obs_last;
      int speed_reset_staticmap;
      int speed_reset_staticmap_last;
      float32 A_target_inside_min;
      float32 guidespeed_min;
      double sbjspeed;
      double Dsafe;
      double Distl_obs_min;
      double Distw_obs_min;
      int objsteps_predict;
      int refpath_source;
      int lidar_stop_pathid;
      bool lidar_seg_update;
      int emergency_flag;
      bool isspeedlimit;
      bool static_obs_flag;
      bool speedupdate;
      bool emergency_decision;
      bool avoid_path;
      bool cautious_status;
      int index_start;
      double dist_car;
      double length_last;
      double speedlimit;
      ivpathplanner::ivmsgpathplanner option_paths;
      //ivpathplanner::path path;
      //ivpathplanner::pathpoint pathpoints_in;
      //ivpredict::predictobj objpred;
      ivpathplanner::ivmsgpath finalpath;
      ivpathplanner::motionpoint motionpoint;
      ivpredict::ivmsgpredict obs;
      //ivpredict::ivmsgpredict static_obs;
      ivpredict::ivmsgpredict dynamic_obs;
      sPoint point;
      std::vector<sPoint>  pathpoints;
      std::vector<std::vector<sPoint> >  option_pathpoints;
      doubleVector Stopflag;
      ivpathplanner::motiondebug motionvars;
      ivmap::ivmapmsgvap car_status;
      ivdecision::ivmsgdecision decisions;

      //ivmap::ivmapmsgobj staticobs_map;

public:
     motionplannerWBD();
     ~motionplannerWBD();
     void Init_motionplanner();
     void Velocitygenbasedroad(ivpathplanner::ivmsgpathplanner option_paths_in);
     void Velocitygenbasedobs(std::vector<std::vector<sPoint> > &option_pathpoints,ivpredict::ivmsgpredict obs,ivmap::ivmapmsgbdcell obs_map);
     int32 PathSelect(std::vector<std::vector<sPoint> > option_pathpoints);
     void PathGeneration(std::vector<std::vector<sPoint> > option_pathpoints);//road roadsource???
     void Basesignals_get(ivmap::ivmapmsgvap car_status);
     void Decision_get(ivdecision::ivmsgdecision decision,bool path_passable,bool emergency_f, bool avoid_flag);
private:
     void pathprocess(ivpathplanner::ivmsgpathplanner option_paths_in);
     void curvefitting(std::vector<std::vector<sPoint> > &option_pathpoints);
     void velocitygenetation(std::vector<std::vector<sPoint> > &option_pathpoints);
     void Velocity_staticobs(std::vector<std::vector<sPoint> > &option_pathpoints,ivmap::ivmapmsgbdcell obs_map);
     void Velocity_dynamicobs(std::vector<std::vector<sPoint> > &option_pathpoints,ivpredict::ivmsgpredict obs);
     double Velocity_dynamicobj(std::vector<sPoint> &pathpoints,ivpredict::predictobj objpred);
     objpos objPosCal(float32 x, float32 y, std::vector<sPoint> pathpoints);
     float32 velocity_sobj(float32 dist,float32 velocity_origin);
     doubleVector getCoeff(vector<pt> sample, int n);
     void curvature_cal(std::vector<std::vector<sPoint> > &option_pathpoints);
     double dist_get(double x0,double y0,double x1,double y1);
     bool cautious_path(bool avoid_status, std::vector<sPoint> pathpoints);


};

#endif //_MOTIONSPLANNER_H
