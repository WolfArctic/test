#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_
 
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>

#include "helper.h"
#include "jmt.h"
#include <vector>
#include <math.h>
#include <tuple>
#include<algorithm>


#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
#include "ivpathplanner/pathpoint.h"
#include "ivpredict/ivmsgpredict.h"
#include "../motionplanner/motionplannercom.h"
#include "ivmap/ivmapmsgvap.h"

#define SAFE_WIDTH 2.0
#define FOLLOW_CAR 1
#define NO_CAR 0
#define CUT_IN 3
class BehaviorPlanner
{
public:
	
    BehaviorPlanner();
    ~BehaviorPlanner();
    std::tuple<int,float,float> UpdateBehavior(std::vector<std::vector<sPoint> > &optionPathPoints, ivpredict::ivmsgpredict &objDynamic, double const &current_curve, ivmap::ivmapmsgvap &carStatus);
     vector<double> UpdateVelocity(double const &current_velocity, const std::tuple<int,float,float> behavior, vector<double> const &average_curvature, const RoadType roadtype);
    void behavior_init();

   private:
   	double CURVE_VELOCITY, UTURN_VELOCITY;
    clock_t start_update_;
    bool time_difference(clock_t &current_time );
	   double CalcuPassWidth(double const &distanceObj, double const &relspeedObj, double const &current_curve);
    vector<double> CalcuObjPos(float32 x, float32 y, std::vector<sPoint> &pathPointss);
    double calculate_velocity_no_obstacle(double const &current_velocity, vector<double> const &average_curvature,  const RoadType roadtype);
    double set_target_s(double const &current_velocity, const double target_v_follow_car, const std::tuple<int,float,float> behavior);
    

};

#endif // BEHAVIORPLANNER_H_