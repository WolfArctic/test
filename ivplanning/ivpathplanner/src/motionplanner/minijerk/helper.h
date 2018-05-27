#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"
#pragma once
// Here's the duration period for each path plan we send to the controller

//double MAX_VELOCITY;



const double TIME_INCREMENT = 0.02;
const double REALLY_BIG_NUMBER = 1000000.0;
const double MAX_VELOCITY = 30/3.6;

//behaviorplanner
const double TIME_THRESHOLD_ROADTYPE = 1;//0.5;
const double CURVE_WEIGHT = 0.5;
const double UTURN_WEIGHT = 0.5;
const double SPEED_BUFFER_THESHOLD = 3;
const double TRAVERSE_TIME = 0.2;
const double FRONT_GAP_THRESH = 10.0;
const double FRONT_GAP_SAFE = 15.0;
const double FRONT_BUFFER = FRONT_GAP_THRESH + 35.0; //50m 40 - 60 


//pathdecision
const double SAFE_VELOCITY = 30/3.6;
const double STANDARD_VELOCITY = 30/3.6;
const int NUMBER_OF_CURVE = 150;
const int NUMBER_OF_U_TURN = 20;
const double CURVE_POINT_CURVATURE = 0.02;
const double UTURN_POINT_CURVATURE = 0.3;

//velocitygeneration
const double NO_CAR_DELTA_S = 3;
const double FOLLOW_CAR_DELTA_S = 6;
const double FORWARD_LENGTH = 47.9;
const double VELOCITY_DIFFERNCE = 0.15;
const double DISTANCE_BUFFER = 5.0;



//optimize trajectory
const double MAX_ACCEL = 1.5;
const double MAX_JERK = 1.5;
const double EXPECTED_JERK_IN_ONE_SEC = 0.2; //1.5
const double EXPECTED_ACC_IN_ONE_SEC = 0.5;
const double EXPECTED_DIFF_V_IN_ONE_SEC = 1; //0.5
const double VEHICLE_RADIUS = 1.5;


enum class BehaviorType 
{
  FOLLOW_CAR, AVOID_OBSTACLE, NO_CAR
};

enum class VelocityType
{
	ACCEL, DECEL, STOP
};

enum class RoadType
{
  CURVE, STRIGHT, U_TURN
};

class TrjObject
{
public:
  TrjObject()
  {

  }
  TrjObject(const std::vector<double> &goal_p, const std::vector<double> &unperturb_goal_p,const std::vector<double> &unperturb_s_coeff_p, double t_p, double unperturbed_t_p)
  {
    goal = goal_p;
    unperturb_goal = unperturb_goal_p;
    t = t_p;
    unperturbed_t = unperturbed_t_p;
    unperturb_s_coeff = unperturb_s_coeff_p;
  }

  std::vector<double> goal;
  double t;
  double unperturbed_t;
  std::vector<double> s_coeff;
  std::vector<double> unperturb_s_coeff;
  std::vector<double> unperturb_goal;
};

/* State - stores three doubles p, v, a
 * intended to store position, velocity, and acceleration components in the s, or d axis
 */
struct State 
{
  double p;
  double v;
  double a;
};

/* XYPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds an int n intended to store the number of (x, y) pairs
 */
struct XYPoints 
{
  std::vector<double> xs;
  std::vector<double> ys;
  int n;
};

typedef float float32;

typedef struct pathPoint 
{
  float32 length;
  float32 t;
  float32 max_curvature;
  float32 curvature;
  float32 y_fitting;
  //ivpathplanner::pathpoint pathpoints_in;
} pathPoint;

inline double sigmoid(double const &v, double const &low_value, double const &med_value)
{
  return (2*(med_value-low_value) / (1 + exp(-v)) + low_value);
}

/*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

Useful for cost functions.
 */
inline double logistic(double x)
{
  return 2.0 / (1 + exp(-x)) - 1.0;
}






#endif // HELPERS_H_
