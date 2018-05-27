#ifndef VELOCITYGENERATION_H_
#define VELOCITYGENERATION_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include<algorithm>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ivpathplanner/pathpoint.h"
#include "ivpredict/ivmsgpredict.h"
#include "../motionplanner/motionplannercom.h"
#include "spline.h"
#include "helper.h"
#include "jmt.h"
#include "behaviorplanner.h"
#include "trajectory.h"
#include "lib/mathLib.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/mapobject.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivmap/ivmapmsgstcell.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/staticcell.h"
#include "ivpathplanner/velocitydebug.h"
#include "ivmap/ivmapmsguserfun.h"
#include "lib/visualization.h"

#define MOTIONDEBUG true
using namespace std;
class VelocityGeneration
{

  public:

    VelocityGeneration();
    ~VelocityGeneration();
    double SetVelocity(std::vector<std::vector<sPoint> >  &optionPathPoints, ivmap::ivmapmsgvap &carStatus, vector<double> &target, std::tuple<int,float,float> behaviorState, int objFlag, double accelEgo);
    ivpathplanner::velocitydebug velocity_bug;
    //Visualization *visualize_target_s;

    ivpathplanner::velocitydebug getData();
    

  private:
  	//double MAX_VELOCITY;
  	int SetAcceleration(std::vector<std::vector<sPoint> > const &optionPathPoints,double const &current_velocity_target, double &s_start_a, int objMark, double accelCurrent );
    vector<double> convert_velocity_acc_to_originals(vector<State> &jmts,const double originals) const;
  	void MakePath(JMT jmt_s, const double t, const int n, vector<State> &velocity_path);
  	double g_start_v;
  	double ego_velocity;
    Trajectory m_trajectory;
};

 


#endif // VELOCITYGENERATION_H_










