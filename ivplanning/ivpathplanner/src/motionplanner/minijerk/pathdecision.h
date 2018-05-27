#ifndef PATHDECISION_H_
#define PATHDECISION_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
#include "ivpathplanner/pathpoint.h"
#include "ivpredict/ivmsgpredict.h"
#include "../motionplanner/motionplannercom.h"
#include "spline.h"
#include "helper.h"
#include "jmt.h"
#include "lib/mathLib.h"
#include "lib/filter.h"
#include "lib/visualization.h"


class PathDecision
{

  public:

    PathDecision();
    ~PathDecision();
    RoadType PathCurve(std::vector<std::vector<sPoint> >  &optionPathPoints,  const double &ego_speed,vector<double> &spline_value);
    sPoint Max_curvature;

  private:
  	vector<sPoint> Max_path_curvature;

  	mean_filter_param mf1;
  	mean_filter_param filter_uturn_size;
    mean_filter_param filter_vlookforward_size;
  	Visualization *visualization;
  	Visualization *visualization_long;
};

#endif // PATHDECISION_H_
