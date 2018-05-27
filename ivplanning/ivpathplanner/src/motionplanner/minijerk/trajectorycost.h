#ifndef TRAJECTORYCOST_H_
#define TRAJECTORYCOST_H_

#include<vector>
#include "helper.h"
#include "jmt.h"
#include <map>
#include <math.h>
#include <algorithm>
#include <iostream>

using namespace std;

#define FOLLOW_CAR 1
#define NO_CAR 0
#define CUT_IN 3

	double TotalAccCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double MaxAccCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double TotalJerkCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double MaxJerkCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double NearestApproachToVehicle(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double VDiffCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose = false);
	double TimeDiffCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose = false);
	double BufferCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);
	double CollisionCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose=false);


#endif