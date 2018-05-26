#ifndef LOCTOOL_H
#define LOCTOOL_H
#pragma once

#include <math.h>

#include "ivlocmsg/ndt_status.h"
#include "ivlocmsg/ivmsglocpos.h"
#include "ivlocmsg/ivsensorgps.h"
#include "ivlocmsg/ivsensorodom.h"
#include "ivlocmsg/ivsensorimu.h"
#include "common.h"

typedef ivlocmsg::ivsensorgps sGps;
typedef ivlocmsg::ivmsglocpos sLoc;
typedef ivlocmsg::ivsensorodom sOdom;
typedef ivlocmsg::ivsensorimu sImu;
typedef ivlocmsg::ndt_status sStatus;

namespace loclib
{
	class loctool
	{
	public:
		loctool(){};
		~loctool(){};
		void init(double input);
		sGps getGps(sGps oringinGps, sPose2Dd poseInput, double height);
		sPose3Dd get_delta_pose(sGps gps_source, sGps gps_input, double gps_start_angle);
		double degreeToRadian(double v);
		double radianToDegree(double v);
		double getBearingOfGpsPts(sGps source, sGps target);
		double getDisBetweenGpsPoints(sGps pt1, sGps pt2);
		sPoint2Dd BLH2XYZ(double B, double L, double H);
		double GetL0InDegree(double dLIn);
	private:
		double gps_to_frame;
	};
}

#include "loctool.hpp"

#endif
