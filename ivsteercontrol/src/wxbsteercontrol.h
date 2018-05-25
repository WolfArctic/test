/*******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
* NodeName: ivsteercontrol
* FileName: steercontrol.h, steercontrol.cpp
* 
* Description: 
* 1. calculate the steering angle to track the target path
* 2. calculate the torque
*
* History: 
* zhangzhuo        18/03/17    1.0.0    build this module. 
********************************************************************************/
#ifndef _WXBSTEERCONTROL_H
#define _WXBSTEERCONTROL_H

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

#include "filter.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"

#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include "ivsteercontrol/ivsteercontrol.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivmap/ivmapmsgvap.h"
#include "steercontrol.h"
/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

const double MAX = 100000;
const double MIN = 0.00001;

class wxbsteercontrol
{
public:
	wxbsteercontrol(ros::NodeHandle mh);
	~wxbsteercontrol();

	void run();
	void StartDataProcess();
	void SubMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg);
	void SubVAP(const ivmap::ivmapmsgvap::ConstPtr &msg);	
	void Initial();
	void CalculateMain();
	void GetTargetPath();
	
	void Init(ros::NodeHandle mh);
    void InitFilter1(); 
    void InitFilter2(); 

	int Signn(double value);
	int GetNearestPointNum(double x_point, double y_point);
	double GetTargetAngle(double lateral_error, double heading_error, 
						  double car_speed, double k_coe);
	double wheel2eps(double wheel_angle);
	double eps2wheel(double eps_angle);
	double GetLength(double x_point1, double y_point1, double x_point2, double y_point2);
	double GetDist2Seg(double x, double y, double x1, double y1, double x2, double y2);
	double LimiteTargetAngle(double original_angle);
	double MeanFilter(double original_angle);
	sPath GetOriginalPath() const;
	sPath GetSmoothPath(sPath &original_path) const;
	sTrackError GetTrackError(int id_nearest_point, double car_x[], double car_y[]);

  private:
	int amount_motion_path_point;
	int amount_target_path_point;
	int i_speed_count;
	int i_steer_count;
	int sc_loop_freq;
	//car parameters
	double length_vehicle;
	double width_vehicle;
	double k_ratio_r;
	double k_ratio_l;
	double steerangle_max_r;
	double steerangle_max_l;
	double vehicle_angle_center;
	double gps_angle_center;
	double torque_max;	
	double x_middle_front;
	double y_middle_front;

	double actuator_steerangle;
	double actuator_speed;
	double car_speed_last;
	double eps_angle_last;
	double target_angle_record[10];

	sPath target_path;

	mean_filter_param actuator_steerangle_filter_param;
	mean_filter_param actuator_speed_filter_param;
    //inte_param speed_integrator_param;


	ivpathplanner::ivmsgpath motionpath;
	ivsteercontrol::ivsteercontrol resultcontrol;	


    FILE* fp1;
	
	ros::Subscriber    sub_;
	ros::Subscriber    sub2_;
	ros::Publisher     pub_;
};

#endif // _WXBSTEERCONTROL_H
