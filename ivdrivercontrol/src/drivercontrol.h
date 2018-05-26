/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: ivdrivercontrol
* FileName: drivercontrol.h, drivercontrol.cpp
*
* Description:
* 1. Using Pid algrithm to control actuator.
*
* History:
* yanbo         16/11/20    1.0.0    build this module.
******************************************************************************/
#ifndef _DRIVERCONTROL_H
#define _DRIVERCONTROL_H
#pragma once
using namespace std;
#include "ivdrivercontrol.h"
#include "lib/filter.h"
#include "lib/pid.h"
#include "lib/log.h"
#include "lib/mathLib.h"
#include "vehicle_state.h"
#include "fstream"
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/ivmapmsgbdcell.h"
#include "ivmap/ivmapmsgstcell.h"
#define SUB_TOPIC_OBJ "ivmapobj"
#define SUB_TOPIC_OBJST "ivmapmsgstcell"
class drivercontrol {
public:
  drivercontrol(ros::NodeHandle nh);
  ~drivercontrol();
  void Run();

private:
  double PidSpeed(const double targetSpeed, const double lagTargetSpeed, const double currentSpeed,double dt);
  sPointMsg FollowPoints(std::vector<sPointMsg> &points);
  sPointMsg GetKeyPoint(const std::vector<sPointMsg> &points);
  bool IsClose(const double num1, const double num2, const double factor);
  double CalcDistance(const sPointMsg &point1, const sPointMsg &point2);
  double DrControl(double virspeed,double dt);
  int ActuatorControlPid(double accel_command, double dt);
  void SbjspeedFilter();
  void PubMessageIvdrivercontrol();
  double TestVirtualSpeed();
  void cout_test();
  void SubCallbackActuator(const ivactuator::ivactuator::ConstPtr &msg);
  void SubCallbackMap(const ivmap::ivmapmsgvap::ConstPtr &msg);
  void SubCallbackMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg);
  void SpeedPlan(float start_point,float start_speed,float acc,float end_speed,float stable_speed_dt,float *end_speed_t,float *end_t);
  double SignalLag(uint8_t lag_time_s, double *cache_for_lag, double new_signal);
  double GetSbjAcc();
  void chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg);//
  void chatterCallbackObjstcell(const ivmap::ivmapmsgstcell::ConstPtr &msg);//
  void Init(ros::NodeHandle nh);
  void InitParam();
  void InitFilter(); 
  void InitLaunchfile(ros::NodeHandle nh);
  void InitPid();
  void UpdateVehicleState(VehicleState_s *vehicle_state);
  void FileOperate(ofstream *dirver_file, Log_s *log_struct, int length);
  void LogWrite(ofstream *dirver_file);
  double GetDt();
  double AccToTorqueNew(double acc_desired,double speed_now);
  int generate_objROI();//
  double generate_objROI1();
  double generate_objROI2();
  ros::Subscriber sub_motionplanner;
  ros::Subscriber sub_actuator;
  ros::Subscriber sub_map;
  ivmap::ivmapmsgobj ivobj;//
  ivmap::ivmapmsgstcell ivstobj;//
 
  int loop_freq;
  double dt;
  double sbjspeed;
  double sbjspeed_last;
  double sbjspeed_last_for_filter;
  double torque_temp_last;
  unsigned char speedlost_cnt;
  bool is_backsliding;
  bool is_backsliding_4_obstacle;
  double virtualspeed;
  int emergency_flag;
  int shiftposition;
  int shiftpositionlast;
  double kps_max;
  double kps;
  double kis;
  double ci_velocity_lower;
  double ci_velocity_upper_normal;
  double ci_velocity_upper_max;
  double ci_velocity_upper;
  double acc_kp_torque;
  double acc_ki_torque;
  double acc_kp_press;
  double acc_ki_press;

  double acel_max;
  double acel_min;

  bool is_start_excitation;
  bool is_test_virtualspeed;
  bool is_pos_control;
  double cache_for_lag[80];
  double cache_for_lag2[80];
  double min_driver_torque;
  uint8_t lag_time_s;
  double max_speed_error_p;
  double max_speed_error_n;

  double min;
  int emergency_f;
  double min_1;
  double min_2;

  ros::Time start_time;
  ros::Time start_last_time;
  lpf_1p_param param;
  lpf_1p_param param_temp;
  inte_param speed_integrator_param;
  VehicleState_s vehicle_state;
  PID_Typedef speed_control_pid;
  PID_Typedef acc_control_pid;
  sPointMsg key_point;
  std::vector<sPointMsg> path;
  ivdrivercontrol::ivdrivercontrol msg_control;
  ivdrivercontrol::ivdriverdebug msg_debug;
  ivdrivercontrol::ivdrivercontrolstates msg_state_debug;
  ofstream dirver_file;
  Log_s log_struct[];
  ros::Publisher pub_;
  ros::Publisher pub_debug_state;
  ros::Publisher pub_debug;
  ros::Subscriber sub_Obj;
  ros::Subscriber sub_Stcell;
};

#endif  // _DRIVERCONTROL_H
