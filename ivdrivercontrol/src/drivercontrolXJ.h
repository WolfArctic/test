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
#ifndef _DRIVERCONTROLXJ_H
#define _DRIVERCONTROLXJ_H
#pragma once
using namespace std;
#include "ivdrivercontrol.h"
#include "lib/filter.h"
#include "lib/pid.h"
#include "lib/log.h"
#include "lib/mathLib.h"
#include "vehicle_state.h"
#include "fstream"
typedef struct 
{
  double last_value;
  double dt;
}SignalDiff;

class drivercontrolXJ {
public:
  drivercontrolXJ(ros::NodeHandle nh);
  ~drivercontrolXJ();
  void Run();

private:
  double PidSpeed(const double targetSpeed, const double lagTargetSpeed, const double currentSpeed,double dt);
  sPointMsg FollowPoints(std::vector<sPointMsg> &points);
  sPointMsg GetKeyPoint(const std::vector<sPointMsg> &points);
  bool IsClose(const double num1, const double num2, const double factor);
  double CalcDistance(const sPointMsg &point1, const sPointMsg &point2);
  double DrControl(double virspeed,double dt);
  int ActuatorControlPid(double accel_command, double dt);
  void PubMessageIvdrivercontrol();
  double TestVirtualSpeed();
  void cout_test();
  void SubCallbackMap(const ivmap::ivmapmsgvap::ConstPtr &msg);
  void SubCallbackMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg);
  void SubCallbackActuator(const ivactuator::ivactuator::ConstPtr &msg);
  void SpeedPlan(float start_point,float start_speed,float acc,float end_speed,float stable_speed_dt,float *end_speed_t,float *end_t);
  double SignalLag(uint8_t lag_time_s, double *cache_for_lag, double new_signal);
  double GetSbjAcc();
  void Init(ros::NodeHandle nh);
  void InitParam();
  void InitFilter(); 
  void InitLaunchfile(ros::NodeHandle nh);
  void InitPid();
  void UpdateVehicleState(VehicleState_s *vehicle_state);
  void UpdateMessageIvdriverdebug();
  void FileOperate(ofstream *dirver_file, Log_s *log_struct, int length);
  void LogWrite(ofstream *dirver_file);
  double GetDt();
  void SignalDdiffInit(SignalDiff *signal_diff_s, double dt);
  double GetSignalDiff(SignalDiff *signal_diff_s,double signal);
  bool IsStartAutoDriving();
  double FFDRControl(double target_acc);
  
  double BaseTorque(double target_speed);
  double FFBRControl(double target_acc);
  double LookUpTable(const std::map<double, double> &table, const double x);
  double LookUp2Table(const std::map<map<double,double>, double> &table, const pair<double,double> x);
  double AutoTestGernaralTorque();
  double AccToTorque(double target_acc, double speed_s);
  double AccpointToTorque(int acc_ind,int speed_ind, double torque[]);
  LookUp LookSection(std::vector<double> speed_v,std::vector< vector<double> > acc_v,pair<double,double> x);
  ros::Subscriber sub_motionplanner;
  ros::Subscriber sub_actuator;
  ros::Subscriber sub_map;

  int loop_freq;
  double dt;
  double sbjspeed;
  double sbjspeed_last;
  double sbjaccel;
  DrivingMode_s driver_flag;
  DrivingMode_s driver_flag_last;
  double virtualspeed;
  double acel_desired;
  int emergency_flag;
  uint8_t currentdrvmode;

  double kps_max;
  double kps;
  double kis;
  double k2t;
  double k2p;
  double ci_velocity_upper;
  double ci_velocity_lower;
  double acc_kp_torque;
  double acc_ki_torque;
  double acc_kp_press;
  double acc_ki_press;

  double acel_max;
  double acel_min;
  double acel_i;

  bool torque_temp_flag;
  double pressure_temp_last;
  double torque_max; 
  double torque_min; 
  int zoom_factor;
  double pressure_max; 
  double pressure_min; 
  double modeSwitchLower;
  double modeSwitchUpper;
  double max_speed_error_p;
  double max_speed_error_n;
  bool is_start_excitation;
  bool is_test_virtualspeed;
  bool is_pos_control;
  bool is_use_speed_ff;
  uint8_t lag_time_s;
  double cache_for_lag[80];
  double cache_for_lag2[80];
  // ros::Time start_time;
  // ros::Time start_last_time;
  lpf_1p_param param;
  lpf_1p_param param_temp;
  lpf_1p_param param_temp2;
  mean_filter_param sbjspeed_filter_param;
  inte_param speed_integrator_param;
  VehicleState_s vehicle_state;
  PID_Typedef speed_control_pid;
  PID_Typedef acc_control_pid;
  PID_Typedef press_control_pid;
  sPointMsg key_point;
  SignalDiff des_speed_diff_s;
  std::vector<sPointMsg> path;
  ivdrivercontrol::ivdrivercontrol msg_control;
  ivdrivercontrol::ivdriverdebug msg_debug;
  ivdrivercontrol::ivdrivercontrolstates msg_state_debug;
  ofstream dirver_file;
  //Log_s log_struct[];
  ros::Publisher pub_;
  ros::Publisher pub_debug_state;
  ros::Publisher pub_debug;
};

#endif  // _DRIVERCONTROL_H
