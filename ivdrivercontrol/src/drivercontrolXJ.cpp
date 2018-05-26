#include "drivercontrolXJ.h"

drivercontrolXJ::drivercontrolXJ(ros::NodeHandle nh) 
{
  Init(nh);
  dirver_file.open("home/home/SeqDataFromDriver.txt");
  
  
}  

drivercontrolXJ::~drivercontrolXJ() 
{
 dirver_file.close();
}

void drivercontrolXJ::Init(ros::NodeHandle nh) 
{
  InitParam();
  InitLaunchfile(nh);
  InitFilter();
  InitPid();
}
void drivercontrolXJ::InitParam() 
{
  // variable initialize
  loop_freq = 20;
  dt = 0.05;
  sbjspeed = 0;
  sbjspeed_last = 0;
  sbjaccel = 0;
  driver_flag = DRIVERING;
  driver_flag_last = DRIVERING;
  virtualspeed = 0;
  emergency_flag = 0;
  currentdrvmode = 1;
  vehicle_state = STOP;
  lag_time_s =1.0;
  max_speed_error_p = 0.5;
  max_speed_error_n = -10;
  torque_temp_flag = false;
  pressure_temp_last = 12;
  zoom_factor = 1;
  memset(cache_for_lag, 0, sizeof(cache_for_lag));
  memset(cache_for_lag2, 0, sizeof(cache_for_lag));
  SignalDdiffInit(&des_speed_diff_s, dt);
}
void drivercontrolXJ::InitFilter() 
{
  double acc_cutoff = 0.1;//hz
  lpf_1p_set_cutoff_freq(&param, dt, acc_cutoff);
  double window_size = 5;
  mean_filter_init(&sbjspeed_filter_param,window_size); 
  double fcut_temp = 0.05;
  lpf_1p_set_cutoff_freq(&param_temp, dt,fcut_temp);
  lpf_1p_set_cutoff_freq(&param_temp2, dt,fcut_temp);
}
void drivercontrolXJ::InitLaunchfile(ros::NodeHandle nh)
{
  // PID_speed param, for common road
  nh.param("kps", kps, kps);
  nh.param("kis",kis, kis);
  

  nh.param("ciVelocityUpper", ci_velocity_upper, ci_velocity_upper);
  nh.param("ciVelocityLower", ci_velocity_lower, ci_velocity_lower);
  nh.param("acel_max", acel_max, acel_max);
  nh.param("acel_min", acel_min, acel_min);

  nh.param("acc_kp_torque", acc_kp_torque, acc_kp_torque);
  nh.param("acc_ki_torque", acc_ki_torque, acc_ki_torque);
  nh.param("acc_kp_press", acc_kp_press, acc_kp_press);
  nh.param("acc_ki_press", acc_ki_press, acc_ki_press);
  nh.param("k2t",k2t,k2t);
  nh.param("k2p",k2p,k2p);
  nh.param("torque_max",torque_max,torque_max);
  nh.param("torque_min",torque_min,torque_min);
  nh.param("pressure_max",pressure_max,pressure_max);
  nh.param("pressure_min",pressure_min,pressure_min);
  nh.param("zoom_factor",zoom_factor,zoom_factor);
  nh.param("modeSwitchLower",modeSwitchLower,modeSwitchLower);
  nh.param("modeSwitchUpper",modeSwitchUpper,modeSwitchUpper);

  nh.param("is_test_virtualspeed", is_test_virtualspeed, is_test_virtualspeed);
  nh.param("is_start_excitation", is_start_excitation, is_start_excitation);
  nh.param("is_pos_control", is_pos_control, is_pos_control);
  nh.param("is_use_speed_ff", is_use_speed_ff, is_use_speed_ff);

  // receive data from topic ivmap
  sub_map = nh.subscribe("ivmapvap", 1000, &drivercontrolXJ::SubCallbackMap, this);

  // receive data from topic ivmsgpath
  sub_motionplanner = nh.subscribe("ivmotionplanner", 1000, &drivercontrolXJ::SubCallbackMotionplanner, this);
  sub_actuator = nh.subscribe("ivactuator",1000,  &drivercontrolXJ::SubCallbackActuator, this);
  // publish data through topic ividrivercontrolXJ
  pub_ = nh.advertise<ivdrivercontrol::ivdrivercontrol>("ivdrivercontrol", 1000);

  // publish debug data through topic ividriverdebug
  pub_debug = nh.advertise<ivdrivercontrol::ivdriverdebug>("ivdriverdebug", 1000);
  pub_debug_state = nh.advertise<ivdrivercontrol::ivdrivercontrolstates>("ivdrivercontrolstates", 1000);
}
void drivercontrolXJ::InitPid()
{
  inte_param_init(&speed_integrator_param, dt);
  speed_control_pid.kp = kps;
  speed_control_pid.ki = kis;
  speed_control_pid.integ = 0;
  speed_control_pid.integ_max = ci_velocity_upper;
  speed_control_pid.integ_min = ci_velocity_lower;
  acc_control_pid.kp = acc_kp_torque;
  acc_control_pid.ki = acc_ki_torque;
  acc_control_pid.integ = 0;
  acc_control_pid.integ_max = 100;

  press_control_pid.kp = acc_kp_press;
  press_control_pid.ki = acc_ki_press;
  press_control_pid.integ = 0;
  press_control_pid.integ_max = 100;
}
void drivercontrolXJ::Run() 
{
  ros::Rate rate(loop_freq);
  while (ros::ok())
  {
    ros::spinOnce();
    memset(&msg_control, 0, sizeof(msg_control));
    memset(&msg_debug, 0, sizeof(msg_debug));
    memset(&msg_state_debug,0,sizeof(msg_state_debug));
    //dt = GetDt();
    if(IsStartAutoDriving())//now is auto driving mode?
    {
      key_point = FollowPoints(path);
      virtualspeed = key_point.velocity;
   // virtualspeed = lpf_1p_apply(&param_temp, virtualspeed);
      cout<<"is_test_virtualspeed1"<<is_test_virtualspeed<<endl;
      if(is_test_virtualspeed)
      {
        cout<<"is_test_virtualspeed"<<is_test_virtualspeed<<endl;
        virtualspeed = TestVirtualSpeed();
      }  
      acel_desired = DrControl(virtualspeed, dt);
     // acel_desired = constrain_float(3,-0.35,acel_desired);
      ActuatorControlPid(acel_desired, dt);
    }
    else
    {
      set_integrator(&speed_control_pid, 0);
    }
    UpdateMessageIvdriverdebug();
    UpdateVehicleState(&vehicle_state);
    PubMessageIvdrivercontrol();
    cout_test();
    LogWrite(&dirver_file);
    sbjspeed_last = sbjspeed;
    rate.sleep();

  }
}
bool drivercontrolXJ::IsStartAutoDriving()
{
  //if(currentdrvmode == 0)
    return true;
   // else
   //  return false;
}
double drivercontrolXJ::GetDt()
{
  double dt_temp;
  ros::Time start_time = ros::Time::now();
  static ros::Time start_last_time  = ros::Time::now();
  dt_temp = (start_time - start_last_time).toSec();
  start_last_time = start_time;
  return dt_temp; 
}
void drivercontrolXJ::cout_test()
{

  //cout<<"is_test_virtualspeed "<<is_test_virtualspeed<<endl;
  // cout<<"acc_kp_torque "<<acc_kp_torque<<endl;
  // cout<<"acc_ki_torque "<<acc_ki_torque<<endl;
  // cout<<"is_pos_control"<<is_pos_control<<endl;
  // cout<<"is_test_virtualspeed"<<is_test_virtualspeed<<endl;

}
void  drivercontrolXJ::LogWrite(ofstream *dirver_file )
{
  if (!dirver_file->is_open())
  {
   // cerr<<"Oop!!!"<<endl;
    return;
  }
  int length;
  Log_s log_struct[] = 
  {
    {"dt",(double)dt},
    {"sbjspeed", (double)sbjspeed},
    {"sbjaccel",(double)sbjaccel},
    {"virtualspeed", (double)virtualspeed},
    {"acel_desired",(double)acel_desired},
    {"torque_temp",(double)msg_control.targettorque},
    {"pressure",(double)msg_control.targetacc},
    {"drivermode",(double)msg_control.actuatormode},
    {"speed_control_p",(double)msg_debug.speed_p},
    {"speed_control_i",(double)msg_debug.speed_i},
    {"acel_control_p",(double)msg_debug.acel_p},
    {"acel_control_i",(double)msg_debug.acel_i},
    {"acel_max", acel_max}
  };
  length = sizeof(log_struct)/24;
  FileOperate(dirver_file, log_struct, length);
}

void drivercontrolXJ::FileOperate(ofstream *dirver_file, Log_s *log_struct, int length)
{
  static bool WriteVariableName = true;
  static string spaces = "     ";

  if(WriteVariableName)
  {
    WriteVariableName = false;
   
    for(int i = 0; i<length; i++)
    {
      *dirver_file<<log_struct[i].m_name<<spaces;
    }
    *dirver_file<<endl;
  }
  for(int i = 0;i<length; i++)
  {
    *dirver_file<<(log_struct[i].m_value)<<spaces;
  }
  *dirver_file<<endl;
}

void drivercontrolXJ::SubCallbackMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg) 
{
  sPointMsg pointMsg;
  sPointMsg pointMsg_last;
  memset(&pointMsg, 0, sizeof(pointMsg));
  memset(&pointMsg_last, 0, sizeof(pointMsg_last));
  DEBUG_LOG(to_string(msg->points.size()));
  path.clear();

  for (int i = 0; i < msg->points.size(); ++i) 
  {
    pointMsg.x = msg->points[i].x;
    pointMsg.y = msg->points[i].y;
    pointMsg.length += CalcDistance(pointMsg_last, pointMsg);
    pointMsg.velocity = msg->points[i].velocity;
    emergency_flag = msg->points[i].emergency_flag;

    pointMsg_last = pointMsg;
    path.push_back(pointMsg);

    if (i < 40) 
    {
        DEBUG_LOG(to_string(i) + ":pointMsg.x:" + to_string(pointMsg.x) + ":.y:" + to_string(pointMsg.y) + ":.length:" + to_string(pointMsg.length) + ":.velocity:" + to_string(pointMsg.velocity));
    }
  }

  if (path.empty()) 
  {
    DEBUG_LOG("Way point is lost, give it a stop point for safe!");
    sPointMsg pointMsg;
    // memset(&pointMsg, 0, sizeof(pointMsg));
    pointMsg.x = 0; // 0 may cause SafeControl
    pointMsg.y = 0;
    pointMsg.length = 0;
    pointMsg.velocity = 0;
    path.push_back(pointMsg);
    path.push_back(pointMsg);
  }
}

void drivercontrolXJ::SubCallbackMap(const ivmap::ivmapmsgvap::ConstPtr &msg) 
{
  sbjspeed = (double)msg->v;
  sbjspeed = mean_filter_apply(&sbjspeed_filter_param, sbjspeed);
}

void drivercontrolXJ::SubCallbackActuator(const ivactuator::ivactuator::ConstPtr &msg)
{
  currentdrvmode = msg->currentdrvmode;
}

// check if num1 is similar to num2
bool drivercontrolXJ::IsClose(const double num1, const double num2, const double factor)
{
  return ((num2 - num1) <= factor && (num2 - num1) >= -factor);
}


// calculate distance between 2 points
double drivercontrolXJ::CalcDistance(const sPointMsg &point1, const sPointMsg &point2) 
{
  return (sqrt((point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) * (point2.y - point1.y)));
}


sPointMsg drivercontrolXJ::FollowPoints(std::vector<sPointMsg> &points) 
{
  // if path is null, give it a .velocity=0 point, to brake for safe
  if (points.empty()) 
  {
    DEBUG_LOG("Way point is lost, give it a stop point for safe!");
    sPointMsg pointMsg;
    // memset(&pointMsg, 0, sizeof(pointMsg));
    pointMsg.x = 0; // 0 may cause SafeControl
    pointMsg.y = 0;
    pointMsg.length = 0;
    pointMsg.velocity = 0;
    points.push_back(pointMsg);
    // insert 2 points, for p[0] is ignored
    points.push_back(pointMsg);
    return pointMsg;
  }

  return GetKeyPoint(points);
}


sPointMsg drivercontrolXJ::GetKeyPoint(const std::vector<sPointMsg> &points) 
{
  
    int start_index = 0;
   for (int i = 0; i < points.size(); i++) 
   {
      if (points[i].x > 0) 
      {
          start_index = i;
          break;
      }
   }
  // cout<<"start_index"<<start_index<<endl;
   int preview_index = 0;
   //msg_debug.e_distance = points[start_index].velocity;
   const double y_offset = fabs(points[start_index].y);
   double relspeed_target = (points[start_index].velocity - sbjspeed);
  if(driver_flag == BRAKING)
  {
    relspeed_target = -relspeed_target;
  }
   double k2 = 0.5;
   double distance = 1.0;//2.2
   double ratio_adjust = 0.2;
   static double distance_adjust = 0;
   //double preview_distance = k1 * sbjspeed;
  // double preview_distance = 0.15*sbjspeed*sbjspeed + 0.3*sbjspeed;
   double preview_distance = 0.5*GetSbjAcc()+ sbjspeed*0.8;//0.4//0.6 ok
   //distance_adjust = MIN(k2 * relspeed_target, ratio_adjust * preview_distance);
   distance_adjust = distance_adjust + k2 * relspeed_target;
   distance_adjust = constrain_float(5,0,distance_adjust);
   distance_adjust = lpf_1p_apply(&param_temp, distance_adjust);
   // cout<<"relspeed_target"<<relspeed_target<<endl;
   // cout<<"distance_adjust"<<distance_adjust<<endl;
   msg_debug.e_distance = distance_adjust;
   preview_distance +=  distance_adjust + distance; 
   msg_debug.distance = preview_distance;
  if (points.size() > start_index) 
  {
    for (size_t i = start_index; i < points.size(); ++i) 
    {
      if (points[i].length - points[start_index].length >= preview_distance || points[i].velocity <= 0.01) 
      {
        preview_index = i;
        break;
      }
    }
  }
  msg_debug.thwi =points[start_index].velocity;

  return points[preview_index];
}
double drivercontrolXJ::GetSbjAcc()
{
    double sbjaccel = (sbjspeed - sbjspeed_last)/dt;
    return lpf_1p_apply(&param, sbjaccel);
}

double drivercontrolXJ::PidSpeed(const double targetSpeed, const double lagTargetSpeed, const double currentSpeed, double dt) 
{
  double acel_speed = 0;
  // PID dead zone
  double relspeed = targetSpeed - currentSpeed;
  double relspeedLag = lagTargetSpeed - currentSpeed;
  msg_debug.ttci = lagTargetSpeed;
  
  // P element
  double p;
  double i;
  p = pid_get_p(&speed_control_pid,relspeed);
  // I element
  if(torque_temp_flag == true)
    i = pid_get_i(&speed_control_pid,0, dt);
  else
    i = pid_get_i(&speed_control_pid,relspeedLag, dt);
 
  //if(fabs(sbjspeed)- 0.1<0 || driver_flag != driver_flag_last)
  if(fabs(sbjspeed)- 0.1<0)
  {
    i = 0;
    set_integrator(&speed_control_pid, 0);
  }  
  msg_debug.speed_p = p;
  msg_debug.speed_i = i;

  acel_speed = p + i;
  return acel_speed;
}



double drivercontrolXJ::DrControl(double virspeed, double dt) 
{
  DEBUG_LOG("Entrance.");

  double relspeed_temp = virspeed - sbjspeed;
  relspeed_temp = constrain_float(max_speed_error_p,max_speed_error_n,relspeed_temp);
  double 	virspeedLimited = relspeed_temp + sbjspeed;
  if(driver_flag == DRIVERING)
    lag_time_s = 1.0;
  else
    lag_time_s = 1.5;
  double virspeedLag = SignalLag(lag_time_s,cache_for_lag,virspeedLimited);
  double acel_des_ff = GetSignalDiff(&des_speed_diff_s,virspeedLimited);
  double acel_temp  = PidSpeed(virspeed, virspeedLag, sbjspeed, dt);
  if(is_use_speed_ff)
    acel_temp = acel_temp + acel_des_ff;
  // accel output limit
   acel_temp = MAX(acel_temp, acel_min);
   acel_temp = MIN(acel_temp, acel_max);

  // BELOW for debug

  return acel_temp;
}
void drivercontrolXJ::SignalDdiffInit(SignalDiff *signal_diff_s, double dt)
{
  signal_diff_s->last_value = 0;
  signal_diff_s->dt = dt;
}
double drivercontrolXJ::GetSignalDiff(SignalDiff *signal_diff_s,double signal)
{
  double diff_temp = (signal - signal_diff_s->last_value)/signal_diff_s->dt;
  signal_diff_s->last_value = signal;
  return diff_temp;
}
double drivercontrolXJ::TestVirtualSpeed()
{
  static int test_counter = 0;
  static bool is_test_counter_increase = true;

  static float start_t[10] = {0};
  static float start_speed[10] = {0};

  start_t[0] = 20;
  start_speed[0] = 0;
  static float end_speed[] = {4,8, 10,8,  6, 3, 0};
  static float acc[] =       {1,2,0.5,-1,-0.5,-2,-1};
  static float stable_speed_dt[] ={20,20,20,20,20,20,20}; 

  int max_a = sizeof(acc)/sizeof(float);
  static float time_point[30] = {0};
  static float dt = 1.0/(double)loop_freq;


  static int n = 0;
  static int s = 0;
  static int a = 0;
  float end_speed_t;
  float end_t;
  int j= 0;
  static double virtualspeed_temp;
  for (int i = 0;i<max_a;i++)
  {  
    SpeedPlan(start_t[i],start_speed[i],acc[i],end_speed[i],stable_speed_dt[i],&end_speed_t,&end_t);
    start_t[i+1]= end_t;
    start_speed[i+1] = end_speed[i];
    time_point[j++] =  start_t[i];
    time_point[j++] = end_speed_t;
  }
  time_point[j++] = end_t;
  float t = test_counter*dt;
  while(a<max_a)
  {
    if(t<time_point[n])
    {
      virtualspeed_temp = start_speed[s];
      break;
    }     
    else if(t<time_point[n+1])
    {
      virtualspeed_temp = start_speed[s] + acc[a]*(t-time_point[n]);
      break;
    }    
    else if(t<time_point[n+2])
    {
      virtualspeed_temp = start_speed[s+1];
      break;
    }
    else
    {
      n = n+2;
      s = s+1;
      a = a+1;
    }
  }
  
  if (is_test_counter_increase) 
  {
    ++test_counter;
  }
  else 
  {
    --test_counter;
  }
  // if (2600 == test_counter || 0 == test_counter) 
  // {
  //   is_test_counter_increase = !is_test_counter_increase;
  // }
        
    return virtualspeed_temp;
}
void drivercontrolXJ::SpeedPlan(float start_point,float start_speed,float acc,float end_speed,float stable_speed_dt,float *end_speed_t,float *end_t)
{
  (*end_speed_t) = start_point + fabs((end_speed - start_speed)/acc);
  (*end_t) = (*end_speed_t) + stable_speed_dt;
}
double drivercontrolXJ::SignalLag(uint8_t lag_time_s, double *cache_for_lag, double new_signal)
{
 uint8_t lag_tick = lag_time_s / dt;
  for(int i=0;i<lag_tick-1;i++)
  {
    cache_for_lag[i] = cache_for_lag[i+1];
  }
  cache_for_lag[lag_tick-1] = new_signal;
  return  cache_for_lag[0];                  

}
double drivercontrolXJ::AutoTestGernaralTorque()
{
  //torque to speed
  double dt = 0.5;
  double time = 20;
  static int c = 0;
  while(c<time/dt)
  {
    c++;
  }
  static ros::Time test_time_start = ros::Time::now();
  ros::Time test_time_end = ros::Time::now();
  static double torque_temp_test = 0;
  static int cnt = 1;
  static bool increas_flag = true;
  double det_time = 10;
  double det_torque = 0.1;
  double start_time = 20;
 // while()
    if((test_time_end - test_time_start).toSec()>cnt*det_time)
     {
      if(torque_temp_test<torque_max  && increas_flag == true)
         torque_temp_test = torque_temp_test+det_torque*torque_max;
      if(torque_temp_test>=torque_max)
       {
        torque_temp_test = torque_max;
        increas_flag = false;
       } 
       if(torque_temp_test>torque_min  && increas_flag == false)
       {
        torque_temp_test = torque_temp_test-det_torque*torque_max;
       }
       if(torque_temp_test<=torque_min)
       {
        torque_temp_test = torque_min;
        increas_flag = true;
       }
      
      cnt++;
     }
     return torque_temp_test;
}

int drivercontrolXJ::ActuatorControlPid(double accel_command,double dt) 
{
  DEBUG_LOG("Entrance.");
  int errCode = 0;
  double p;
  double i;
  double e_acc;
  sbjaccel = GetSbjAcc();
  double torque_temp = 0;
  double pressure = 0;
  double torque = 0;
  e_acc = (accel_command - sbjaccel);
  //e_acc = constrain_float(7,-20,e_acc);
  double accel_command_lag = SignalLag(lag_time_s,cache_for_lag2,accel_command);
  double  e_acc_lag = accel_command_lag - sbjaccel;
  torque_temp = AccToTorque(accel_command, sbjspeed);

  if(torque_temp >= modeSwitchUpper )
  {
    driver_flag = DRIVERING;  
    p = pid_get_p(&acc_control_pid,e_acc_lag);
    i = pid_get_i(&acc_control_pid,e_acc_lag,dt);
    msg_debug.acel_p = p;
    msg_debug.acel_i = i;
   // torque_temp = FFDRControl(accel_command)+BaseTorque(virtualspeed)+p+i;
    torque = torque_temp;
  } 
  else if (torque_temp >= modeSwitchLower) 
  {
    if(DRIVERING == driver_flag_last) 
    {
      driver_flag = DRIVERING;  
      p = pid_get_p(&acc_control_pid,e_acc);
      i = pid_get_i(&acc_control_pid,e_acc_lag,dt);
      msg_debug.acel_p = p;
      msg_debug.acel_i = i;
      torque = torque_temp;
    }
    else if(BRAKING == driver_flag_last)
    {
      driver_flag = BRAKING;
      p = pid_get_p(&press_control_pid,e_acc);
      i = pid_get_i(&press_control_pid,e_acc_lag,dt);
      msg_debug.acel_p = p;
      msg_debug.acel_i = i;
      pressure = -torque_temp;//+BaseTorque(virtualspeed)-(p + i);
      torque = 0;
    }
  }
  else
  {
    driver_flag = BRAKING;
    p = pid_get_p(&press_control_pid,e_acc);
    i = pid_get_i(&press_control_pid,e_acc_lag,dt);
    msg_debug.acel_p = p;
    msg_debug.acel_i = i;
    pressure = -torque_temp;//+BaseTorque(virtualspeed)-(p + i);
    torque = 0;
  }

  torque = MIN(torque, torque_max);
  torque = MAX(torque, torque_min);
  pressure = MIN(pressure, pressure_max);
  pressure = MAX(pressure, pressure_min);
if(torque_temp >=torque_max)
  torque_temp_flag = true;
else
  torque_temp_flag = false;
  if (is_start_excitation)
  {
     torque = 0;
     pressure = 60;
     driver_flag = BRAKING;
   //torque_temp = AutoTestGernaralTorque();
   //pressure_temp = 0;
   //driver_flag = DRIVERING;
  }
  else if (fabs(virtualspeed) <= 0.1 && fabs(sbjspeed) <= 1.8)//1
  {
     torque_temp = torque_min;
     driver_flag = BRAKING;
     //pressure_temp = pressure_temp_last;
     pressure = 0.6*pressure_temp_last+0.4*fabs(12*acel_desired)+5;
  }

 // if (emergency_flag==1 || currentdrvmode == 1 || currentdrvmode ==2)
 if (emergency_flag==1) 
  {
      torque_temp = torque_min;  
      driver_flag = DRIVERING;
      set_integrator(&speed_control_pid, 0);
  }
  driver_flag_last = driver_flag;
  pressure_temp_last = pressure;
  msg_debug.THW_set = torque/300;
  msg_control.targettorque = torque;
  msg_control.targetacc = pressure;
  msg_control.shiftposition = 3;
  if(driver_flag == DRIVERING)
    msg_control.actuatormode = 1;
  else
    msg_control.actuatormode = 0;

  return errCode;
}
double drivercontrolXJ::FFDRControl(double target_acc)
{
  
   return 0;

}
double drivercontrolXJ::FFBRControl(double target_acc)
{
  
   return 0;

}
double drivercontrolXJ::BaseTorque(double target_speed)
{
  
   return 0;

}
double drivercontrolXJ::LookUpTable(const std::map<double, double> &table, const double x) 
{
 // std::map is auto sorted
 // DEBUG_LOG("Entrance.");
  const auto little_iter = table.cbegin();
  const auto big_iter = table.crbegin();

  if(x <= little_iter->first) 
  {
    //DEBUG_LOG(to_string(x) +" is too small.");
    return little_iter->second;
  }
  if(x >= big_iter->first) 
  {
    // DEBUG_LOG(to_string(x) +" is too big.");
    return big_iter->second;
  }

  const auto next = table.lower_bound(x);
  // to float type, 2.0000000000000000000001 might be equal to 2
  // in this case, --temp may fail
  if(next == little_iter) 
  {
   // DEBUG_LOG(to_string(x) +" is the small endpoint.");
    return little_iter->second;
  }
  auto temp = next;
  const auto previous = (--temp);
  return (next->second - previous->second) / (next->first - previous->first) * (x - previous->first) + previous->second;
}
double drivercontrolXJ::LookUp2Table(const std::map<map<double,double>, double> &table, const pair<double,double> x) 
{

  return 0;
}
LookUp drivercontrolXJ::LookSection(std::vector<double> speed_v,std::vector< vector<double> > acc_v,pair<double,double> x)
{
    vector<double>::size_type speed_s = 0;
    vector<double>::size_type acc_s = 0;
    pair<double,double> speed_section;
    pair<double,double> acc_section1;
    pair<double,double> acc_section2;
    map<double,double> section;
    LookUp out;
    pair<double,double> x_temp = x;
    out.is_lookup = false;
    out.acc_low_limit_low = false;
    out.acc_up_limit_low = false;
    out.acc_low_limit_high = false;
    out.acc_up_limit_high = false;
    out.speed_low_limit = false;
    out.speed_up_limit = false;
    if(x_temp.first<speed_v[0])
    {
        out.is_lookup = false;
        out.speed_low_limit = true;
        out.speed_up_limit = false;
        //out.section.clear();
        speed_s = 0;
        speed_section.first = speed_s;
        speed_section.second = speed_s+1;
        x.first = speed_v[0];
    }
    else if(x_temp.first>speed_v[speed_v.size()-1])
    {
        out.is_lookup = false;
        out.speed_up_limit = true;
        out.speed_low_limit = false;
       // out.section.clear();
        speed_s = speed_v.size()-2;
        speed_section.first = speed_s;
        speed_section.second = speed_s+1;
        x.first = speed_v[speed_v.size()-1];
    }
    //else
        while(speed_s<speed_v.size())
        {
            if(x.first>=speed_v[speed_s]&& x.first<=speed_v[speed_s+1])
            {
                speed_section.first = speed_s;
                speed_section.second = speed_s+1;
                break;
            }
            speed_s++;

        }
     if(x_temp.second<acc_v[speed_section.first][0])
    {
        out.is_lookup = false;
        out.acc_low_limit_low = true;
        out.acc_up_limit_low = false;
        out.acc_low_limit_high = false;
        out.acc_up_limit_high = false;
        x.second = acc_v[speed_section.first][0];
       // out.section.clear();
    }
    else if(x_temp.second>acc_v[speed_section.first][acc_v[speed_section.first].size()-1])
    {
        out.is_lookup = false;
        out.acc_low_limit_low = false;
        out.acc_up_limit_low = true;
        out.acc_low_limit_high = false;
        out.acc_up_limit_high = false;
        x.second = acc_v[speed_section.first][acc_v[speed_section.first].size()-1];
       // out.section.clear();
    }
     while(acc_s<acc_v[speed_section.first].size())
    {
        if(x.second>=acc_v[speed_section.first][acc_s]&& x.second<=acc_v[speed_section.first][acc_s+1])
        {
            acc_section1.first = acc_s;
            acc_section1.second = acc_s+1;
            break;
        }
        acc_s++;
    }
        acc_s = 0;
    if(x_temp.second<acc_v[speed_section.second][0])
    {
        out.is_lookup = false;
        out.acc_low_limit_low = false;
        out.acc_up_limit_low = false;
        out.acc_low_limit_high = true;
        out.acc_up_limit_high = false;
        x.second = acc_v[speed_section.second][0];
       // out.section.clear();
    }
    else if(x.second>acc_v[speed_section.second][acc_v[speed_section.second].size()-1])
    {
        out.is_lookup = false;
        out.acc_low_limit_low = false;
        out.acc_up_limit_low = false;
        out.acc_low_limit_high = false;
        out.acc_up_limit_high = true;
        x.second = acc_v[speed_section.second][acc_v[speed_section.second].size()-1];
    }        
        while(acc_s<acc_v[speed_section.second].size())
        {
            if(x.second>=acc_v[speed_section.second][acc_s]&& x.second<=acc_v[speed_section.second][acc_s+1])
            {
                acc_section2.first = acc_s;
                acc_section2.second = acc_s+1;
                break;
            }
            acc_s++;
        }
    out.speed_sec = speed_section;
    out.acc_sec_low = acc_section1;
    out.acc_sec_high = acc_section2;
    out.is_lookup = true;
   // out.section = section;
    return out;
}
double drivercontrolXJ::AccToTorque(double target_acc, double speed_s)
{
    pair<double,double> target_x;
    target_x.first = speed_s;//speed;
    target_x.second = target_acc;//acc
    #define SPEED_S 9
    #define TORQUE 21
    double torque[] = {-100,0,2.5, 5.0, 7.5,10.0, 12.5,15.0,17.5,20.0,25.0,30.0,35.0,40.0,45.0,50.0,55.0,60.0,65.0,70.0,75.0};
    double acc[SPEED_S][TORQUE] = 
           {{-5,  0.0,  0.1,  0.6,    0.6,   0.6,  0.6,  0.6,  0.7,  0.8,  1.3, 1.8, 2.4,  2.5,  2.7,  2.9,  3.1,  3.2,  3.4, 3.4,3.5},\
            {-5,  0.0, 0.01,  0.02,  0.03,  0.04, 0.25,  0.3,  0.4,  0.7,  1.2, 1.7, 2.2,  2.2,  2.5,  2.7,  3.0,  3.0,  3.2, 3.3,3.4},\
            {-5,-0.17,-0.16, -0.15, -0.05,     0, 0.15,  0.3,  0.4,  0.5,  0.7, 1.0, 1.4,  1.8,  2.2,  2.6,  2.9,  2.9,  3.1, 3.3,3.4},\
            {-5,-0.17,-0.16, -0.13,  -0.1,     0, 0.03,  0.2,  0.2,  0.3,  0.6, 0.7, 1.1,  1.3,  1.7,  1.8,  2.6,  2.8,  3.0, 3.2,3.3},\
            {-5,-0.36,-0.32,  -0.3, -0.11,  -0.1,    0,  0.1,  0.1,  0.2,  0.4, 0.6, 0.7,  0.8,  1.3,  1.6,  2.0,  2.2,  2.5, 3.0,3.2},\
            {-5,-0.36,-0.32,  -0.3,  -0.2,  -0.1,-0.09,    0,  0.1,  0.2,  0.4, 0.6, 0.7,  0.8,  1.0,  1.2,  1.9,  2.0,  2.1, 2.2,2.5},\
            {-5,-0.36, -0.33, -0.3, -0.25, -0.15,-0.09,-0.03,    0, 0.18, 0.38,0.48, 0.6, 0.76, 0.93,  1.1,  1.2,  1.8,  1.9, 2.1,2.2},\
            {-5,-0.36, -0.33, -0.3, -0.25,  -0.2,-0.15,-0.07,-0.05, 0.15, 0.29,0.32,0.38,  0.5, 0.75,    1, 1.15,  1.3,  1.5, 1.8,  2},\
            {-5,-0.36, -0.33, -0.3, -0.25,  -0.2,-0.15,-0.08,-0.05,  0.1,  0.2,0.22,0.38, 0.43, 0.65, 0.95,  1.1, 1.13, 1.25,1.78,1.9}};
    double speed[] = {1,3,5,7,9,11,13,15,17};
    
    for(int i= 0;i<TORQUE;i++)
      if(torque[i]>=0)
        torque[i] = torque[i]*zoom_factor;
    vector< vector<double> > acc_v;
    vector<double> speed_v;
    vector<double> temp;

    LookUp out;
    //map<double,double>::iterator sec_it;
    for(int i = 0;i<SPEED_S;i++)
    {
        temp.clear();
        for(int j = 0;j<TORQUE;j++)
        {
          temp.push_back(acc[i][j]);
        }
        acc_v.push_back(temp);
    }

    for(int i = 0;i<SPEED_S;i++)
    {
        speed_v.push_back(speed[i]);
    }

    out = LookSection(speed_v,acc_v,target_x);
    double f_Q11,f_Q12,f_Q21,f_Q22;
    double x1,x2,x;
    double y1,y2,y;
    double f_R1,f_R2;
    double out_torque;
    f_Q11 = AccpointToTorque(out.acc_sec_low.first,out.speed_sec.first, torque);
    f_Q12 = AccpointToTorque(out.acc_sec_high.first,out.speed_sec.second, torque);
    f_Q21 = AccpointToTorque(out.acc_sec_low.second,out.speed_sec.first, torque);
    f_Q22 = AccpointToTorque(out.acc_sec_high.second,out.speed_sec.second, torque);
    x1 = acc_v[out.speed_sec.first][out.acc_sec_low.first];
    x2 = acc_v[out.speed_sec.first][out.acc_sec_low.second];
    if(out.acc_low_limit_low)
      x = acc_v[out.speed_sec.first][0];
    else if(out.acc_up_limit_low)
      x = acc_v[out.speed_sec.first][acc_v[out.speed_sec.first].size()-1];
    else
      x = target_x.second;
    f_R1 = (x2-x)/(x2-x1)*f_Q11 +(x-x1)/(x2-x1)*f_Q21;
    x1 = acc_v[out.speed_sec.second][out.acc_sec_high.first];
    x2 = acc_v[out.speed_sec.second][out.acc_sec_high.second];
   if(out.acc_low_limit_high)
      x = acc_v[out.speed_sec.second][0];
    else if(out.acc_up_limit_high)
      x = acc_v[out.speed_sec.second][acc_v[out.speed_sec.second].size()-1];
    else
      x = target_x.second;
    f_R2 = (x2-x)/(x2-x1)*f_Q12 +(x-x1)/(x2-x1)*f_Q22;

    y1 = speed_v[out.speed_sec.first];
    y2 = speed_v[out.speed_sec.second];
    if(out.speed_low_limit)
        target_x.first =speed_v[0] ;
    if(out.speed_up_limit)
        target_x.first =speed_v[speed_v.size()-1];
    y = target_x.first;
    out_torque = (y2 - y)/(y2-y1)*f_R1 + (y-y1)/(y2-y1)*f_R2;
    // cout<<out.speed_sec.first<<"  "<<out.speed_sec.second<<endl;
    // cout<<out.acc_sec_low.first<<"  "<<out.acc_sec_low.second<<endl;
    // cout<<out.acc_sec_high.first<<"  "<<out.acc_sec_high.second<<endl;
    // cout<<out_torque<<endl;
    return out_torque;

}
double drivercontrolXJ::AccpointToTorque(int acc_ind,int speed_ind, double torque[])
{
    return torque[acc_ind];
}
void drivercontrolXJ::UpdateVehicleState(VehicleState_s *vehicle_state) 
{
  static int stop_t = 0;
  static int stable_t = 0;
  static int acc_t = 0;
  static int dec_t = 0;
  static float safe_time = 2;
  if(fabs(sbjspeed)<0.05 && fabs(GetSbjAcc()<0.1))
  {
    stop_t++;
    stable_t = 0;
    acc_t = 0;
    dec_t = 0;
  }
  else if (fabs(sbjspeed)>=0.05 && fabs(GetSbjAcc()<0.1))
  {
    stop_t = 0;
    stable_t++;
    acc_t = 0;
    dec_t = 0;
  }  
  else if((GetSbjAcc()>0.1))
  {
    stop_t = 0;
    stable_t = 0;
    acc_t++;
    dec_t = 0;
  }
  else if((GetSbjAcc()<-0.1))  
  {
    stop_t = 0;
    stable_t = 0;
    acc_t = 0;
    dec_t ++;
  }

  if(stop_t > safe_time) 
  {
    stop_t = 0;
    (*vehicle_state) = STOP;
  } 
  else if(stable_t > safe_time) 
  {
    stable_t = 0;
    (*vehicle_state) = STABLE;
  } 
  else if(acc_t > safe_time) 
  {
    acc_t = 0;
    (*vehicle_state) = ACCELERATE;
  } 
  else if(dec_t > safe_time) 
  {
    dec_t = 0;
    (*vehicle_state) = DECELERATION;
  }

  // msg_state_debug.vehicle_state = (uint)(*vehicle_state);
  unsigned char temp = (unsigned char)(*vehicle_state);
  msg_state_debug.vehicle_state = temp;
}
// void DriverHelp()
// {

// }
void drivercontrolXJ::PubMessageIvdrivercontrol() 
{
  pub_debug.publish(msg_debug);
  pub_.publish(msg_control);
  pub_debug_state.publish(msg_state_debug);
}
void drivercontrolXJ::UpdateMessageIvdriverdebug()
{
  msg_debug.sbjspeed = sbjspeed;
  msg_debug.virtualspeed = virtualspeed;
  msg_debug.acel_cal = sbjaccel;
  msg_debug.acel_temp = acel_desired;
  msg_debug.dt = dt;
}