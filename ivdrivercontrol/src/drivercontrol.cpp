#include "drivercontrol.h"

drivercontrol::drivercontrol(ros::NodeHandle nh) 
{
  Init(nh);
  dirver_file.open("/home/idriver/wy_work/wy_work/code/Omga/1114/SeqDataFromDriver.txt");
  


}  

drivercontrol::~drivercontrol() 
{
  dirver_file.close();
}

void drivercontrol::Init(ros::NodeHandle nh) 
{
  InitParam();
  InitLaunchfile(nh);
  InitFilter();
  InitPid();
}
void drivercontrol::InitParam() 
{
  // variable initialize
  loop_freq = 20;
  dt = 0.05;
  sbjspeed = 0;
  sbjspeed_last_for_filter = 0;
  torque_temp_last = 0;
  speedlost_cnt = 0;
  is_backsliding = false;
  is_backsliding_4_obstacle = false;
  virtualspeed = 0;
  emergency_flag = 0;
  shiftposition = 2;
  shiftpositionlast = 2;
  min_driver_torque = 630;
  vehicle_state = STOP;
  lag_time_s =0.8;//TBD
  emergency_f = 0;
  min = 5;//cjs
  min_1 = 5;
  min_2 = 5;
  max_speed_error_p = 0.5;
  max_speed_error_n = -1.5;
  memset(cache_for_lag, 0, sizeof(cache_for_lag));
  memset(cache_for_lag2, 0, sizeof(cache_for_lag));
}
void drivercontrol::InitFilter() 
{
  double acc_cutoff = 1;//hz
  lpf_1p_set_cutoff_freq(&param, dt, acc_cutoff);
  double acc_cutoff_temp = 0.05;//hz
  lpf_1p_set_cutoff_freq(&param_temp, dt, acc_cutoff_temp);
  
}
void drivercontrol::InitLaunchfile(ros::NodeHandle nh)
{
  // PID_speed param, for common road
  nh.param("min_driver_torque", min_driver_torque, min_driver_torque);
  nh.param("kps", kps, kps);
  nh.param("kpsMax", kps_max, kps_max);
  nh.param("kis",kis, kis);
  

  nh.param("ciVelocityLower", ci_velocity_lower, ci_velocity_lower);
  nh.param("ciVelocityUpper", ci_velocity_upper_normal, ci_velocity_upper_normal);
  nh.param("ciVelocityUpperMax", ci_velocity_upper_max, ci_velocity_upper_max);
  nh.param("acel_max", acel_max, acel_max);
  nh.param("acel_min", acel_min, acel_min);

  nh.param("acc_kp_torque", acc_kp_torque, acc_kp_torque);
  nh.param("acc_ki_torque", acc_ki_torque, acc_ki_torque);
  nh.param("acc_kp_press", acc_kp_press, acc_kp_press);
  nh.param("acc_ki_press", acc_ki_press, acc_ki_press);

  nh.param("is_test_virtualspeed", is_test_virtualspeed, is_test_virtualspeed);
  nh.param("is_start_excitation", is_start_excitation, is_start_excitation);
  nh.param("is_pos_control", is_pos_control, is_pos_control);
  // receive data from topic ivactuator
  sub_actuator = nh.subscribe("ivactuator", 1000, &drivercontrol::SubCallbackActuator, this);

  // receive data from topic ivmap
  sub_map = nh.subscribe("ivmapvap", 1000, &drivercontrol::SubCallbackMap, this);

  // receive data from topic ivmsgpath
  sub_motionplanner = nh.subscribe("ivmotionplanner", 1000, &drivercontrol::SubCallbackMotionplanner, this);
  
  // 
  sub_Stcell = nh.subscribe(SUB_TOPIC_OBJST, 10, &drivercontrol::chatterCallbackObjstcell, this);
  sub_Obj     = nh.subscribe(SUB_TOPIC_OBJ, 10, &drivercontrol::chatterCallbackObj, this);
  // publish data through topic ividrivercontrol
  pub_ = nh.advertise<ivdrivercontrol::ivdrivercontrol>("ivdrivercontrol", 1000);

  // publish debug data through topic ividriverdebug
  pub_debug = nh.advertise<ivdrivercontrol::ivdriverdebug>("ivdriverdebug", 1000);
  pub_debug_state = nh.advertise<ivdrivercontrol::ivdrivercontrolstates>("ivdrivercontrolstates", 1000);
}
void drivercontrol::InitPid()
{
  inte_param_init(&speed_integrator_param, dt);
  speed_control_pid.kp = kps;
  speed_control_pid.ki = kis;
  speed_control_pid.integ = 0;
  speed_control_pid.integ_max = ci_velocity_upper_max;

  acc_control_pid.kp = acc_kp_torque;
  acc_control_pid.ki = acc_ki_torque;
  acc_control_pid.integ = 0;
  acc_control_pid.integ_max = 100;

}
void drivercontrol::Run() 
{
  ros::Rate rate(loop_freq);
  double pos_ref = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    memset(&msg_control, 0, sizeof(msg_control));
    memset(&msg_debug, 0, sizeof(msg_debug));
    memset(&msg_state_debug,0,sizeof(msg_state_debug));
    //dt = GetDt();

    generate_objROI();//cjs
    //generate_objROI1();
    //generate_objROI2();

    key_point = FollowPoints(path);
    virtualspeed = key_point.velocity;

    if(is_test_virtualspeed)
      virtualspeed = TestVirtualSpeed();
    if(shiftposition == 1)
      virtualspeed = 0.3;
    double acel_req = DrControl(virtualspeed, dt);
    ActuatorControlPid(acel_req, dt);
    PubMessageIvdrivercontrol();
    UpdateVehicleState(&vehicle_state);
    cout_test();
    //LogWrite(&dirver_file);
    sbjspeed_last = sbjspeed;
    shiftpositionlast = shiftposition;
    rate.sleep();

  }
}


double drivercontrol::generate_objROI1() 
{ 
  double min1=5;
  double min2=5;
  double passWidth = 0.4;
  if(ivobj.obj.size() > 0) 
  {
    for (int i = 0; i < ivobj.obj.size(); i++) 
    {
      // if(ivobj.obj[i].type < 4) 
      {
        for (int j = 0; j < ivobj.obj[i].cell.size(); j++) 
        {
          if (fabs(ivobj.obj[i].cell[j].y) <= passWidth && ivobj.obj[i].cell[j].x > 1.4)
              {
               if(ivobj.obj[i].cell[j].x<min1)
               min1=ivobj.obj[i].cell[j].x;
              }
        }
      }     
    }
  }
  if(ivstobj.cell.size() > 0) 
  {
    for (int i = 0; i < ivstobj.cell.size(); i++) 
    {
          if (fabs(ivstobj.cell[i].y) <= passWidth && ivstobj.cell[i].x > 1.4)
              {
               if(ivstobj.cell[i].x<min2)
               min2=ivstobj.cell[i].x;
              }        
    }
  }
  min_1 = MIN(min1,min2);
  return min_1;
}

double drivercontrol::generate_objROI2() 
{
  double min1=5;
  double min2=5;
  double passWidth = 0.4;
  if(ivobj.obj.size() > 0) 
  {
    for (int i = 0; i < ivobj.obj.size(); i++) 
    {
      // if(ivobj.obj[i].type < 4) 
      {
        for (int j = 0; j < ivobj.obj[i].cell.size(); j++) 
        {
          if (fabs(ivobj.obj[i].cell[j].y) <= passWidth && ivobj.obj[i].cell[j].x < -0.4)
              {
               if(fabs(ivobj.obj[i].cell[j].x)<min1)
               min1=ivobj.obj[i].cell[j].x;
              }
        }
      }     
    }
  }
  if(ivstobj.cell.size() > 0) 
  {
    for (int i = 0; i < ivstobj.cell.size(); i++) 
    {
          if (fabs(ivstobj.cell[i].y) <= passWidth && ivstobj.cell[i].x < -0.4)
              {
               if(fabs(ivstobj.cell[i].x)<min2)
               min2=ivstobj.cell[i].x;
              }        
    }
  }
  min_2 = MIN(fabs(min1),fabs(min2));
  return min_2;
}

int drivercontrol::generate_objROI()
{
  if(shiftposition == 3)
  {
    min = generate_objROI1();
    double dis;
    double deltaS = 0.3;
    dis = sbjspeed * 0.6 + 0.2*sbjspeed * sbjspeed + 1.5;
    if (min < dis) 
    {
      emergency_f = 1;
    }
    else if ((min >= (dis + deltaS) && sbjspeed <= 0.05) || shiftposition == 1) 
    {
      emergency_f = 0;
    }
    
  }
  else if(shiftposition == 1)
  {
    min = generate_objROI2();
     double dis1;
    double deltaS1 = 0.3;
    dis1 = sbjspeed * 0.6 + 0.2*sbjspeed * sbjspeed + 0.4;
    //msg_debug.acel_cal = dis1;
    if (min < fabs(dis1) )
    {
      emergency_f = 1;
    }
    else if ((min >= fabs(dis1 + deltaS1) && fabs(sbjspeed) <= 0.05) || shiftposition == 3) 
    {
      emergency_f = 0;
    }
  }
} 


double drivercontrol::GetDt()
{
  double dt_temp;
  start_time = ros::Time::now();
  dt_temp = (start_time - start_last_time).toSec();
  start_last_time = start_time;
  return dt_temp; 
}
void drivercontrol::cout_test()
{
  //cout<<"vehicle_state "<<vehicle_state<<endl;
 // cout<<"dt***"<<dt<<endl;
  // cout<<"acc_kp_press "<<acc_kp_press<<endl;
  // cout<<"acc_kp_torque "<<acc_kp_torque<<endl;
  // cout<<"acc_ki_torque "<<acc_ki_torque<<endl;
  // if(fp1 == NULL)
  //   {
  //       printf("open file error\n");
  //       return;
  //   }
  //   fprintf(fp1, "virtualspeed:%.4lf sbjspeed:%0.4lf \n", virtualspeed,sbjspeed);
}
void  drivercontrol::LogWrite(ofstream *dirver_file )
{
  if (!dirver_file->is_open())
  {
    cerr<<"Oop!!!"<<endl;
    return;
  }
  int length;
  Log_s log_struct[] = 
  {
    {"sbjspeed", sbjspeed},
    {"virtualspeed", virtualspeed},
    {"acel_max", acel_max}
  };
  length = sizeof(log_struct)/24;
  FileOperate(dirver_file, log_struct, length);
}

void drivercontrol::FileOperate(ofstream *dirver_file, Log_s *log_struct, int length)
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

void drivercontrol::SubCallbackMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg) 
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
    if(msg->currentpathdirection == 0)
       shiftposition = 3;
    else if(msg->currentpathdirection == 1)
       shiftposition = 1;
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

void drivercontrol::SubCallbackMap(const ivmap::ivmapmsgvap::ConstPtr &msg) 
{
  sbjspeed = fabs((double)msg->v);
  SbjspeedFilter();
}

void drivercontrol::SubCallbackActuator(const ivactuator::ivactuator::ConstPtr &msg) 
{
  // sbjspeed<-0.3: set backsliding, otherwise: 0
  is_backsliding = (bool)msg->reserve1;
  DEBUG_LOG(to_string(is_backsliding));
}

// check if num1 is similar to num2
bool drivercontrol::IsClose(const double num1, const double num2, const double factor)
{
  return ((num2 - num1) <= factor && (num2 - num1) >= -factor);
}


// calculate distance_adjust between 2 points
double drivercontrol::CalcDistance(const sPointMsg &point1, const sPointMsg &point2) 
{
  return (sqrt((point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) * (point2.y - point1.y)));
}

/******************************************************************************
// Author     : dunwenqiang
// Date       : 2017.06.08
// Description: check if obstacle is too near, to determine whether enter safe control;
                if not, it search for the key point
// Input      : points: road points
// Output     : NA
// Return     : point: key point
// Others     : NA
******************************************************************************/
sPointMsg drivercontrol::FollowPoints(std::vector<sPointMsg> &points) 
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

/******************************************************************************
// Author     : dunwenqiang
// Date       : 2017.06.08
// Description: get the key point in road points
// Input      : points: road points
// Output     : NA
// Return     : key_point_index: inci_relspeed_trgdex of key point in road points
// Others     : NA
******************************************************************************/
sPointMsg drivercontrol::GetKeyPoint(const std::vector<sPointMsg> &points) 
{
  
    int start_index = 0;
   for (int i = 0; i < points.size(); i++) 
   {
      if (fabs(points[i].x) > 0) //cjs
      {
          start_index = i;
          break;
      }
   }
   int preview_index = 0;
   msg_debug.e_distance = points[start_index].velocity;
   const double y_offset = fabs(points[start_index].y);
   double relspeed_target = points[start_index].velocity - sbjspeed;
   double distance_adjust = 1.2;
   double acc_adjust = 0;
   acc_adjust = kps*relspeed_target;
   acc_adjust = lpf_1p_apply(&param_temp, acc_adjust);
   acc_adjust = constrain_float(0.5,-0.5,acc_adjust);
   double preview_distance = 0;
   double acceleration = GetSbjAcc();

   if(acceleration < 0)
   {
    acc_adjust = -1 * acc_adjust;
   }
   else if(acceleration == 0)
   {
    acc_adjust = 0;
   }

   preview_distance = sbjspeed * lag_time_s + 0.5 * (acceleration + acc_adjust) * lag_time_s * lag_time_s;
   preview_distance =  acc_adjust + distance_adjust;

  if (points.size() > start_index) 
  {
    for (size_t i = start_index; i < points.size(); ++i) 
    {
      if (points[i].length-points[start_index].length >= preview_distance || points[i].velocity <= 0.01) 
      {
        preview_index = i;
        break;
      }
    }
  }
  return points[preview_index];
}
double drivercontrol::GetSbjAcc()
{
    double sbjaccel = (sbjspeed - sbjspeed_last)/dt;
    return lpf_1p_apply(&param, sbjaccel);
}

double drivercontrol::PidSpeed(const double targetSpeed, const double lagTargetSpeed, const double currentSpeed, double dt) 
{
  double acel_speed = 0;
  double relspeed = targetSpeed - currentSpeed;
  double relspeedLag = lagTargetSpeed - currentSpeed;
  double p;
  double i;
  p = pid_get_p(&speed_control_pid,relspeed);
  i = pid_get_i(&speed_control_pid,relspeedLag, dt);
  i = constrain_float(0.2 , -0.2 , i);
  if(fabs(sbjspeed)- 0.1 < 0)
  {
    i = 0;
    set_integrator(&speed_control_pid, 0);
  }
  if(shiftposition != shiftpositionlast)
  {
    i = 0;
    set_integrator(&speed_control_pid, 0);
  }
  msg_debug.speed_p = p;
  msg_debug.speed_i = i;
  acel_speed = p + i;

  return acel_speed;
}



double drivercontrol::DrControl(double virspeed, double dt) 
{
  DEBUG_LOG("Entrance.");
  double acel_temp = 0;
  double acel_cal = 0;
  double acel_raw = 0;
  double e_distance = 0;
  double relspeed_temp = virspeed - sbjspeed;
  msg_debug.virtualspeed = virspeed;
  msg_debug.sbjspeed = sbjspeed;
  relspeed_temp = constrain_float(max_speed_error_p,max_speed_error_n,relspeed_temp);//max_speed_error_p  max_speed_error_n  TBD
  double targer_speed = sbjspeed+relspeed_temp;
  double virspeedLag = SignalLag(lag_time_s,cache_for_lag,targer_speed);
  acel_temp  = PidSpeed(targer_speed, virspeedLag, sbjspeed, dt);
  acel_temp = MAX(acel_temp, acel_min);
  acel_temp = MIN(acel_temp, acel_max);
  return acel_temp;
}

double drivercontrol::TestVirtualSpeed()
{
  static int test_counter = 0;
  static bool is_test_counter_increase = true;

  static float start_t[10] = {0};
  static float start_speed[10] = {0};

  start_t[0] = 10;
  start_speed[0] = 0;
  static float end_speed[] = {0.8,1.0,0.8,0};
  static float acc[] = {0.5,0.5,-0.4,-0.2};
  static float stable_speed_dt[] ={20,20,20,20}; 

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
  if (2600 == test_counter || 0 == test_counter) 
  {
    is_test_counter_increase = !is_test_counter_increase;
  }
        
    return virtualspeed_temp;
}
void drivercontrol::SpeedPlan(float start_point,float start_speed,float acc,float end_speed,float stable_speed_dt,float *end_speed_t,float *end_t)
{
  (*end_speed_t) = start_point + fabs((end_speed - start_speed)/acc);
  (*end_t) = (*end_speed_t) + stable_speed_dt;
}
double drivercontrol::SignalLag(uint8_t lag_time_s, double *cache_for_lag, double new_signal)
{
  uint8_t lag_tick = lag_time_s / dt;
  for(int i=0;i<lag_tick-1;i++)
  {
    cache_for_lag[i] = cache_for_lag[i+1];
  }
  cache_for_lag[lag_tick-1] = new_signal;
  return  cache_for_lag[0];                  

}


int drivercontrol::ActuatorControlPid(double accel_command,double dt) 
{
  DEBUG_LOG("Entrance.");
  int errCode = 0;
  double torque_temp = 0;
  double look_up_torque = 0;
  double torque_max = 2500; 
  double p;
  double i;
  double e_acc;
  double sbjaccel = GetSbjAcc();
  look_up_torque = AccToTorqueNew(accel_command, sbjspeed);
  e_acc = (accel_command - sbjaccel);
  double accel_command_lag = SignalLag(lag_time_s,cache_for_lag2,accel_command);
  double  e_acc_lag = accel_command_lag - sbjaccel;
  p = pid_get_p(&acc_control_pid,e_acc_lag);
  i = pid_get_i(&acc_control_pid,e_acc_lag,dt);
  p = constrain_float(300 , 0 , p);
  i = constrain_float(200 , 0 , i);
  msg_debug.acel_p = p/200;
  msg_debug.acel_i = i/200;
  torque_temp = look_up_torque + p + i;
  torque_temp = MIN(torque_temp, torque_max);
  torque_temp = MAX(torque_temp, 0);

  if (is_start_excitation)
  {
     torque_temp = 650;
  }

  if (emergency_flag == 1 || emergency_f == 1) 
  {
      torque_temp = 0;  
  }
  torque_temp_last = torque_temp;
  msg_debug.acel_temp = accel_command;
  msg_control.targettorque = torque_temp;
  msg_control.targetacc = 0;
  msg_control.actuatormode = 1;
  msg_control.shiftposition = shiftposition;
  msg_debug.acel_cal = sbjaccel;
  msg_debug.acel_esp = look_up_torque;

  return errCode;
}



/******************************************************************************
// Author     : yanbo
// Date       : **
// Description: car speed filter
// Input      : NA
// Output     : modi member variable: sbjspeed
// Return     : NA
// Others     : NA
******************************************************************************/
void drivercontrol::SbjspeedFilter() 
{
  if (((fabs(sbjspeed - sbjspeed_last_for_filter) >= 2) || (sbjspeed_last_for_filter >= 1 && sbjspeed == 0)) && speedlost_cnt < 20) 
  {
    sbjspeed = sbjspeed_last_for_filter;
    ++speedlost_cnt;
    if (speedlost_cnt >= 200) 
    {
      speedlost_cnt = 200;
    }
  }
  else 
  {
    sbjspeed_last_for_filter = sbjspeed;
    speedlost_cnt = 0;
  }
}
void drivercontrol::UpdateVehicleState(VehicleState_s *vehicle_state) 
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
  unsigned char temp = (uint)(*vehicle_state);
  msg_state_debug.vehicle_state = temp;
  cout<<"****vehicle_state"<<temp<<endl;
  cout<<"****msg_state_debugvehicle_state"<<msg_state_debug.vehicle_state<<endl;
}
// void DriverHelp()
// {

// }
void drivercontrol::PubMessageIvdrivercontrol() 
{
  pub_debug.publish(msg_debug);
  pub_.publish(msg_control);
  ivdrivercontrol::ivdrivercontrolstates msg_state_debug1;
  msg_state_debug1.vehicle_state = vehicle_state;
  pub_debug_state.publish(msg_state_debug1);
}

void drivercontrol::chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg)
{
    ivobj = *msg;
}

void drivercontrol::chatterCallbackObjstcell(const ivmap::ivmapmsgstcell::ConstPtr &msg)
{
    ivstobj = *msg;
}

double drivercontrol::AccToTorqueNew(double acc_desired,double speed_now)
{
    int aa=0;
    int vv=0;
    double Q11=0;
    double Q12=0;
    double Q21=0;
    double Q22=0;
    double T1=0;
    double T2=0;
    double T_finally=0;
    double acc_lower=0;
    double acc_upper=0;
    double speed_lower=0;
    double speed_upper=0;
    const int velocity_num=11;
    const int acc_num =14;
    double acc[acc_num]={-1.0, -0.8, -0.6, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8};//加速度为横坐标
    double velocity[velocity_num]={0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0};//速度为纵坐标
    double torque[velocity_num][acc_num]={
              {  0,   0,   0,   0,  100,  250,  380,  500,  620,  750,  900, 1200, 1700, 2000},//0
              {  0,   0,   0,  50,  210,  350,  480,  600,  720,  850, 1150, 1400, 1900, 2200},//0.2
              {  0,   0,   0, 160,  310,  450,  580,  700,  820, 1100, 1350, 1700, 2000, 2400},//0.4
              {  0,   0, 100, 260,  410,  550,  680,  800, 1050, 1300, 1650, 1900, 2200, 2500},//0.6
              {  0,  30, 200, 330,  480,  620,  750, 1000, 1250, 1600, 1850, 2000, 2400, 2500},//0.8
              {  0, 100, 240, 400,  550,  690,  950, 1200, 1550, 1800, 1950, 2200, 2500, 2500},//1.0
              {  0, 140, 310, 470,  620,  890, 1150, 1500, 1750, 1900, 2150, 2400, 2500, 2500},//1.2
              { 30, 210, 380, 540,  820, 1090, 1450, 1700, 1850, 2100, 2350, 2450, 2500, 2500},//1.4
              {100, 280, 450, 740, 1020, 1390, 1650, 1800, 2050, 2300, 2400, 2500, 2500, 2500},//1.6
              {170, 350, 650, 940, 1320, 1590, 1750, 2000, 2250, 2350, 2450, 2500, 2500, 2500},//1.8
              {240, 550, 850,1240, 1520, 1690, 1950, 2200, 2300, 2400, 2500, 2500, 2500, 2500}};//2.0
            // -1 ,-0.8,-0.6,-0.4, -0.3, -0.2, -0.1,    0,  0.1,  0.2,  0.3,  0.4,  0.6,  0.8
            //求出期望加速度两侧的加速度
            if(acc_desired>=acc[acc_num-1])
            {
                acc_desired=acc[acc_num-1];
            }
            else if(acc_desired<=acc[0])
            {
                acc_desired=acc[0];
            }

            for(int i=0;i<(acc_num-1);i++)
            {
               if(acc[i]<=acc_desired&&acc_desired<=acc[i+1])
               {
                   acc_lower=acc[i];//加速度左x1
                   acc_upper=acc[i+1];//加速度右x2
                   aa=i;
               }
            }

            //求出期望速度两侧的速度

             if(speed_now>=velocity[velocity_num-1])
            {
               speed_now>=velocity[velocity_num-1];
            }
            else if(speed_now<=velocity[0])
            {
                speed_now=velocity[0];
            }

            for(int j=0;j<(velocity_num-1);j++)
            {
               if(velocity[j]<=speed_now&&speed_now<=velocity[j+1])
               {
                   speed_lower=velocity[j];//速度左y1
                   speed_upper=velocity[j+1];//速度右y2
                   vv=j;
               }
            }
            Q11=torque[vv][aa];
            Q12=torque[vv][aa+1];
            Q21=torque[vv+1][aa];
            Q22=torque[vv+1][aa+1];
//            cout<<"acc_desired "<<acc_desired<<endl;
//            cout<<"speed_now "<<speed_now<<endl;
//            cout<<"acc_lower "<<acc_lower<<endl;
//            cout<<"acc_upper "<<acc_upper<<endl;
//            cout<<"speed_lower "<<speed_lower<<endl;
//            cout<<"speed_upper "<<speed_upper<<endl;
//            cout<<"Q11 "<<Q11<<endl;
//            cout<<"Q12 "<<Q12<<endl;
//            cout<<"Q21 "<<Q21<<endl;
//            cout<<"Q22 "<<Q22<<endl;
//            cout<<"vv"<<vv<<endl;
//            cout<<"aa"<<aa<<endl;
//            cout<<"torque[4][16]"<<torque[4][16]<<endl;
            T1=(acc_upper-acc_desired)/(acc_upper-acc_lower)*Q11+(acc_desired-acc_lower)/(acc_upper-acc_lower)*Q12;
            T2=(acc_upper-acc_desired)/(acc_upper-acc_lower)*Q21+(acc_desired-acc_lower)/(acc_upper-acc_lower)*Q22;
            T_finally=(speed_upper-speed_now)/(speed_upper-speed_lower)*T1+(speed_now-speed_lower)/(speed_upper-speed_lower)*T2;
            return T_finally;
}
