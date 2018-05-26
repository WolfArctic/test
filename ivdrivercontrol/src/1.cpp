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
  ci_relspeed = 0;
  acel_distance_last = 0;
  torque_temp_last = 0;
  controlstate = 0;
  controlstate_last = 0;
  speedlost_cnt = 0;
  speed_target_p1 = 0;
  is_safe_control = false;
  is_backsliding = false;
  is_too_steep = false;
  is_backsliding_4_obstacle = false;
  driver_flag = 1;
  driver_flag_last = 1;
  virtualspeed = 0;
  emergency_flag = 0;
  shiftposition = 2;
  shiftpositionlast = 2;
  acel_i = 0;
  min_driver_torque = 630;
  vehicle_state = STOP;
  lag_time_s =1.0;
  emergency_f = 0;
  emergency_f_last = 0;
  min = 3.7;//cjs
  max_speed_error_p = 0.2;
  max_speed_error_n = -1;
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
  ci_velocity_upper = ci_velocity_upper_normal;
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

  press_control_pid.kp = acc_kp_torque;
  press_control_pid.ki = acc_ki_torque;
  press_control_pid.integ = 0;
  press_control_pid.integ_max = 100;
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

    //generate_objROI();//cjs

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

double drivercontrol::generate_objROI() 
{ 
  double min1=10;
  double min2=10;
  double passWidth;
  passWidth = 0.5;
  if(ivobj.obj.size() > 0) 
  {
    for (int i = 0; i < ivobj.obj.size(); i++) 
    {
      // if(ivobj.obj[i].type < 4) 
      {
        for (int j = 0; j < ivobj.obj[i].cell.size(); j++) 
        {
          if (fabs(ivobj.obj[i].cell[j].y) <= passWidth && ivobj.obj[i].cell[j].x>1.3)
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
          if (fabs(ivstobj.cell[i].y) <= passWidth && ivstobj.cell[i].x>1.3)
              {
               if(ivstobj.cell[i].x<min2)
               min2=ivstobj.cell[i].x;
              }     
    }
   
  }
  min = MIN(min1,min2);
  return min;
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


// calculate distance between 2 points
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
      if (points[i].x > 0) 
      {
          start_index = i;
          break;
      }
   }
   int preview_index = 0;
   msg_debug.e_distance = points[start_index].velocity;
   const double y_offset = fabs(points[start_index].y);
   double relspeed_target = (points[start_index].velocity - sbjspeed);
  if(driver_flag == 0)
  {
    relspeed_target = -relspeed_target;
  }
   double k1 = 1.4;
   double k2 = 1.0;
   double distance = 2.2;
   double ratio_adjust = 0.2;
   double distance_adjust = 0;
   //double preview_distance = k1 * sbjspeed;
  // double preview_distance = 0.15*sbjspeed*sbjspeed + 0.3*sbjspeed;
   double preview_distance = 0.5*GetSbjAcc()+ sbjspeed;
   distance_adjust = k2*relspeed_target;
   distance_adjust = lpf_1p_apply(&param_temp, distance_adjust);
   distance_adjust = constrain_float(3,0,distance_adjust);
   preview_distance +=  distance_adjust + distance;

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
  // PID dead zone
  double relspeed = targetSpeed - currentSpeed;
  double relspeedLag = lagTargetSpeed - currentSpeed;
  // P element
  double p;
  double i;
//msg_debug.ttci = lagTargetSpeed;
  //relspeed = constrain_float(0.5,-0.5,relspeed);
  p = pid_get_p(&speed_control_pid,relspeed);
  // I element
  //if(fabs(relspeedLag)<0.7)
  i = pid_get_i(&speed_control_pid,relspeedLag, dt);
 
  //  i = 0;
  if(fabs(sbjspeed)- 0.1<0)
  {
    i = 0;
    set_integrator(&speed_control_pid, 0);
    //p = 2*p;
  }
  if(shiftposition == 3 && shiftpositionlast == 1)
  {
    i = 0;
    set_integrator(&speed_control_pid, 0);
  }
  if(shiftposition == 1)
  {
    i = constrain_float(0.2,0.0,i);
    p = constrain_float(0.2,0.0,p);
  }

  acel_speed = p + i;
 // if(shiftposition == 1)
   // acel_speed = constrain_float(-0.2,0.2,acel_speed);
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
  relspeed_temp = constrain_float(max_speed_error_p,max_speed_error_n,relspeed_temp);
  double targer_speed = sbjspeed+relspeed_temp;
  double virspeedLag = SignalLag(lag_time_s,cache_for_lag,targer_speed);
  acel_temp  = PidSpeed(targer_speed, virspeedLag, sbjspeed, dt);

  // accel output limit
   acel_temp = MAX(acel_temp, acel_min);
   acel_temp = MIN(acel_temp, acel_max);

  // BELOW for debug
  msg_debug.sbjspeed = sbjspeed;
  msg_debug.virtualspeed = virtualspeed;

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
  static float stable_speed_dt[] ={200,20,20,20}; 

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
  double torque_max = 2000; 
  double pressure_temp = 0;
  double pressure_max = 2000;
  double modeSwitchLower = -0.1;
  double modeSwitchUpper = 0.1;
  double p;
  double i;
  double e_acc;
  double sbjaccel = GetSbjAcc();

  double k2t = 2000;
  e_acc = (accel_command - sbjaccel);
 // msg_debug.thwi = e_acc;
  double accel_command_lag = SignalLag(lag_time_s,cache_for_lag2,accel_command);
  double  e_acc_lag = accel_command_lag - sbjaccel;
  //driver_flag = 1;
  p = pid_get_p(&acc_control_pid,e_acc_lag);
  i = pid_get_i(&acc_control_pid,e_acc_lag,dt);
  // msg_debug.kps_uphill = p/300;
  // msg_debug.kps_downhill = i/300;
  torque_temp = p + i;
  if(accel_command>=0)
  {
    k2t = 1600;
  }
  else
  {
    k2t = 4500;
  }
  torque_temp = torque_temp + k2t*accel_command;
  if(torque_temp > 0 )
  {
  driver_flag = 1;
  torque_temp = MIN(torque_temp, torque_max);
  torque_temp = MAX(torque_temp, 0);
  torque_temp = torque_temp/torque_max*(torque_max-min_driver_torque)+min_driver_torque;
  } 
  else
  {
    driver_flag = 0;
    torque_temp = MIN(torque_temp, 0);
    torque_temp = MAX(-torque_max, torque_temp);
    torque_temp = (torque_temp+torque_max)/torque_max*min_driver_torque;
    
  }

  torque_temp = MIN(torque_temp, torque_max);
  torque_temp = MAX(torque_temp, 0);
  // !! SAFE CONTROL: use 2nd brake
 /* if (is_safe_control) 
  {
    // driver_flag = 2; // or if there is no 2nd brake:
    //driver_flag = 0;
    //pressure_temp = 0; // pressure_max;
    // actuator node will brake when receiving driver_flag 2; @lining
    DEBUG_LOG("Obstacle too near, enter safe control.");
  }
  if (is_backsliding_4_obstacle) 
  {
    driver_flag = 0;
    pressure_temp = 0; // pressure_max;
    DEBUG_LOG("Obstacle is on ramp, brake!");
  }
  if(is_too_steep) 
  {
    driver_flag = 0;
    pressure_temp = 0; // pressure_max;
    DEBUG_LOG("Ramp is too steep, brake 4ever; will not exit brake unless kill process!");
  }*/

  if (is_start_excitation)
  {
     torque_temp = 650;
     pressure_temp = 0;
     driver_flag = 1;
  }
  else if (fabs(virtualspeed <= 0.1) && fabs(sbjspeed <= 0.15))
  {
     driver_flag = 0;
     pressure_temp = 20; 
     torque_temp = 0;
  }
   // torque_temp = 2000;
   // driver_flag  =1;
  // cout<<"********"<<torque_temp<<endl;
/*static int test_counter = 0;
if(test_counter<30000)
{torque_temp=2000;}
else if(test_counter<200000)
{torque_temp=0;}
test_counter++;*/
//msg_debug.THW_set = torque_temp/1000;
  double dis;
  double deltaS = 0.6;
    dis = sbjspeed * 0.6 + 0.2*sbjspeed * sbjspeed + 1.5; 
    if(min < dis)
        {
         emergency_f = 1;
        }
    else if(min>=dis+deltaS && sbjspeed <= 0.05)
        {
                 emergency_f = 0;
        }//cjs
cout<<"*******"<<emergency_f<<endl;
 if (emergency_f==1) 
  {
      //torque_temp = 0;  
  }   
 if (emergency_flag==1) 
  {
     // driver_flag = 0;
      torque_temp = 0;  
  }



  driver_flag_last = driver_flag;
  torque_temp_last = torque_temp;
  msg_debug.THW_set = min;
  msg_debug.controlstate = dis;//accel_command;
  msg_control.targettorque = torque_temp;
  msg_control.targetacc = accel_command;
  msg_control.actuatormode = 1;
  msg_control.shiftposition = shiftposition;
  msg_debug.acel_cal = sbjaccel;

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

