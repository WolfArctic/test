
#include "velocitygeneration.h"

VelocityGeneration::VelocityGeneration() 
{
  ros::NodeHandle mh;
}



VelocityGeneration::~VelocityGeneration()
{
}


/******************************************************
 * Function: SetVelocity()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: calculate length, velocity, acceleration on every points;
 * Input: orignal path points of optionPathPoints, ego velocity carStatus, local path planning(target), if_follow_car behaviorState
 * Output: velocity_path
 * Return: orignal path with velocity
 * Others: None
 *******************************************************/
double VelocityGeneration::SetVelocity(std::vector<std::vector<sPoint> >  &optionPathPoints,ivmap::ivmapmsgvap &carStatus,  vector<double> &target, std::tuple<int,float,float> behaviorState, int objFlag, double accelEgo)
{
  auto if_follow_car = std::get<0>(behaviorState);
  auto follow_car_velocity = std::get<1>(behaviorState);
  int test_in = 0;
  static int if_update = 0;
  ego_velocity = carStatus.v;
  static double current_updated_velocity = ego_velocity;
  static double current_velocity_record = ego_velocity;
  static double current_velocity_target = ego_velocity;
  vector<State> ss;
  State temp;
  double gap_time = 0.01;
  //double temp_time = ros::Time::now().toSec()*1000;
  static double temp_max_a = 0;
  static double temp_min_a = 0;
  static double temp_acceleration = 0;
  static double current_update_length = 0;
  static double s_start_a = 0.0;
  static double plan_speed = 0;

  bool stop_sign = 0;
  bool update_current_velocity = 0;
  static bool first_time_stop = 1;
  int temp_number = 0;
  double print_target_s = 0;

  //int looking = 500 * (MAX_VELOCITY/STANDARD_VELOCITY);

  //check if the looking forward point has velocity or not? looking is the length point.
  int looking = optionPathPoints[0].size()-2;
  bool breakingflag = false;
  for(int i = 0; i<optionPathPoints.size(); i++)
  {
    for(int j = 1;j<optionPathPoints[i].size()-1;j++)
    {
      if(FORWARD_LENGTH < optionPathPoints[i][j].length)
      {
        looking = j;
        breakingflag = true;
      }
      if(breakingflag)
        break;
    }
    if(breakingflag)
        break;
  }

  //decide if need to update the velocity on the road
  
  double error_velocity = fabs(target[1] - current_velocity_target); 
	if(error_velocity > VELOCITY_DIFFERNCE)
	{
		current_velocity_record = ego_velocity; //record current velocity, record when ego_velocity is beyond the range of current velocity, stop update
		current_velocity_target = target[1]; // record the target, means only the variation of target velocity beyond 0.3m/s, update the target velocity.
    
    if(fabs(ego_velocity - current_velocity_target) < 0.5 && if_follow_car ==1 && follow_car_velocity > 1 )
    {
      target[0] += 10; 
    }
  }

  if(ego_velocity < 0.05 )
  {
    if(if_follow_car == 1 && target[0] < 0.1)
    {
      target[0] = 0;
      target[1] = 0;
      current_velocity_target = target[1];
      stop_sign = 1;
    }
    else if(first_time_stop)
    {
      stop_sign = 1;
      first_time_stop = 0;
    }
  }

  if((ego_velocity > 2) && (first_time_stop == 0))
  {
    first_time_stop = 1;
  }

  //if(fabs(ego_velocity - g_start_v) > 1 )
  if(fabs(ego_velocity - g_start_v) > 1  || objFlag)   //TODO @pqg 0322  when obj exists , update velocity forever
  {
    g_start_v = ego_velocity ;
    update_current_velocity = 1;
  }

   SetAcceleration(optionPathPoints, current_velocity_target, s_start_a,objFlag, accelEgo);  //TODO @pqg 0323

  //update condition
  if( (fabs(current_updated_velocity - current_velocity_target) > VELOCITY_DIFFERNCE && if_update == 0) || 
  	  optionPathPoints[0][looking].pathpoints_in.velocity < 0 ||
      stop_sign || update_current_velocity)
  {
    
      target[1] = current_velocity_target;

      print_target_s = target[0];


      if( ((target[1] < 0.01) && (ego_velocity < 0.01)))
      {
	     	  if(target[0] < 0.01)
	      	{
	      		  temp.p = 0;
	          	temp.v = 0;
              temp.a = 0;
	          	ss.push_back(temp);
	      	}
	      	else
	      	{
	      		double temp_v = sqrt(target[0]*0.1); //max velocity of velocity
	      		temp_v = temp_v > 3 ? 3 : temp_v; 
	      		State target_temp1 = {target[0]/2, temp_v, 0.0};
	      		State start_temp1  = {0, ego_velocity, 0.2};
	      		State target_temp2 = {target[0], target[1], 0.0};
	      		double temp_t = target[0]/temp_v;
	      		//test_in = 8;
	      		int temp_n = temp_t/gap_time;
	      		JMT temp_jmt_s1(start_temp1, target_temp1, temp_t);
	      		MakePath(temp_jmt_s1, gap_time, temp_n, ss);

	      		JMT temp_jmt_s2(target_temp1, target_temp2, temp_t);
	      		MakePath(temp_jmt_s2, gap_time, temp_n, ss);

	      	}
      }
      else
      {
        if(fabs(target[1] + g_start_v) <0.001)
          target[1] = 0.001;
      	
        double travel_time = 2*target[0]/(target[1] + g_start_v);

        State target_state_s = {target[0], target[1], 0.0};
      	State start_state_s = {0,g_start_v,s_start_a};
        int n =  travel_time/gap_time;
        //11/21 
        if(n<1)
        {
            temp.p = 0;
            temp.v = 0;
            temp.a = 0;
            ss.push_back(temp);
        }
        else
        {/*******************************************************************************  optimize trajectory********************************************/
          TrjObject trjobj;
          vector<double> temp_start_s = {0,g_start_v,s_start_a};
          vector<double> temp_target_s = {target[0], target[1], 0.0};
          double delta = 0;
          if(!if_follow_car)
          {
            delta = NO_CAR_DELTA_S;
          }
          else
          {
            delta = FOLLOW_CAR_DELTA_S;
          }

          trjobj = m_trajectory.ChooseBestTrajectory(temp_start_s,temp_target_s, delta, behaviorState);
        
          target_state_s = {trjobj.goal[0],trjobj.goal[1],trjobj.goal[2]};

          #if MOTIONDEBUG
          
          double gap_s = trjobj.goal[0] - target[0];
          bool found_trajectory = false;

          while(!found_trajectory)
          {
            if(fabs(temp_number * delta - gap_s) <0.01 )
              found_trajectory = true;
            temp_number ++;
          }
          #endif

/***************************************************************************************************************************************/

          n = trjobj.t/gap_time;

          //start point     
          JMT jmt_s(start_state_s, target_state_s, trjobj.t);

          MakePath(jmt_s, gap_time, n, ss);
          if(target[1] < 0.1)
          {
            temp.p += 0.1;
            temp.v = 0;
            temp.a = 0;
            ss.push_back(temp); 
          }

        }  

      }

      //set velocity to the remaining points
      
      double maxlength = optionPathPoints[0][optionPathPoints[0].size()-1].length;
      if( (maxlength - target[0]) > 0.1 && target[1] > 0.3 && target[0] > 0.3)
      {
          State target_state_s_continue = {maxlength, target[1], 0.0};
          State start_state_s_continue = {target[0], target[1], 0.0};
          double travel_time = (maxlength-target[0])/target[1];
          JMT jmt_s(start_state_s_continue, target_state_s_continue, travel_time);
          int n = travel_time/gap_time;
          for(int i =0; i< n; i++) 
          {
          	temp.p = jmt_s.get(i*gap_time);
          	temp.v = jmt_s.get_velocity(i*gap_time);
            temp.a = jmt_s.get_acceleration(i*gap_time);
          	temp_acceleration = temp.a;
          	ss.push_back(temp);
          	temp_max_a = max(temp_acceleration, temp_max_a);
          	temp_min_a = min(temp_acceleration, temp_min_a);
          }

     	}

      //revert the initial ss to the original points.

      vector<double> vel_acc_temp;
      for(int i=0; i<optionPathPoints.size(); i++)
      {
          for(int j=0; j<optionPathPoints[i].size(); j++)
          {
             vel_acc_temp = this->convert_velocity_acc_to_originals(ss, optionPathPoints[i][j].length);
          		optionPathPoints[i][j].pathpoints_in.velocity = vel_acc_temp[0];
             optionPathPoints[i][j].pathpoints_in.a = vel_acc_temp[1];
          }
      }
      current_updated_velocity = target[1];
      current_update_length = target[0];
      if_update = 1;
  }
  else
  {
    if_update = 0;
  }

  plan_speed = optionPathPoints[0][0].pathpoints_in.velocity;


  //SetAcceleration(optionPathPoints, current_velocity_target, s_start_a,objFlag);  //It should be placed at first position 
	
  #if MOTIONDEBUG
  velocity_bug.speed = optionPathPoints[0][0].pathpoints_in.velocity;
  velocity_bug.start_acceleration = optionPathPoints[0][0].pathpoints_in.a *10;
  
	velocity_bug.target_velocity = current_velocity_target;
	velocity_bug.target_length = print_target_s;
	velocity_bug.ego_velocity = ego_velocity;
  velocity_bug.inttest1 = test_in;
  //velocity_bug.inttest2 = test_looking;
  velocity_bug.followcar_veloicty = target[2];
  velocity_bug.followcar_length = target[3];
  velocity_bug.lookforward_curvature = target[4];
  //velocity_bug.optimal_trajectory_num = temp_number;
  #endif

  return g_start_v;
 }

 ivpathplanner::velocitydebug VelocityGeneration::getData()
 {

  return velocity_bug;
 }



int VelocityGeneration::SetAcceleration(std::vector<std::vector<sPoint> > const &optionPathPoints,double const &current_velocity_target, double &s_start_a, int objMark, double accelCurrent)
{
  
  if (objMark == 0)
  {
     g_start_v = optionPathPoints[0][0].pathpoints_in.velocity > 0 ? optionPathPoints[0][0].pathpoints_in.velocity : ego_velocity;
     
     // cout<<"0326***debug01***Point_start_a"<<optionPathPoints[0][1].pathpoints_in.a<<endl;
     
     if ( fabs(optionPathPoints[0][1].pathpoints_in.a) < 5 )
     {
        s_start_a = optionPathPoints[0][1].pathpoints_in.a;

        // cout<<"0326***debug02***start_a"<<s_start_a<<endl;
     }
     else
     {
        //s_start_a = 0 ;
        s_start_a = accelCurrent;//@pqg TODO 0326
        // cout<<"0326***debug03***start_a"<<s_start_a<<endl;
     }
     
  }
  else if (objMark == 1)
  {
     g_start_v = ego_velocity;
     if (current_velocity_target > ego_velocity )
     {
       s_start_a = 0.5 ;
     }
     else if (current_velocity_target < ego_velocity)
     {
       s_start_a = -0.8;
     }
     else
     {
       s_start_a = 0 ;
     }
  }

  if (s_start_a > 3)   //limited @pqg TODO 0326
  {
    s_start_a = 3;
  }
  if (s_start_a < -5)
  {
    s_start_a = -5;
  }

  return 1;
}

/******************************************************
 * Function: convert_velocity_acc_to_originals()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: value the velocity of orignal path based on planned path;
 * Input: motion path, orignal path
 * Output: 
 * Return: orignal path with velocity
 * Others: None
 *******************************************************/
vector<double> VelocityGeneration::convert_velocity_acc_to_originals(vector<State> &jmts,const double originals) const
{
  double vel_temp = 0;
  double acc_temp = 0;
  
  if(originals < jmts[0].p)
  {
    vel_temp = jmts[0].v;
    acc_temp = jmts[0].a;
  }
  else
  {
    for(int i =0; i<jmts.size(); i++)
    {
      if( (originals >= jmts[i].p) && (originals < jmts[i+1].p) )
      {
        vel_temp = jmts[i].v;
        acc_temp = jmts[i].a;
        break;
      }
    }
  }
  return {vel_temp, acc_temp};
 }

/******************************************************
 * Function: MakePath()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: calculate length, velocity, acceleration on every points;
 * Input: parameter of jmt equation, t, n points
 * Output: velocity_path
 * Return: orignal path with velocity
 * Others: None
 *******************************************************/
void VelocityGeneration::MakePath(JMT jmt_s, const double t, const int n, vector<State> &velocity_path)
{

  State temp;

  for(int i=0; i<n; i++) 
  {
    temp.p = jmt_s.get(i*t);
    temp.v = jmt_s.get_velocity(i*t);
    temp.a = jmt_s.get_acceleration(i*t);
    velocity_path.push_back(temp); 
  }
 }

