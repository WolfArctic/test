#include "behaviorplanner.h"

BehaviorPlanner::BehaviorPlanner() 
{
  
}

void BehaviorPlanner::behavior_init()
{
      CURVE_VELOCITY = min(MAX_VELOCITY, CURVE_WEIGHT * MAX_VELOCITY);
      UTURN_VELOCITY = min(MAX_VELOCITY, UTURN_WEIGHT * MAX_VELOCITY);
      start_update_ = clock();
}

BehaviorPlanner::~BehaviorPlanner() 
{

}


double BehaviorPlanner::CalcuPassWidth(double const &distanceObj, double const &relspeedObj, double const &current_curve) 
{
  double passWidth = 1.9;
  double passWidthDist = 1.9;
  double passWidthVrel = 1.9;
  double passWidthLower = 1.0;
  double passWidthUpper = 1.9;
  double passWidthMin = 1.0;

  if (distanceObj > 10) 
  {
    passWidthDist = passWidthLower + 0.03 * (distanceObj - 10);
  }
  else
  {
    passWidthDist = passWidthLower;
  }
  passWidthDist = max(passWidthDist, passWidthLower);
  passWidthDist = min(passWidthDist, passWidthUpper);
  //passWidth = passWidthDist * sigmoid((current_curve-0.1) * 10, 1.0, 2.0);
  return passWidth;
}


vector<double> BehaviorPlanner::CalcuObjPos(float32 x, float32 y, std::vector<sPoint> &pathPointss) 
{
    double dist = 0;
    double length = 0;
    float64 disThre = REALLY_BIG_NUMBER;
    int32 indexStamp = 0;
    for (int i = 0;i < pathPointss.size(); i++) 
    {
      float64 ptX = pathPointss[i].pathpoints_in.x;
      float64 ptY = pathPointss[i].pathpoints_in.y;
      float64 dis = std::hypot((x - ptX),(y - ptY));
      if (disThre > dis) 
      {
        disThre = dis;
        indexStamp = i;
      } 
    }
    dist = disThre;
    length = pathPointss[indexStamp].length;
    return {dist,length};
}

// State Update
/******************************************************
 * Function: UpdateBehavior()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: Determine the current status;
 * Input: optionPathPoints, objDynamic, current_curve
 * Output: 
 * Return: behavior_state
 * Others: None
 *******************************************************/
std::tuple<int,float,float> BehaviorPlanner::UpdateBehavior(std::vector<std::vector<sPoint> > &optionPathPoints, ivpredict::ivmsgpredict &objDynamic, double const &current_curve, ivmap::ivmapmsgvap &carStatus)
{
  int behavior_state = NO_CAR;
  double dist= REALLY_BIG_NUMBER;
  vector<double> length2ego;
  double min_dist = REALLY_BIG_NUMBER;
  double pass_Width = 0;
  int32 indexStamp = 0;

  if(objDynamic.objects.size() == 0)
  {
    return std::make_tuple(NO_CAR, 0, 0);
  }

  for(int i = 0; i<optionPathPoints.size(); i++)
  {
    double checksum = 0;
    for(int j=0; j<objDynamic.objects.size(); j++)
    {
      //if(objDynamic.objects[j].steps>1){
    	length2ego = CalcuObjPos(objDynamic.objects[j].positions[0].xmean, objDynamic.objects[j].positions[0].ymean, optionPathPoints[i]);
    	pass_Width = CalcuPassWidth(length2ego[1], objDynamic.objects[j].v_abs,current_curve);
      //calculate the pass width according to the curve
      if(length2ego[0] < pass_Width)
      { //car in the front of the egocar
        behavior_state = FOLLOW_CAR;
        if(min_dist > length2ego[1])
        {
          min_dist = length2ego[1];
          indexStamp = j;
        }
      }
    }
  }

  if(behavior_state == FOLLOW_CAR)
  {
   double front_car_vabs = objDynamic.objects[indexStamp].xvrel + carStatus.v;
   return std::make_tuple(FOLLOW_CAR, front_car_vabs , min_dist);
  }
  else
  {
  	return std::make_tuple(NO_CAR, 0, 0);
  }
}

// Find the final velocity and final position
/******************************************************
 * Function: UpdateVelocity()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: Determine the local target velocity and target length;
 * Input: current_velocity, behavior, average_curvature, roadtype
 * Output: 
 * Return: target_v, target_s
 * Others: None
 *******************************************************/
vector<double> BehaviorPlanner::UpdateVelocity(double const &current_velocity, const std::tuple<int,float,float> behavior, vector<double> const &average_curvature,  const RoadType roadtype) 
{
  double target_s = 0;
  double target_v_no_obstacle = calculate_velocity_no_obstacle(current_velocity, average_curvature, roadtype);
  double target_v_follow_car = target_v_no_obstacle;

  /*according to the target_velocity, choose a reasonable target_s
  there are two different ways, since the range of acceleration is (0, 1), but the deceleration is (-3, 0), so the deceleration 
   is unlimited*/
  double target_a = sigmoid((target_v_no_obstacle - current_velocity)/3.2,-1,0);
  if(fabs(target_a)>0.1)
  {
      target_s = fabs((target_v_no_obstacle*target_v_no_obstacle -  current_velocity*current_velocity)/(2*target_a));
  }
  else
  {
      target_s = TRAVERSE_TIME * 0.5 * (target_v_no_obstacle + current_velocity);
  }

  auto behavior_state = std::get<0>(behavior);
  auto front_car_speed = std::get<1>(behavior);
  auto front_car_distance = std::get<2>(behavior);

  //in case of the front car velocity is not accurate. Sometimes the measurement of velocity is negative
  front_car_speed = front_car_speed <0.1 ? 0 : front_car_speed;


  if(behavior_state == FOLLOW_CAR)
  {
      // If the car in front is going fast or we are very far from it anyway, go as fast as we can
      // Else let's go a notch slower than the car in front
      /*the magnification of speed is from 0.8 to 1.2
       (2*front_car_speed + FRONT_GAP_THRESH) means the best distance between ego-car and the front car*/
      double speed_buffer = sigmoid( (front_car_distance - 2*front_car_speed -FRONT_GAP_THRESH)/2, 0.8,1); //safe distance 
      if(fabs(front_car_distance - 2*front_car_speed -FRONT_GAP_THRESH) <= SPEED_BUFFER_THESHOLD)
      {  //to keep the speed steady 
        speed_buffer = 1;
      }
      
      /*FRONT_BUFFER is the threshod of follow car distance. 
        distance_buffer is the magnification based on the current_velocity*/
      double distance_buffer = sigmoid((current_velocity - SAFE_VELOCITY)*2, 0.8,1);
      //check1: the front car's speed is faster than the rule of road, we choose the less velocity
      //check2: the front car is far away from the ego car -> front_car_distance
      bool safe = ((front_car_speed*speed_buffer > target_v_no_obstacle)||(front_car_distance > FRONT_BUFFER*distance_buffer));// FRONT_BUFFER = 45 m
      if(!safe)
      {
        target_v_follow_car = (front_car_speed * speed_buffer);
        target_s = set_target_s(current_velocity, target_v_follow_car, behavior);

      }

  }
  double target_v = min(target_v_follow_car,target_v_no_obstacle);

      //end point need to modify.
        // if(average_curvature[3] < 70){
        //   target_v = 0.0;
        //   if(target_s>average_curvature[3]){
        //     target_s = average_curvature[3];
        //   }
        //   //target[0] = min(target[0],Max_curvature.length);
        //   target_s = 7;
        // }
       
  return {target_s,target_v,front_car_speed,front_car_distance, average_curvature[0]};
}

/******************************************************
 * Function: calculate_velocity_no_obstacle()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: Calculate the target velocity based on the curvature;
 * Input: optionPathPoints, current_velocity, average_curvature, roadtype
 * Output: 
 * Return: target_v_no_obstacle
 * Others: None
 *******************************************************/
double BehaviorPlanner::calculate_velocity_no_obstacle(double const &current_velocity, vector<double> const &average_curvature,  const RoadType roadtype)
{

  clock_t current_time = clock();

  bool if_update_time_threshold = time_difference(current_time);

  static double target_v_no_obstacle = MAX_VELOCITY;
  double lookforward_curvature = max(average_curvature[0], average_curvature[4]);
  double check_target_v_update = target_v_no_obstacle;

  /*if the car could not have proper speed to pass the curve, you should change all the parameters below.
  there are three types of curve. through the parameter of average_curvatures, the car would have proper time to in&out curve.*/
  if((roadtype == RoadType::STRIGHT) && (if_update_time_threshold) &&
       (( (target_v_no_obstacle >CURVE_VELOCITY) && (target_v_no_obstacle <=MAX_VELOCITY) && (lookforward_curvature<3.5) ) ||    //out curve
        ( (target_v_no_obstacle > (UTURN_VELOCITY)) && (target_v_no_obstacle <=CURVE_VELOCITY) && (lookforward_curvature<0.6) ) ||  //out U turn
        ( (target_v_no_obstacle <= (UTURN_VELOCITY + 0.1)) && (average_curvature[0] < 1.5) )))  //out U turn
  {
     target_v_no_obstacle = MAX_VELOCITY;
      //U_turn_flag = false;
  } 
    
  if( (roadtype == RoadType::CURVE) && (if_update_time_threshold) &&
     (( (target_v_no_obstacle > CURVE_VELOCITY) && (target_v_no_obstacle <= MAX_VELOCITY) && (lookforward_curvature > 8) ) || //10   //from stright line to curve
      ( (target_v_no_obstacle > UTURN_VELOCITY) && (target_v_no_obstacle <= CURVE_VELOCITY) && (lookforward_curvature > 20) ) ||   //in curve process
      ( (target_v_no_obstacle <= UTURN_VELOCITY) && (average_curvature[0] < 0.2) )))  //out of U turn
  {
    target_v_no_obstacle = CURVE_VELOCITY;//5.56
  }

  if( (roadtype == RoadType::U_TURN) && (if_update_time_threshold) &&
      ( (average_curvature[0] > 220) ||  //any time 
      ( (target_v_no_obstacle > CURVE_VELOCITY) && (target_v_no_obstacle <= MAX_VELOCITY) && (lookforward_curvature > 180) ) || //from stright line to U turn
      ( (target_v_no_obstacle > UTURN_VELOCITY) && (target_v_no_obstacle <= CURVE_VELOCITY) && (lookforward_curvature > 180) ) || //from curve to U turn
      ( (target_v_no_obstacle <= UTURN_VELOCITY) && (lookforward_curvature >2) ))) // in U turn
  {
    target_v_no_obstacle = UTURN_VELOCITY ;//4.16
  }

  //find there is a U-turn in front of you
  if( (roadtype == RoadType::U_TURN) && (average_curvature[2]>220) )
  {
    target_v_no_obstacle = UTURN_VELOCITY;//4.16
  }

  if(fabs(check_target_v_update - target_v_no_obstacle) >1e-3)
  {
    start_update_ = current_time;
  }

  return target_v_no_obstacle;


}



inline bool BehaviorPlanner::time_difference(clock_t &current_time )
{

  bool ok_update = false;
  double duration = (double)(current_time - start_update_) / CLOCKS_PER_SEC;

  if (duration > TIME_THRESHOLD_ROADTYPE) //10 second
  {
    ok_update = true;
  }

  return ok_update;

}

double BehaviorPlanner::set_target_s(double const &current_velocity, const double target_v_follow_car, const std::tuple<int,float,float> behavior)
{

  double target_s = 0;
  auto behavior_state = std::get<0>(behavior);
  auto front_car_speed = std::get<1>(behavior);
  auto front_car_distance = std::get<2>(behavior);

  /*according to the target velocity, the target length is decide by the length of the front car.
          the FRONT_GAP_THRESH = 10m is the safe distance. 
          in order to keep safe, the target length should less than the front car distance, which means the car will reach the target velocity before hit the front car, even if the front car is static.
          while in order to feel comfortable, we decide the target length is longer than the current front car position, because the front car is moving during the whole time.
          based on my experience, the following parameters could make sure the ego car has the minimum distance of 3 meters from the front car 
          you could change the target_s to balance safe index and comfortable index.
  */

  if((front_car_distance - FRONT_GAP_THRESH)>0)  //FRONT_GAP_THRESH = 10
  {
    target_s = front_car_distance + front_car_speed;

    if(target_v_follow_car < 3)
    {
      target_s = front_car_distance - FRONT_GAP_THRESH;
    }
    //target_s = front_car_distance - FRONT_GAP_THRESH +5;
  }
  else if(front_car_distance > 5)
  {
    if( target_v_follow_car > 1)
    {
       target_s = front_car_distance - 3;
    }
    else
    {
      target_s = 1;
    }
    //target_s = front_car_distance - FRONT_GAP_THRESH +5;
  }
  else
  {
    //target_v_follow_car = 0 ;
    target_s = 0;
  }

  if( (target_s > 20) && (target_v_follow_car > current_velocity) )
  {
    target_s = 20;
  }

  return target_s;


}