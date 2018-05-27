#include "pathdecision.h"



PathDecision::PathDecision() 
{
  ros::NodeHandle mh;

  mean_filter_init(&mf1,3);
  mean_filter_init(&filter_uturn_size,13);
  mean_filter_init(&filter_vlookforward_size,8);
  //planner.behavior_init(MAX_VELOCITY);
  visualization = new Visualization(mh);
  visualization_long = new Visualization(mh);
}


PathDecision::~PathDecision() 
{
  if(visualization)
    delete visualization;
  if(visualization_long)
    delete visualization_long;
}


/******************************************************
 * Function: PathCurve()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: Calculate the looking forward curvature and path type ;
 * Input: optionPathPoints, current_velocity, average_curvature
 * Output: spline_value
 * Return: RoadType
 * Others: None
 *******************************************************/
RoadType PathDecision::PathCurve(std::vector<std::vector<sPoint> >  &optionPathPoints,  const double &ego_speed, vector<double> &spline_value)
{
  spline_value.clear();
  
  vector<double> xs;
  vector<double> ys;
  vector<double> ss;
  vector<double> curvatures;
  double sum = 0;
  double sum_velocity = 0;
  double temp_curvature = 0; 
  double filter_curvature = 0;
  double filter_curvature_velocity = 0;
  //look forward for the U turn
  double temp_curvature_U = 0;
  double temp_curvature_velocity = 0;
  double sum_U = 0;

  /*
  sigmoid((MAX_VELOCITY - STANDARD_VELOCITY)/1.5,0.5,1.7) is to make sure different maximum velocities could have same results, it leads tp the bigger velocity, the longer foward point.
  sigmoid((ego_speed/2),-3,0.1)/pow(ego_speed/MAX_VELOCITY,6) change the looking forward point based on the ego_velocity
  all parameters in front of the equations are changable. such as 10/30/4/5
  */
  double lookforward_startlength_U = 10 * sigmoid((MAX_VELOCITY - STANDARD_VELOCITY)/1.5,0.5,1.7)*sigmoid((ego_speed/2),-3,0.1);
  double lookforward_startlength = 30 * sigmoid((MAX_VELOCITY - STANDARD_VELOCITY)/1.0,0.5,1.7)*pow(ego_speed/MAX_VELOCITY,6);
  double lookforward_startlength_velocity =  30 * sigmoid((MAX_VELOCITY - STANDARD_VELOCITY)/1.0,0.5,1.7)*pow(ego_speed/MAX_VELOCITY,1);
  double lookforward_length_u = 4* sigmoid((ego_speed/2),-2,0.1);
  double lookforward_length = 5* sigmoid((ego_speed/2),-2,0.1);
  int count_size = 0;
  int count_size_velocity = 0;
  int count_size_U = 0;
  int roadtype_count = 0;
  int roadtype_u_count = 0;
  double temp_length = lookforward_startlength + lookforward_length;
  static double filter_roadtype_u_count = 0;
  bool timefirst = true;
  bool timefirst2 = true;
  bool timefirst3 = true;
  bool timefirst4 = true;
  vector<double> look_position;
  look_position.clear();
  
  static double curvature_pre_pre = 0;
  static double curvature_pre = 0;
  //bool 
  for(int i = 0; i<optionPathPoints.size(); i++)
  {
      double check_curve_length = max(optionPathPoints[i][optionPathPoints[i].size()-1].length * ego_speed/MAX_VELOCITY, 50.0);
      
      for(int j = 1;j<optionPathPoints[i].size()-1;j++)
      {
          xs.emplace_back(optionPathPoints[i][j].pathpoints_in.x);
          ys.emplace_back(optionPathPoints[i][j].pathpoints_in.y);
          curvatures.emplace_back(optionPathPoints[i][j].curvature);
          Max_curvature.curvature = optionPathPoints[i][j].curvature;
          Max_curvature.length = optionPathPoints[i][j].length;  // the maximum value of length
          Max_path_curvature.emplace_back(Max_curvature);
          ss.emplace_back(optionPathPoints[i][j].length);
              
          if((Max_curvature.length > lookforward_startlength) && 
            (Max_curvature.length <= (lookforward_startlength + lookforward_length) ))
          {
            sum += fabs(optionPathPoints[i][j].curvature);
            count_size++;
          }
  
          if((Max_curvature.length > lookforward_startlength_velocity) && 
            (Max_curvature.length <= (lookforward_startlength_velocity + lookforward_length) ))
          {
            sum_velocity += fabs(optionPathPoints[i][j].curvature);
            count_size_velocity++;
          }            

          /*
          0.02 and 0.3 are threshold values to decide the road point is curve line or U turn.
          combined with the NUMBER_OF_CURVE & NUMBER_OF_U_TURN to decide the looking forward line is stright, curve, or U turn.
          */

          if((fabs(optionPathPoints[i][j].curvature) > CURVE_POINT_CURVATURE) && 
              (Max_curvature.length < check_curve_length))
          {
            roadtype_count++;
            if(fabs(optionPathPoints[i][j].curvature) > UTURN_POINT_CURVATURE)
            {
              roadtype_u_count++;
            }
          }
  
          if((Max_curvature.length > lookforward_startlength_U) &&
             (Max_curvature.length <= (lookforward_startlength_U + lookforward_length_u)))
          {
            sum_U += fabs(optionPathPoints[i][j].curvature);
            count_size_U++;
          }

          //*********************************************************for visulization**********************************************************//
  
          if((Max_curvature.length > check_curve_length) && (timefirst3))
          {
              look_position.push_back(optionPathPoints[i][j].pathpoints_in.x);
              look_position.push_back(optionPathPoints[i][j].pathpoints_in.y);
              timefirst3 = false;           
          }

          if((Max_curvature.length>lookforward_startlength_U ) && (timefirst4))
          {
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.x);
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.y);
            timefirst4 = false;
            //visualization_long->showPoint(optionPathPoints[i][j].pathpoints_in.x,optionPathPoints[i][j].pathpoints_in.y);
          }
    
          if((Max_curvature.length>lookforward_startlength ) && (timefirst))
          {
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.x);
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.y);
            timefirst = false;
          }
          if( (Max_curvature.length > (lookforward_startlength + lookforward_length)) && (timefirst2) )
          {
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.x);
            look_position.push_back(optionPathPoints[i][j].pathpoints_in.y);
            timefirst2 = false;
          }

          //*************************************************************************************************************************************//
      }

  }

  visualization->showPoint(look_position);

  //calculate the average curvature of looking forward points. and filter them with mean_filter_nosort_apply.

  if(count_size > 0)
  {
    // ROS_INFO("fs_filter_curvature_sum = %f", sum);
    // ROS_INFO("fs_filter_curvature_count_size = %d", count_size);
    temp_curvature = ( sum / count_size)*1000;
  }
  if(count_size_U > 0)
  {
    temp_curvature_U = ( sum_U / count_size_U)*1000;
  }

  if(count_size_velocity > 0)
  {
    temp_curvature_velocity = (sum_velocity / count_size_velocity) * 1000;
  }

  // mean filter to smooth the curvature variant
  filter_curvature = mean_filter_nosort_apply(&mf1,temp_curvature);
  filter_roadtype_u_count =  mean_filter_nosort_apply(&filter_uturn_size,roadtype_u_count);
  filter_curvature_velocity = mean_filter_nosort_apply(&filter_vlookforward_size, temp_curvature_velocity);

  // ROS_INFO("fs_filter_curvature = %f", temp_curvature);

  // ROS_INFO("fs_number of roadtype = %d", roadtype_count);

  spline_value = {filter_curvature, temp_length, temp_curvature_U, Max_curvature.length, filter_curvature_velocity};


  if(filter_roadtype_u_count > NUMBER_OF_U_TURN)
  {
    return RoadType::U_TURN;
  }
  else if(roadtype_count > NUMBER_OF_CURVE)
  {
    return RoadType::CURVE; 
  }
  else
  {
    return  RoadType::STRIGHT;
  }

}