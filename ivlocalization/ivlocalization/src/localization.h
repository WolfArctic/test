/*************************************************************************
> File Name: localization.h
> Author:Elbert Huang
> Mail:
> Created Time: 2017年03月07日 星期一 17时22分59秒
************************************************************************/

#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

// c++ lib
#include <vector>
#include <fstream>
#include <iostream>
// ROS lib
#include <ros/ros.h>
#include <ros/time.h>
#include <iomanip>

#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

//kalman
#include <kalman/UnscentedKalmanFilter.hpp>
#include "SystemModel.hpp"
#include "MeasurementModel.hpp"

#include "../../../avoslib/geotool.h"
#include "../../../avoslib/globalvariables.h"
#include "ivlocmsg/ivmsglocpos.h"
#include "ivlocmsg/ndt_status.h"
#include "ivlocmsg/ivsensorodom.h"
#include "../../loclib/common.h"
#include "../../loclib/loctool.h"
#include "monitor_client.h"  

using namespace std;
using namespace KalmanApplications;

#define INVALID_VALUE 8888

typedef float T;

// Some type shortcuts
typedef LSAV::State<T> State;
typedef LSAV::Control<T> Control;
typedef LSAV::SystemModel<T> SystemModel;

typedef LSAV::Measurement<T> Measurement;
typedef LSAV::MeasurementModel<T> MeasurementModel;

class localization 
{
public:
  localization(ros::NodeHandle nh);
  ~localization();
  void callback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callback_odom(const ivlocmsg::ivsensorodom::ConstPtr &msg);
  void callback_gps(const sGps::ConstPtr &msg);
  void callback_status(const ivlocmsg::ndt_status::ConstPtr &msg);
  void callback_map(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void callback_initialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void run();
  void gps_to_loc();
  void gps_to_loc(sPoint2d input);
  void pub_gps_pose();
  void pub_loc_pose();
  sLoc matching_to_loc(sPose3Dd temp_pose);
  sLoc matching_to_loc2(sPose3Dd temp_pose);
  void calculate_speed();
  void publish_tf(sLoc loc_pos_input);
  void publish_tf(geometry_msgs::PoseStamped loc_pos_input);
  sPose3Dd loc_to_matching(sLoc temp_loc_pos);
  sPose3Dd pose_fusion();
  void pub_initial_pose(sPose3Dd pose_input);
  bool status_check(sStatus status_input);
  void pose_filter(sLoc& loc_pos_input);
  void pub_std_pose(sPose3Dd pose_input);

  geometry_msgs::PoseStamped get_pose;
  sGps get_gps, gps_origin;
  sLoc loc_pos, loc_pos_gps;
  ivlocmsg::ndt_status ndt_status;
  ivlocmsg::ivsensorodom get_odom;
  bool gps_flag, pose_flag, odom_flag;
  //pose witout gps
  double last_time, last_xg, last_yg, last_velocity;
  int velocity_count;

  geotool geo_tool;
  loclib::loctool loc_tool;
  ofstream fileWritePose;

  //params from launch
  int save_path_, loc_type_;
  double max_x, min_x, max_y, min_y;
  double cell_size_;
  int rotate_img;//-1 for counterclockwise, 0 for unchanged, 1 for clockwise
  std::string file_path;
  double map_angle;

  sGps iabasemapTL;
  double cellSize;
  double previous_xg;
  double previous_yg;
  double previous_lidar_xg, previous_lidar_yg;

  int score_check_count, gps_count;
  bool status_flag, initial_flag, get_intialpose;
  double score_limit_, score_max;
  bool map_flag;
  //int originOrUTM;

  sPose2Dd antenna_pose;
  double origin_x, origin_y;

private:
  ros::Subscriber sub_pose;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_gps;
  ros::Subscriber sub_status;
  ros::Subscriber sub_map;
  ros::Subscriber sub_initialpose;
  ros::Publisher  pub_initialpose;
  ros::Publisher  pub_locpos;
  ros::Publisher  pub_locstdpos;

  tf::TransformBroadcaster pub_tf;
  double timeGps;
  double timeMatching;
  double timeNow;
  double timeCheck;
  double previous_pub_time;

  State locState;
  Control locU;
  SystemModel locSys;
  MeasurementModel locMeasurementModel;
  // Unscented Kalman Filter
  Kalman::UnscentedKalmanFilter<State> ukf;
  bool initial_ukf;

  Monitor *monitor;
};
#endif
