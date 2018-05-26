#ifndef GPS2ODOM_H
#define GPS2ODOM_H

#include <vector>
#include "ros/ros.h"
#include "ros/time.h"

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

#include "../../../../avoslib/geotool.h"
#include "../../../loclib/common.h"
#include "../../../loclib/loctool.h"
#include "ivlocmsg/ivmsglocpos.h"
#include "ivlocmsg/ivsensorgps.h"
#include "ivlocmsg/ivsensorimu.h"

class gps2odom {
 public:
  gps2odom(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~gps2odom(){};
  void gpsCallback(const ivlocmsg::ivsensorgps::ConstPtr &msg);
  void poseCallback(const ivlocmsg::ivmsglocpos::ConstPtr &msg);
  void stdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void run();
  void init();
  void output(sPoint2d inputPose);
  void output();

  int loop_rate;
  bool initial_flag;

 private:
  loclib::loctool loc_tool;
  geotool geo_tool;
  bool gps_flag;
  bool pose_flag;
  bool sensor_imu_flag;
  sPose2Dd antenna_pose;
  geometry_msgs::PoseStamped get_std_pose;
  nav_msgs::Path gps_path, ndt_path;
  geometry_msgs::PoseStamped ndt_pose;
  sGps get_gps;
  sLoc get_pose;
  sImu get_sensor_imu;
  sGps map_start_point;
  double first_height;
  double previous_time;
  nav_msgs::Odometry odom;
  double gps_time_gap;

  int gpsOrPose;
  double origin_x;
  double origin_y;
  double start_angle;
  std::string pose_topic;

  ros::Subscriber sub_gps;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_stdpose;
  ros::Publisher pub_odom;
  ros::Publisher pub_gps_path;
  ros::Publisher pub_ndt_path;
};

#endif