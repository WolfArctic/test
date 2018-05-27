#include "search_info_ros.h"

SearchInfo::SearchInfo()
  : map_set_(false)
  , start_set_(false)
  , goal_set_(false)
{
}

SearchInfo::~SearchInfo()
{
}

void SearchInfo::mapCallback(ivpredict::ivmsgpredict msg)
{
  map_ = msg;
  map_set_ = true;
}


void SearchInfo::startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed start pose!");

  // start_pose_local_.header = msg->header;
  // start_pose_local_.pose    = msg->pose.pose;
  // start_pose_local_.pose.position.x = 5.0 + start_pose_local_.pose.position.x;
  // start_pose_local_.pose.position.y = 5.0 + start_pose_local_.pose.position.y;

  start_pose_local_.header = msg->header;
  start_pose_local_.pose    = msg->pose.pose;
  start_pose_local_.pose.position.x = 5.0;
  start_pose_local_.pose.position.y = 5.0;

  start_set_ = true;
}




void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed goal pose!");

  goal_pose_local_.header = msg->header;
  goal_pose_local_.pose    = msg->pose;
  goal_pose_local_.pose.position.x = 5.0 + goal_pose_local_.pose.position.x;
  goal_pose_local_.pose.position.y = 5.0 + goal_pose_local_.pose.position.y;

  goal_set_ = true;
}

void SearchInfo::reset()
{
  map_set_   = false;
  start_set_ = false;
  goal_set_  = false;
}
