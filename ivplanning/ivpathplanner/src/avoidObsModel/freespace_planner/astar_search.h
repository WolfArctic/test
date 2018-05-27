#ifndef ASTAR_NAVI_NODE_H
#define ASTAR_NAVI_NODE_H

#define DEBUG 1

#include "astar_util.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

#include "ivpredict/ivmsgpredict.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivpathplanner/path.h"

class AstarSearch
{
 public:
  AstarSearch();
  ~AstarSearch();

  //-- FOR DEBUG -------------------------
  void publishPoseArray(const ros::Publisher &pub, const std::string &frame);
  geometry_msgs::PoseArray debug_pose_array_;
  //--------------------------------------

  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const ivpredict::ivmsgpredict &map);
  void reset();
  nav_msgs::Path getPath() {return path_;}

  ivpathplanner::path pathinterface(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current, ivpathplanner::path raw, ivpredict::ivmsgpredict objs);

 private:
  bool search();
  void resizeNode(int width, int height, int angle_size);
  void createStateUpdateTable(int angle_size);
  void createStateUpdateTableLocal(int angle_size); //
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);
  void setMap(const ivpredict::ivmsgpredict &map);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool detectCollision(const SimpleNode &sn);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);
  

  // ROS param
  int angle_size_;                // descritized angle size
  int map_height;
  int map_width;

  double minimum_turning_radius_; // varying by vehicles
  double goal_radius_;            // meter
  double goal_angle_;             // degree
  double map_resolution;
  double resolution;
  double robot_length_;
  double robot_width_;
  double base2back_;
  double curve_weight_;
  double reverse_weight_;

  bool use_back_;                 // use backward driving
  bool use_wavefront_heuristic_;
  bool node_initialized_;

  std::vector<std::vector<NodeUpdate>> state_update_table_;
  nav_msgs::MapMetaData map_info_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  // Transform which converts OccupancyGrid frame to global frame
  tf::Transform map2ogm_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Searched path
  nav_msgs::Path path_;

};

#endif // ASTAR_NAVI_NODE_H
