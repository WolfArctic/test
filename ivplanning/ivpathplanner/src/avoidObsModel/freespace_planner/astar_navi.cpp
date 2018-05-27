#include "astar_search.h"
#include "search_info_ros.h"

#include "ivpredict/ivmsgpredict.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar_navi");
  ros::NodeHandle n;
  ros::NodeHandle private_nh_("~");

  double waypoint_velocity_kmph;
  private_nh_.param<double>("waypoint_velocity_kmph", waypoint_velocity_kmph, 5.0);

  AstarSearch astar;
  SearchInfo search_info;

  ivpredict::ivmsgpredict predictmsg;
  ivpredict::predictobj objs;

  ivpredict::mapobjcell objcell;
  objcell.xc = 2.0 / 0.05;
  objcell.yc = 0.5 / 0.05;
  objs.cell.push_back(objcell);

  predictmsg.objects.push_back(objs);

  search_info.mapCallback(predictmsg);

  // ROS subscribers
  ros::Subscriber start_sub = n.subscribe("/initialpose", 1, &SearchInfo::startCallback, &search_info);
  ros::Subscriber goal_sub  = n.subscribe("/move_base_simple/goal", 1, &SearchInfo::goalCallback, &search_info);

  // ROS publishers
  ros::Publisher path_pub       = n.advertise<nav_msgs::Path>("astar_path", 1, true);
  ros::Publisher debug_pose_pub = n.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1, true);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    if (!search_info.getMapSet() || !search_info.getStartSet() || !search_info.getGoalSet()) {
      loop_rate.sleep();
      continue;
    }

    auto start = std::chrono::system_clock::now();

    // Execute astar search
    bool result = astar.makePlan(search_info.getStartPose().pose, search_info.getGoalPose().pose, search_info.getMap());

    auto end = std::chrono::system_clock::now();
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    //std::cout << "astar msec: " << usec / 1000.0 << std::endl;
    ROS_INFO("astar msec: %lf", usec / 1000.0);

    if(result) {
      ROS_INFO("Found GOAL!");

#if DEBUG
      astar.publishPoseArray(debug_pose_pub, "/map");
      path_pub.publish(astar.getPath());
#endif

    } else {
      ROS_INFO("can't find goal...");

#if DEBUG
      astar.publishPoseArray(debug_pose_pub, "/map"); // debug
      path_pub.publish(astar.getPath());
#endif

    }

    // Reset flag
    search_info.reset();
    astar.reset();

    loop_rate.sleep();
  }

  return 0;
}
