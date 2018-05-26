/*
 * Author: Anjisi
 * and your big zhu brother
 */
#ifndef __ODOM_NODE_CAN_H
#define __ODOM_NODE_CAN_H

#include <cstdio>
#include <cstdlib>

#include "aodom.h"
#include <iostream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "ivactuator/wheelspeed.h"
#include "../../../avoslib/globalvariables.h"

using namespace std;

class OdomNodeCan
{
public:
    OdomNodeCan(ros::NodeHandle &nh);
    ~OdomNodeCan();
    void run();
    
public:
    float angle_velocity_fl, angle_velocity_fr;
    float angle_velocity_rl, angle_velocity_rr;
    float velocity_fl, velocity_fr, velocity, yaw_rate;
    float velocity_rl, velocity_rr;
    double x_, y_, th_, vx_, vy_, vth_;

    Vehicle_Model vehicle;
    AOdometer aOdometer;

private:
    void callbackWheelVel(const ivactuator::wheelspeed::ConstPtr& msg);
    void odomVelocityComputeAndPublish();
    void odomComputeAndPublish(double dt);
    void parameterInitial();
private:
    ros::Publisher pub_odom;
    ros::Publisher pub_ros_odom;
    ros::Subscriber sub_wheel_vel;
    ros::Time current_time;
    ros::Time last_time;
};

#endif //__ODOM_NODE_CAN_H
