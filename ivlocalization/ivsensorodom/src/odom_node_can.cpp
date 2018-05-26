#include "odom_node_can.h"
#include <nav_msgs/Odometry.h>
#include "ivlocmsg/ivsensorodom.h"

OdomNodeCan::OdomNodeCan(ros::NodeHandle &nh)
{

    sub_wheel_vel = nh.subscribe("wheelspeed", 10,&OdomNodeCan::callbackWheelVel,this);
    pub_odom     = nh.advertise<ivlocmsg::ivsensorodom>("ivsensorodom",50);
    pub_ros_odom = nh.advertise<nav_msgs::Odometry>("ivstdodom",50);

    parameterInitial();
}

OdomNodeCan::~OdomNodeCan()
{
    //TODO
}

void OdomNodeCan::run()
{  
    ros::spin();
}

void OdomNodeCan::callbackWheelVel(const ivactuator::wheelspeed::ConstPtr& msg)
{
    angle_velocity_rl = msg->wheelspeed_lr;
    angle_velocity_rr = msg->wheelspeed_rr;

    odomVelocityComputeAndPublish();
    
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    odomComputeAndPublish(dt);
} 

void OdomNodeCan::odomVelocityComputeAndPublish()
{
    ROS_INFO("angle_rl:%f",angle_velocity_rl);
    ROS_INFO("angle_rr:%f\n",angle_velocity_rr);
    aOdometer.SetObservation(angle_velocity_fl, angle_velocity_fr, angle_velocity_rl, angle_velocity_rr);
    aOdometer.ProcessAndUpdate();
    aOdometer.GetWheelVelocity(velocity_fl, velocity_fr, velocity_rl, velocity_rr);
    aOdometer.GetVehicleVelocity(velocity, yaw_rate);

    ivlocmsg::ivsensorodom msg_ivsensorodom;
    msg_ivsensorodom.header.frame_id = "odom";
    msg_ivsensorodom.header.stamp = ros::Time::now();
    msg_ivsensorodom.angular_velocity_fl = angle_velocity_fl;
    msg_ivsensorodom.angular_velocity_fr = angle_velocity_fr;
    msg_ivsensorodom.angular_velocity_rl = angle_velocity_rl;
    msg_ivsensorodom.angular_velocity_rr = angle_velocity_rr;
    msg_ivsensorodom.velocity_fl = velocity_fl;
    msg_ivsensorodom.velocity_fr = velocity_fr;
    msg_ivsensorodom.velocity_rl = velocity_rl;
    msg_ivsensorodom.velocity_rr = velocity_rr;
    msg_ivsensorodom.velocity = velocity;
    msg_ivsensorodom.yaw_rate = yaw_rate;
    msg_ivsensorodom.using_actuator = false;
    pub_odom.publish(msg_ivsensorodom);
}

void OdomNodeCan::odomComputeAndPublish(double dt)
{
    vx_ = velocity * cos(yaw_rate * dt);
    vy_ = velocity * sin(yaw_rate * dt);

    double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
    double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
    double delta_th = yaw_rate * dt;
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.header.frame_id = "odom";

    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    msg_odom.pose.pose.position.x = x_;
    msg_odom.pose.pose.position.y = y_;
    msg_odom.pose.pose.position.z = 0.0;
    msg_odom.pose.pose.orientation.x = q.x();
    msg_odom.pose.pose.orientation.y = q.y();
    msg_odom.pose.pose.orientation.z = q.z();
    msg_odom.pose.pose.orientation.w = q.w();

    msg_odom.child_frame_id = "base_link";
    msg_odom.twist.twist.linear.x = vx_;
    msg_odom.twist.twist.linear.y = vy_;
    msg_odom.twist.twist.angular.z = vth_;

    pub_ros_odom.publish(msg_odom);
}

void OdomNodeCan::parameterInitial()
{
    vehicle.TRACK_WIDTH = 0.49;
    vehicle.WHEEL_RADIUS_FORWARD_L = 0.166;
    vehicle.WHEEL_RADIUS_FORWARD_R = 0.164;
    vehicle.WHEEL_RADIUS_REAR_L = 0.158;
    vehicle.WHEEL_RADIUS_REAR_R = 0.162;
    vehicle.WHEEL_BASE = 0.8;
    aOdometer.SetVehicleModel(vehicle);

    velocity_fl = 0.0;
    velocity_fr = 0.0;
    velocity_rl = 0.0;
    velocity_rr = 0.0;
    angle_velocity_fl = 0.0;
    angle_velocity_fr = 0.0;
    angle_velocity_rl = 0.0;
    angle_velocity_rr = 0.0;
    velocity = 0.0;
    yaw_rate = 0.0;

    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    vth_ = 0.0;
    current_time = ros::Time::now();
    last_time    = ros::Time::now();
}
