#include "gps2odom.h"

gps2odom::gps2odom(ros::NodeHandle nh, ros::NodeHandle private_nh) {
  initial_flag = pose_flag = gps_flag = false;
  odom.header.seq = 0;
  get_sensor_imu.roll = 0;
  get_sensor_imu.pitch = 0;
  nh.getParam("antenna_angle", antenna_pose.angle);
  nh.getParam("antenna_x", antenna_pose.x);
  nh.getParam("antenna_y", antenna_pose.y);
  nh.getParam("move_origin_x", origin_x);
  nh.getParam("move_origin_y", origin_y);
  private_nh.getParam("gpsOrPose", gpsOrPose);
  private_nh.getParam("start_angle", start_angle);
  private_nh.getParam("loop_rate", loop_rate);
  private_nh.getParam("pose_topic", pose_topic);

  sub_gps = nh.subscribe("ivsensorgps", 10, &gps2odom::gpsCallback, this);
  sub_pose = nh.subscribe("ivlocpos", 10, &gps2odom::poseCallback, this);
  sub_stdpose = nh.subscribe(pose_topic, 10, &gps2odom::stdPoseCallback, this);
  pub_odom = nh.advertise<nav_msgs::Odometry>("Odom", 20);
  pub_gps_path = nh.advertise<nav_msgs::Path>("gpsPath", 20);
  pub_ndt_path = nh.advertise<nav_msgs::Path>("ndtPath", 20);
}

void gps2odom::init() {
  if (!initial_flag && gps_flag && 0 == gpsOrPose) {
    if (get_gps.lon < 50 && get_gps.lat < 5) return;

    first_height = get_gps.height;
    map_start_point = get_gps;
    map_start_point.heading = 90;
    geo_tool.initParam(map_start_point, 0.05);
    map_start_point.heading = get_gps.heading;
    initial_flag = true;
    previous_time = ros::Time::now().toSec();
  } else if (!initial_flag && pose_flag && 1 == gpsOrPose) {
    if (get_pose.lon < 50 && get_pose.lat < 5) return;
    map_start_point.lon = get_pose.lon;
    map_start_point.lat = get_pose.lat;
    map_start_point.heading = 90;
    first_height = get_pose.xg;
    initial_flag = true;
    geo_tool.initParam(map_start_point, 0.05);
    previous_time = ros::Time::now().toSec();
  } else {
    return;
  }
}

void gps2odom::run() {
  if (!initial_flag) {
    return;
  } else {
    sGps new_gps;
    sPoint2d temp_pose;
    if (0 == gpsOrPose) {
      new_gps = loc_tool.getGps(get_gps, antenna_pose, get_gps.height);
      temp_pose = geo_tool.BLH2XYZ(new_gps.lat, new_gps.lon, get_gps.height);
    } else {
      sGps temp_gps;
      temp_gps.lon = get_pose.lon;
      temp_gps.lat = get_pose.lat;
      temp_gps.heading = get_pose.heading;
      new_gps = loc_tool.getGps(temp_gps, antenna_pose, get_pose.xg);
      temp_pose = geo_tool.BLH2XYZ(new_gps.lat, new_gps.lon, get_pose.xg);
    }
    output(temp_pose);
  }
}

void gps2odom::output(sPoint2d inputPose) {
  odom.pose.pose.position.x = inputPose.x - origin_x;
  odom.pose.pose.position.y = inputPose.y - origin_y;
  double odom_heading;
  if (0 == gpsOrPose)
    odom_heading = 90 - get_gps.heading - antenna_pose.angle;
  else if (1 == gpsOrPose)
    odom_heading = 90 - get_pose.heading - antenna_pose.angle;
  if (odom_heading > 180)
    odom_heading -= 360;
  else if (odom_heading < -180)
    odom_heading += 360;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      geo_tool.degreeToRadian(get_sensor_imu.roll),
      geo_tool.degreeToRadian(get_sensor_imu.pitch),
      geo_tool.degreeToRadian(odom_heading));
  output();
}

void gps2odom::output() {
  if (0 == gpsOrPose) {
    odom.pose.pose.position.z = get_gps.height - first_height;
  } else if (1 == gpsOrPose) {
    // odom.pose.pose.position.z = get_pose.xg - first_height;
    odom.pose.pose.position.z = 0;
  }

  geometry_msgs::Vector3 linear_velocity, angular_velocity;
  double delta_heading;
  if (0 == gpsOrPose) {
    delta_heading = get_gps.heading - map_start_point.heading;
    if (delta_heading > 330)
      delta_heading -= 360;
    else if (delta_heading < -330)
      delta_heading += 360;
    linear_velocity.x =
        get_gps.velocity * cos(delta_heading * MPI / 180.0) / 3.6;
    linear_velocity.y =
        -get_gps.velocity * sin(delta_heading * MPI / 180.0) / 3.6;
    linear_velocity.z = get_gps.status;
  } else if (1 == gpsOrPose) {
    delta_heading = get_pose.heading - map_start_point.heading;
    if (delta_heading > 330)
      delta_heading -= 360;
    else if (delta_heading < -330)
      delta_heading += 360;
    linear_velocity.x =
        get_pose.velocity * cos(delta_heading * MPI / 180.0) / 3.6;
    linear_velocity.y =
        -get_pose.velocity * sin(delta_heading * MPI / 180.0) / 3.6;
    linear_velocity.z = get_pose.yg;
  }
  angular_velocity.x = 0;
  angular_velocity.y = 0;
  double gps_time_gap = ros::Time::now().toSec() - previous_time;
  previous_time = ros::Time::now().toSec();
  angular_velocity.z = -delta_heading * MPI / (180 * gps_time_gap);
  odom.twist.twist.linear = linear_velocity;
  odom.twist.twist.angular = angular_velocity;
  odom.header.frame_id = "map";
  odom.header.stamp = ros::Time::now();
  odom.header.seq += 1;
  pub_odom.publish(odom);
  geometry_msgs::PoseStamped gps_pose;
  gps_pose.pose.position.x = odom.pose.pose.position.x;
  gps_pose.pose.position.y = odom.pose.pose.position.y;
  gps_pose.pose.position.z = odom.pose.pose.position.z;
  gps_pose.pose.orientation = odom.pose.pose.orientation;
  gps_path.header.frame_id = "map";
  gps_path.header.stamp = gps_pose.header.stamp = ros::Time::now();
  gps_path.poses.push_back(gps_pose);
  pub_gps_path.publish(gps_path);
}

void gps2odom::stdPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  get_std_pose = *msg;
  ndt_path.header.frame_id = "map";
  ndt_path.header.stamp = get_std_pose.header.stamp;
  ndt_path.poses.push_back(get_std_pose);
  pub_ndt_path.publish(ndt_path);
}

void gps2odom::poseCallback(const ivlocmsg::ivmsglocpos::ConstPtr &msg) {
  get_pose = *msg;
  pose_flag = true;
}

void gps2odom::gpsCallback(const ivlocmsg::ivsensorgps::ConstPtr &msg) {
  get_gps = *msg;
  gps_flag = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps2odom");
  ros::NodeHandle mh;
  ros::NodeHandle private_mh("~");
  gps2odom Gps2Odom(mh, private_mh);
  ros::Rate loop(Gps2Odom.loop_rate);
  while (ros::ok()) {
    ros::spinOnce();
    if (!Gps2Odom.initial_flag) Gps2Odom.init();
    Gps2Odom.run();
    loop.sleep();
  }
  return 0;
}