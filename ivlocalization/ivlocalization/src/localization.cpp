#include "localization.h"

localization::localization(ros::NodeHandle nh) {
  sub_odom =
      nh.subscribe("ivsensorodom", 10, &localization::callback_odom, this);
  sub_pose = nh.subscribe("ndt_pose", 10, &localization::callback_pose, this);
  sub_gps = nh.subscribe("ivsensorgps", 10, &localization::callback_gps, this);
  sub_status =
      nh.subscribe("ndt_status", 10, &localization::callback_status, this);
  sub_map = nh.subscribe("points_map", 10, &localization::callback_map, this);
  sub_initialpose = nh.subscribe("initialpose", 10,
                                 &localization::callback_initialpose, this);
  pub_initialpose =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 20);
  pub_locpos = nh.advertise<ivlocmsg::ivmsglocpos>("ivlocposfusion", 20);
  pub_locstdpos = nh.advertise<geometry_msgs::PoseStamped>("ivlocstdpos", 20);

  // init map
  iabasemapTL.heading = 90;
  iabasemapTL.lon = INVALID_VALUE;
  iabasemapTL.lat = INVALID_VALUE;

  nh.param("gicscellsize", cellSize, cellSize);
  nh.param("iabasemaptllon", iabasemapTL.lon, iabasemapTL.lon);
  nh.param("iabasemaptllat", iabasemapTL.lat, iabasemapTL.lat);
  nh.param("loc_type", loc_type_, loc_type_);
  nh.param("MAX_X", max_x, max_x);
  nh.param("MAX_Y", max_y, max_y);
  nh.param("MIN_X", min_x, min_x);
  nh.param("MIN_Y", min_y, min_y);
  nh.param("ROTATE_IMG", rotate_img, rotate_img);
  nh.param("MAP_ANGLE", map_angle, map_angle);
  nh.param("IF_SAVE_PATH", save_path_, save_path_);
  nh.param("FILE_PATH", file_path, file_path);
  nh.param("score_limit", score_limit_, score_limit_);
  nh.param("antenna_angle", antenna_pose.angle, antenna_pose.angle);
  nh.param("antenna_x", antenna_pose.x, antenna_pose.x);
  nh.param("antenna_y", antenna_pose.y, antenna_pose.y);
  nh.param("move_origin_x", origin_x, origin_x);
  nh.param("move_origin_y", origin_y, origin_y);

  loc_tool.init(-antenna_pose.x);
  geo_tool.initParam(iabasemapTL, cellSize);

  gps_flag = pose_flag = false;
  gps_count = score_check_count = 0;
  status_flag = initial_flag = get_intialpose = false;
  map_flag = initial_ukf = false;

  previous_xg = 0;
  previous_yg = 0;
  previous_lidar_xg = previous_lidar_yg = 0;
  velocity_count = 0;
  last_velocity = 0;
  score_max = 1e32;

  monitor = new Monitor(16);
  timeGps = ros::Time::now().toSec();
  timeMatching = ros::Time::now().toSec();
  timeNow = ros::Time::now().toSec();
  timeCheck = ros::Time::now().toSec();
  previous_pub_time = ros::Time::now().toSec();

  if (save_path_) fileWritePose.open(file_path);
}

localization::~localization() {
  if (save_path_) fileWritePose.close();
  if (monitor) delete monitor;
}

void localization::run() {
  /********monitor**********/
  timeNow = ros::Time::now().toSec();
  if (fabs(timeNow - timeGps) > 0.5) {
    gps_flag = false;
    get_gps.status = 0;
    monitor->sendWarnning(2, 0);
  }
  if (4 != get_gps.status && !initial_flag && 1 == loc_type_)
    monitor->sendWarnning(2, 1);
  if (fabs(timeNow - timeMatching) > 1) {
    pose_flag = false;
    monitor->sendWarnning(2, 2);
  }

  /********pub pos**********/
  if (!initial_flag && gps_flag) pub_gps_pose();
  if (initial_flag && pose_flag) publish_tf(get_pose);
  if (pose_flag && 1 == loc_type_) pub_loc_pose();

  /********initialization**********/
  sPose3Dd temp_pose_fused;
  if (status_check(ndt_status) && pose_flag && 1 == loc_type_)
    initial_flag = true;
  if (4 == get_gps.status && !initial_flag && map_flag && 1 == loc_type_) {
    temp_pose_fused = loc_to_matching(loc_pos_gps);
    if (ros::Time::now().toSec() - previous_pub_time > 2) {
      pub_initial_pose(temp_pose_fused);
      previous_pub_time = ros::Time::now().toSec();
    }
  }
}

void localization::gps_to_loc() {
  //------calculate pos through gps---------------//
  sGps real_gps;
  real_gps = loc_tool.getGps(get_gps, antenna_pose, get_gps.height);
  // UTM
  sPoint2d temp_pose =
      geo_tool.BLH2XYZ(real_gps.lat, real_gps.lon, get_gps.height);
  loc_pos_gps.xg = temp_pose.x - min_x - origin_x;
  loc_pos_gps.yg = -temp_pose.y + max_y + origin_y;
  // origin
  /*sPointOfGCCS real_gps_pose = geo_tool.GPS2GCCS(iabasemapTL, real_gps);*/
  sPixelOfGICS car_gps_pixel =
      geo_tool.GCCS2GICS(loc_pos_gps.xg, loc_pos_gps.yg);

  double angle = get_gps.heading + antenna_pose.angle - 90;
  if (angle >= 180)
    angle -= 360;
  else if (angle < -180)
    angle += 360;

  loc_pos_gps.lon = real_gps.lon;
  loc_pos_gps.lat = real_gps.lat;
  loc_pos_gps.heading = get_gps.heading + antenna_pose.angle;
  if (loc_pos_gps.heading >= 360) loc_pos_gps.heading -= 360;
  if (loc_pos_gps.heading < 0) loc_pos_gps.heading += 360;
  // loc_pos_gps.xg = real_gps_pose.xg;
  // loc_pos_gps.yg = real_gps_pose.yg;
  loc_pos_gps.angle = angle;
  loc_pos_gps.ug = car_gps_pixel.ug;
  loc_pos_gps.vg = car_gps_pixel.vg;
  loc_pos_gps.velocity = get_gps.velocity;

  if (4 == get_gps.status) {
    loc_pos_gps.isvalid = true;
    gps_count++;
    if (gps_count > 3) gps_count = 3;
  } else {
    loc_pos_gps.isvalid = false;
    gps_count = 0;
  }
  if (3 == gps_count)
    status_flag = true;
  else
    status_flag = false;
}

void localization::gps_to_loc(sPoint2d input) {
  loc_pos_gps.lon = get_gps.lon;
  loc_pos_gps.lat = get_gps.lat;
  loc_pos_gps.heading = get_gps.heading + antenna_pose.angle;
  if (loc_pos_gps.heading >= 360) loc_pos_gps.heading -= 360;
  if (loc_pos_gps.heading < 0) loc_pos_gps.heading += 360;
  loc_pos_gps.xg = input.x - origin_x;
  loc_pos_gps.yg = input.y - origin_y;
  loc_pos_gps.angle = 90 - get_gps.heading - antenna_pose.angle;
  loc_pos_gps.velocity = get_gps.velocity;

  if (4 == get_gps.status) {
    loc_pos_gps.isvalid = true;
    status_flag = true;
  } else {
    loc_pos_gps.isvalid = false;
    status_flag = false;
  }
}

void localization::pub_gps_pose() {
  /*---------------save pos files----------------*/
  // pose_filter(loc_pos_gps);
  double dis_points = sqrt(pow(loc_pos_gps.xg - previous_xg, 2) +
                           pow(loc_pos_gps.yg - previous_yg, 2));
  bool dis_save_path_flag = false;
  if (dis_points > 0.05) dis_save_path_flag = true;

  if (0 == loc_type_ && save_path_ && dis_save_path_flag) {
    fileWritePose << setprecision(12) << loc_pos_gps.lon << ","
                  << loc_pos_gps.lat << "," << 0 << "," << loc_pos_gps.heading
                  << "," << 0 << "," << (int)get_gps.status << "," << 0 << ","
                  << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << ","
                  << 0 << "," << loc_pos_gps.xg << "," << loc_pos_gps.yg << ","
                  << 0 << "," << 0 << "," << loc_pos_gps.angle << endl;
    previous_xg = loc_pos_gps.xg;
    previous_yg = loc_pos_gps.yg;
  }

  /*---------------send transform----------------*/
  if (0 == loc_type_) publish_tf(loc_pos_gps);

  /*----------------add header-------------------*/
  loc_pos_gps.header.stamp = ros::Time::now();
  loc_pos_gps.header.frame_id = "map";

  /*----------------publish pos-------------------*/
  pub_locpos.publish(loc_pos_gps);
}

void localization::pub_loc_pose() {
  /*----------------transform ndt pos to loc pos-------------------*/
  double ndt_yaw, ndt_x, ndt_y, ndt_angle, ndt_dis;
  sPose3Dd pose_fused = pose_fusion();
  // loc_pos = matching_to_loc2(pose_fused);
  loc_pos = matching_to_loc(pose_fused);
  // pose_filter(loc_pos);

  if (loc_pos_gps.isvalid) {
    loc_pos.lon = loc_pos_gps.lon;
    loc_pos.lat = loc_pos_gps.lat;
  } else {
    double temp_bearing =
        180 - (atan2(loc_pos.xg, loc_pos.yg) * 180.0 / M_PI - map_angle);
    double temp_dist = sqrt(pow(loc_pos.xg, 2) + pow(loc_pos.yg, 2));
    sGps temp_gps =
        geo_tool.getGpsFromGpsBearDis(iabasemapTL, temp_bearing, temp_dist);
    loc_pos.lon = temp_gps.lon;
    loc_pos.lat = temp_gps.lat;
  }

  /*----------------save pos files-------------------*/
  double check_dis = sqrt(pow(loc_pos.xg - previous_lidar_xg, 2) +
                          pow(loc_pos.yg - previous_lidar_yg, 2));
  if (save_path_ && check_dis > 0.15) {
    fileWritePose << setprecision(12) << loc_pos.lon << "," << loc_pos.lat
                  << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0
                  << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0
                  << "," << 0 << "," << loc_pos.xg << "," << loc_pos.yg << ","
                  << 0 << "," << 0 << "," << loc_pos.angle << endl;
    /*----------------update previous pos-------------------*/
    previous_lidar_xg = loc_pos.xg;
    previous_lidar_yg = loc_pos.yg;
  }

  /*----------------add header-------------------*/
  loc_pos.header.stamp = ros::Time::now();
  loc_pos.header.frame_id = "map";

  /*----------------update isvalid-------------------*/
  if (status_check(ndt_status))
    loc_pos.isvalid = true;
  else
    loc_pos.isvalid = false;

  /*----------------publish pos-------------------*/
  if (initial_flag && status_check(ndt_status))
    pub_locpos.publish(loc_pos);
  else {
    monitor->sendWarnning(2, 3);
    initial_flag = false;
  }

  calculate_speed();
}

bool localization::status_check(sStatus status_input) {
  if (!get_intialpose) return false;
  double temp_delta_dist = sqrt(pow(loc_pos_gps.xg - loc_pos.xg, 2) +
                                pow(loc_pos_gps.yg - loc_pos.yg, 2));
  if (gps_flag && 4 == get_gps.status && temp_delta_dist > 8) {
    return false;
  } else if (status_input.velocity > 3 && status_input.score > 5) {
    if (ros::Time::now().toSec() - timeCheck < 0.3)
      ++score_check_count;
    else
      score_check_count = 0;
    timeCheck = ros::Time::now().toSec();
    if (score_check_count > 5) {
      score_check_count = 0;
      return false;
    } else
      return true;
  } else
    return true;
}

void localization::publish_tf(sLoc loc_pos_input) {
  sPose3Dd temp_pose = loc_to_matching(loc_pos_input);
  tf::Transform temp_transform;
  temp_transform.setOrigin(tf::Vector3(temp_pose.x, temp_pose.y, 0.0));
  tf::Quaternion quaternion;
  quaternion.setRPY(0, 0, temp_pose.yaw);
  temp_transform.setRotation(quaternion);
  // pub_tf.sendTransform(tf::StampedTransform(temp_transform, ros::Time::now(),
  // "map", "base_link"));
  pub_tf.sendTransform(tf::StampedTransform(
      temp_transform, loc_pos_input.header.stamp, "map", "base_link"));
}

void localization::publish_tf(geometry_msgs::PoseStamped loc_pos_input) {
  tf::Transform temp_transform;
  temp_transform.setOrigin(tf::Vector3(loc_pos_input.pose.position.x,
                                       loc_pos_input.pose.position.y,
                                       loc_pos_input.pose.position.z));
  tf::Quaternion quaternion(
      loc_pos_input.pose.orientation.x, loc_pos_input.pose.orientation.y,
      loc_pos_input.pose.orientation.z, loc_pos_input.pose.orientation.w);
  temp_transform.setRotation(quaternion);
  pub_tf.sendTransform(tf::StampedTransform(
      temp_transform, loc_pos_input.header.stamp, "map", "base_link"));
}

/*------------------------------------------------------------------------------*/
/*-------------------------calculate
 * velocity-----------------------------------*/
/*------------------------------------------------------------------------------*/
void localization::calculate_speed() {
  if (odom_flag)
    loc_pos.velocity = get_odom.velocity;
  else {
    velocity_count++;
    if (velocity_count > 5) velocity_count = 0;
    if (last_time > 0) {
      double delta_time, dist;
      if (fabs(last_xg) > 0.0001 && fabs(last_yg) > 0.0001) {
        delta_time = ros::Time::now().toSec() - last_time;
        dist =
            sqrt(pow(loc_pos.xg - last_xg, 2) + pow(loc_pos.yg - last_yg, 2));
      }
      if (0 == velocity_count && delta_time > 0)
        last_velocity = dist / delta_time;
      loc_pos.velocity = last_velocity;

      last_time = ros::Time::now().toSec();
      last_xg = loc_pos.xg;
      last_yg = loc_pos.yg;
    }
  }
}

/*------------------------------------------------------------------------------*/
/*-------------Transform from lidar matching frame to AVOS
 * frame----------------*/
/*------------------------------------------------------------------------------*/
sLoc localization::matching_to_loc(sPose3Dd temp_pose) {
  sLoc temp_loc_pos;
  if (-1 == rotate_img) {
    temp_loc_pos.xg = temp_pose.x - min_x;
    temp_loc_pos.yg = -temp_pose.y + max_y;
    temp_loc_pos.angle = -geo_tool.radianToDegree(temp_pose.yaw);
  } else if (0 == rotate_img) {
    // if(0 == originOrUTM){
    //   temp_loc_pos.xg = - temp_pose.y + max_y;
    //   temp_loc_pos.yg = - temp_pose.x + max_x;
    //   temp_loc_pos.angle = - geo_tool.radianToDegree(temp_pose.yaw) - 90;
    // }else if(1 == originOrUTM){
    temp_loc_pos.xg = temp_pose.x - min_x;
    temp_loc_pos.yg = -temp_pose.y + max_y;
    temp_loc_pos.angle = -geo_tool.radianToDegree(temp_pose.yaw);
    //}
  } else if (1 == rotate_img) {
    temp_loc_pos.xg = -temp_pose.x + max_x;
    temp_loc_pos.yg = temp_pose.y - min_y;
    temp_loc_pos.angle = -geo_tool.radianToDegree(temp_pose.yaw) - 180;
  } else if (2 == rotate_img) {
    temp_loc_pos.xg = temp_pose.y - min_y;
    temp_loc_pos.yg = temp_pose.x - min_x;
    temp_loc_pos.angle = 90 - geo_tool.radianToDegree(temp_pose.yaw);
  }
  if (temp_loc_pos.angle > 180)
    temp_loc_pos.angle -= 360;
  else if (temp_loc_pos.angle < -180)
    temp_loc_pos.angle += 360;
  return temp_loc_pos;
}

/*------------------------------------------------------------------------------*/
/*-------------Transform from AVOS frame to lidar matching
 * frame----------------*/
/*------------------------------------------------------------------------------*/
sPose3Dd localization::loc_to_matching(sLoc temp_loc_pos) {
  sPose3Dd temp_pose;
  if (-1 == rotate_img) {
    temp_pose.x = temp_loc_pos.xg + min_x;
    temp_pose.y = -temp_loc_pos.yg + max_y;
    temp_pose.yaw = -geo_tool.degreeToRadian(temp_loc_pos.angle);
  } else if (0 == rotate_img) {
    temp_pose.x = temp_loc_pos.xg + min_x;
    temp_pose.y = -temp_loc_pos.yg + max_y;
    temp_pose.yaw = -geo_tool.degreeToRadian(temp_loc_pos.angle);
  } else if (1 == rotate_img) {
    temp_pose.x = -temp_loc_pos.xg + max_x;
    temp_pose.y = temp_loc_pos.yg + min_y;
    temp_pose.yaw = -M_PI - geo_tool.degreeToRadian(temp_loc_pos.angle);
  } else if (2 == rotate_img) {
    temp_pose.x = temp_loc_pos.yg + min_x;
    temp_pose.y = temp_loc_pos.xg + min_y;
    temp_pose.yaw = M_PI / 2.0 - geo_tool.degreeToRadian(temp_loc_pos.angle);
  }
  return temp_pose;
}

sLoc localization::matching_to_loc2(sPose3Dd temp_pose) {
  sLoc temp_loc_pos;
  temp_loc_pos.xg = temp_pose.x;
  temp_loc_pos.yg = temp_pose.y;
  temp_loc_pos.angle = geo_tool.radianToDegree(temp_pose.yaw);
  return temp_loc_pos;
}

sPose3Dd localization::pose_fusion() {
  if (pose_flag) {
    sPose3Dd temp_pose_fused;
    tfScalar tf_roll, tf_pitch, tf_yaw;
    tf::Quaternion tf_quaternion;
    quaternionMsgToTF(get_pose.pose.orientation, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(tf_roll, tf_pitch, tf_yaw);

    temp_pose_fused.x = get_pose.pose.position.x;
    temp_pose_fused.y = get_pose.pose.position.y;
    temp_pose_fused.z = get_pose.pose.position.z;
    temp_pose_fused.roll = tf_roll;
    temp_pose_fused.pitch = tf_pitch;
    temp_pose_fused.yaw = tf_yaw;
    return temp_pose_fused;
  }
}

void localization::pub_std_pose(sPose3Dd pose_input) {
  geometry_msgs::PoseStamped std_pose;
  std_pose.header.stamp = ros::Time::now();
  std_pose.header.frame_id = "map";
  std_pose.pose.position.x = pose_input.x;
  std_pose.pose.position.y = pose_input.y;
  std_pose.pose.position.z = pose_input.z;
  std_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      pose_input.roll, pose_input.pitch, pose_input.yaw);
  pub_locstdpos.publish(std_pose);
}

void localization::pub_initial_pose(sPose3Dd pose_input) {
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.stamp = ros::Time::now();
  initial_pose.pose.pose.position.x = pose_input.x;
  initial_pose.pose.pose.position.y = pose_input.y;
  initial_pose.pose.pose.position.z = pose_input.z;
  initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      pose_input.roll, pose_input.pitch, pose_input.yaw);
  pub_initialpose.publish(initial_pose);
}

void localization::pose_filter(sLoc &loc_pos_input) {
  if (odom_flag) {
    sPose3Dd temp_pose_fused;
    if (!initial_ukf) {
      temp_pose_fused = loc_to_matching(loc_pos_input);
      locState.x() = temp_pose_fused.x;
      locState.y() = temp_pose_fused.y;
      locState.vx() = 0;
      locState.vy() = 0;
      locState.ax() = 0;
      locState.ay() = 0;
      locState.yaw() = temp_pose_fused.yaw;
      locState.yawrate() = 0;
      ukf.init(locState);
      initial_ukf = true;
    }
    locState = locSys.f(locState, locU);
    auto x_ukf = ukf.predict(locSys, locU);

    temp_pose_fused = loc_to_matching(loc_pos_input);
    Measurement locZ;
    locZ.z_x() = temp_pose_fused.x;
    locZ.z_y() = temp_pose_fused.y;
    locZ.z_v() = get_odom.velocity;
    locZ.z_yaw() = temp_pose_fused.yaw;
    locZ.z_yawrate() = get_odom.yaw_rate;
    x_ukf = ukf.update(locMeasurementModel, locZ);

    temp_pose_fused.x = x_ukf.x();
    temp_pose_fused.y = x_ukf.y();
    temp_pose_fused.yaw = x_ukf.yaw();
    loc_pos_input = matching_to_loc(temp_pose_fused);
    loc_pos_input.velocity = sqrt(pow(x_ukf.vx(), 2) + pow(x_ukf.vy(), 2));
    loc_pos_input.yaw_rate = x_ukf.yawrate();
    pub_std_pose(loc_to_matching(loc_pos_input));
  } else {
    // doing nothing
  }
}

void localization::callback_odom(const ivlocmsg::ivsensorodom::ConstPtr &msg) {
  get_odom = *msg;
  odom_flag = true;
}

void localization::callback_map(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  map_flag = true;
}

void localization::callback_status(const ivlocmsg::ndt_status::ConstPtr &msg) {
  ndt_status = *msg;
}

void localization::callback_pose(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  timeMatching = ros::Time::now().toSec();
  get_pose = *msg;
  if (get_pose.pose.position.x == 0 && get_pose.pose.position.y == 0) {
    ROS_ERROR("THE CALLBACK NDT POSE IS ERROR");
    return;
  } else {
    pose_flag = true;
  }
}

void localization::callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  get_intialpose = true;
}

void localization::callback_gps(const ivlocmsg::ivsensorgps::ConstPtr &msg) {
  timeGps = ros::Time::now().toSec();
  get_gps = *msg;
  gps_flag = true;

  if (get_gps.lon < 50 || get_gps.lat < 10 || get_gps.velocity > 200) {
    get_gps.status = 0;

    ROS_ERROR("GPS IS ERROR!!!!");
    return;
  }

  gps_to_loc();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ivlocalization");
  ros::NodeHandle nh;
  localization data(nh);
  ros::Rate loop(10);
  while (ros::ok()) {
    ros::spinOnce();
    data.run();
    loop.sleep();
  }
  return 0;
}
