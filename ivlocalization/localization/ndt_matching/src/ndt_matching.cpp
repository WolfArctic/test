#include "ndt_matching.h"

ndt_matching::ndt_matching(ros::NodeHandle nh, ros::NodeHandle private_nh) {
  init_params();
  private_nh.getParam("get_height", _get_height);
  private_nh.getParam("use_odom", _use_odom);
  private_nh.getParam("input_x", input_x_);
  private_nh.getParam("input_y", input_y_);
  private_nh.getParam("input_z", input_z_);
  private_nh.getParam("input_roll", input_roll_);
  private_nh.getParam("input_pitch", input_pitch_);
  private_nh.getParam("input_yaw", input_yaw_);
  private_nh.getParam("input_res", input_res_);
  private_nh.getParam("input_step_size", input_step_size_);
  private_nh.getParam("input_trans_eps", input_trans_eps_);
  private_nh.getParam("input_max_iter", input_max_iter_);
  private_nh.getParam("input_threshold", input_threshold_);
  private_nh.getParam("leaf_size", voxel_leaf_size);
  private_nh.getParam("min_scan_range", min_scan_range);
  private_nh.getParam("max_scan_range", max_scan_range);
  private_nh.getParam("min_scan_z", min_scan_z);
  private_nh.getParam("enable_syn_flag", enable_syn_flag);

  nh.getParam("points_topic", POINTS_TOPIC);

  // Setting parameters
  ndt.setResolution(input_res_);
  ndt.setStepSize(input_step_size_);
  ndt.setTransformationEpsilon(input_trans_eps_);
  ndt.setMaximumIterations(input_max_iter_);

  // Updated in initialpose_callback or gnss_callback
  initial_pose = Eigen::Matrix4f::Identity();

  // Publishers
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1000);
  ndt_stat_pub = nh.advertise<ivlocmsg::ndt_status>("ndt_status", 1000);
  adj_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("adjacent_pose", 100);

  // Subscribers
  map_sub = nh.subscribe("points_map", 10, &ndt_matching::mapCallback, this);
  initialpose_sub = nh.subscribe("initialpose", 1000,
                                 &ndt_matching::initialposeCallback, this);
  points_sub =
      nh.subscribe(POINTS_TOPIC, 1, &ndt_matching::pointsCallback, this);
  bbx_sub = nh.subscribe("ivsensorlidar", 1, &ndt_matching::bbxCallback, this);
  odom_sub = nh.subscribe("Odom", 1, &ndt_matching::odomCallback, this);
}

void ndt_matching::init_params() {
  previous_pose = Eigen::Matrix4f::Identity();
  guess_pose = Eigen::Matrix4f::Identity();
  guess_pose_odom = Eigen::Matrix4f::Identity();
  current_pose = Eigen::Matrix4f::Identity();
  delta_pose = Eigen::Matrix4f::Identity();

  // Default values
  max_iter = 30;
  ndt_res = 1.0;
  step_size = 0.1;
  trans_eps = 0.01;
  exe_time = 0.0;
  iteration = 0;
  fitness_score = 0.0;
  trans_probability = 0.0;

  _use_odom = false;
  _get_height = false;
  initialFlag = false;
  map_loaded = false;
  initial_pose_flag = false;
  odomFlag = false;
  min_scan_range = 2;
  max_scan_range = 50;
  min_scan_z = 2.0;
  //current_map_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  last_map_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

inline int ndt_matching::FastRand() {
  fast_rand_seed = (214013 * fast_rand_seed + 2531011);
  return (fast_rand_seed >> 16) & 0x7FFF;
}

void ndt_matching::removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_sample_pc_ptr =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  fast_rand_seed = (int)time(NULL);
  size_t input_size = input->size();
  for (size_t i = 0; i < input_size; ++i) {
    pcl::PointXYZ p = input->points[i];
    if (p.x > -30 && p.x < 30 && p.y > -8 && p.y < 8 &&
        i % (FastRand() % 10 + 1) == 0) {
      plane_sample_pc_ptr->push_back(p);
    }
  }
  igeometry2<pcl::PointXYZ> geo;
  sPlaneEquationf splane = geo.planeFitting(plane_sample_pc_ptr, 0.1);
  for (size_t i = 0; i < input_size; i++) {
    pcl::PointXYZ p = input->points[i];
    if (splane.c != 0) {
      double z_ground = -1 * (splane.a * p.x + splane.b * p.y + 1) / splane.c;
      if (p.z - z_ground > 0.3 &&
          sqrt(pow(p.x, 2.0) + pow(p.y, 2.0)) > min_scan_range &&
          sqrt(pow(p.x, 2.0) + pow(p.y, 2.0)) < max_scan_range) {
        output->push_back(p);
      }
    } else if (splane.c == 0) {
      output = input;
    }
  }
}

void ndt_matching::setGuessPose(Eigen::Matrix4f& output) {
  if (initialFlag)
    output = guess_pose = previous_pose * delta_pose;
  else
    output = guess_pose = previous_pose;
  if (_use_odom) odomCalculate(current_scan_time);
  if (_use_odom && odomFlag) {
    output = guess_pose_odom;
  } else
  {
    output = guess_pose;
}
}

bool ndt_matching::setInput(const sensor_msgs::PointCloud2::ConstPtr& input) {
  if ((enable_syn_flag || _use_odom) && odom_deq.size() < 1) return false;
  current_scan_time = ndt_stat_msg.header.stamp = input->header.stamp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_without_ground_ptr =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_prior =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input, *scan_ptr);

  if(scan_ptr->size() < 1){
  	ROS_INFO("The input cloud is empty!!!");
  	return false;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size,
                                voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  removeGround(filtered_scan_ptr, pc_without_ground_ptr);
  double radius_from_lidar;
  pcl::PointXYZ p;
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item =
           pc_without_ground_ptr->begin();
       item != pc_without_ground_ptr->end(); ++ item) {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    radius_from_lidar = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (radius_from_lidar > min_scan_range &&
        radius_from_lidar < max_scan_range && p.z > min_scan_z) {
      bool bbx_flag = true;
      for (unsigned int i = 0; i < get_bbx.objs.size(); ++i) {
        if (fabs(p.x - get_bbx.objs[i].x) < get_bbx.objs[i].length &&
            fabs(p.y - get_bbx.objs[i].y) < get_bbx.objs[i].width) {
          bbx_flag = false;
          break;
        }
      }
      if (bbx_flag) {
        output_prior->push_back(p);
      }
    }
  }

  //current_map_ptr = output_prior;
  // last_map_ptr=output_prior;
  if(output_prior -> size() > 1){
  ndt.setInputSource(output_prior);
  return true;
  } else{
  	return false;
  }
}

void ndt_matching::ndtRegistration(Eigen::Matrix4f input_guess,
                                   Eigen::Matrix4f& output) {
  std::chrono::time_point<std::chrono::system_clock> align_start, align_end;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud =boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  align_start = std::chrono::system_clock::now();
#ifndef USING_GPU_NDT_
  ndt.align(*output_cloud, input_guess);
#else
  ndt.align(input_guess);
#endif
  align_end = std::chrono::system_clock::now();
  align_time = std::chrono::duration_cast<std::chrono::microseconds>(
                   align_end - align_start).count() /1000.0;
  iteration = ndt.getFinalNumIteration();
  fitness_score = ndt.getFitnessScore();
  trans_probability = ndt.getTransformationProbability();
  // std::cout << "trans_probability = " << trans_probability << std::endl;
  output = ndt.getFinalTransformation();
  ndt_stat_msg.exe_time = align_time;
  ndt_stat_msg.iteration = iteration;
  ndt_stat_msg.score = fitness_score;
  if (trans_probability > 2) initialFlag = true;
}

void ndt_matching::publishOutput(Eigen::Matrix4f input) {
  current_pose = input;
  delta_pose = previous_pose.inverse() * current_pose;

  Eigen::Matrix4f guess_adj_points = delta_pose.inverse();

  // limit roll and pitch
  tf::Matrix3x3 mat_limit;
  transform_.EigenMatrixToTFMatrix(delta_pose, mat_limit);
  double temp_roll, temp_pitch, temp_yaw;
  mat_limit.getRPY(temp_roll, temp_pitch, temp_yaw, 1);
  mat_limit.setEulerYPR(temp_yaw, 0, 0);
  transform_.TFMatrixToEigenMatrix(mat_limit, delta_pose);

  adj_pose_msg.header.frame_id = "map";
  adj_pose_msg.header.stamp = current_scan_time;
  transform_.EigenToMsg(guess_adj_points, adj_pose_msg.pose);
  adj_pose_pub.publish(adj_pose_msg);

  previous_pose = current_pose;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = current_scan_time;
  transform_.EigenToMsg(input, ndt_pose_msg.pose);
  if(delta_pose(0,3) > 50 || delta_pose(1,3)>50)
    {
      ROS_ERROR("THE POSE CHANGING IS TOO LARGE!!!");
      return;
    }else{
  ndt_pose_pub.publish(ndt_pose_msg);
  ndt_stat_pub.publish(ndt_stat_msg);
  adj_pose_pub.publish(adj_pose_msg);
}
}

void ndt_matching::odomCalculate(ros::Time current_time) {
  transform_.MsgToEigen(odom.pose.pose, guess_pose_odom);
  if (enable_syn_flag) {
    if (odom_deq.size() < 1) {
      guess_pose_odom(0, 3) = 0;
      guess_pose_odom(1, 3) = 0;
    } else {
      int get_i = 0;
      int temp_i = odom_deq.size();
      if ((current_time - odom_deq[temp_i - 1].header.stamp).toSec() > 0) {
        // std::cout<<"too new, stamp error !!!"<<std::endl;
      } else if ((current_time - odom_deq[0].header.stamp).toSec() < 0) {
        // std::cout<<"too old, stamp error !!!"<<std::endl;
        transform_.MsgToEigen(odom_deq[0].pose.pose, guess_pose_odom);
      } else {
        for (int i = odom_deq.size() - 1; i >= 0; i--) {
          ros::Time temp_time = odom_deq[i].header.stamp;
          if ((current_time - temp_time).toSec() >= 0) {
            transform_.MsgToEigen(odom_deq[i].pose.pose, guess_pose_odom);
            ros::Time t_time = odom_deq[get_i].header.stamp;
            break;
          }
        }
      }
    }
  }
}

void ndt_matching::odomCallback(const nav_msgs::Odometry::ConstPtr& input) {
  odom = *input;
  if (fabs(odom.twist.twist.linear.z - 4) < 0.0001)
    odomFlag = true;
  else
    odomFlag = false;

  if (enable_syn_flag || _use_odom) {
    if (odom_deq.size() < odomCacheSize) {
      odom_deq.push_back(odom);
    }
    if (odomCacheSize == odom_deq.size()) odom_deq.pop_front();
  }
}

void ndt_matching::bbxCallback(
    const perception_msgs::ivmsgobjfus::ConstPtr& input) {
  if (initialFlag) get_bbx = *input;
}

void ndt_matching::mapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input) {
  pcl::fromROSMsg(*input, ndt_map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(ndt_map));
  if (map_ptr->size() <= 1) {
    ROS_ERROR("The map for localization is error!!!");
    return;
  } else {
    ndt.setInputTarget(map_ptr);
    map_loaded = true;
  }
}

void ndt_matching::initialposeCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input) {
  transform_.MsgToEigen(input->pose.pose, previous_pose);
  transform_.MsgToEigen(input->pose.pose, current_pose);
  delta_pose = Eigen::Matrix4f::Identity();
  initial_pose_flag = true;
}

void ndt_matching::pointsCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input) {
  if (!setInput(input) || !map_loaded || !initial_pose_flag) return;
  Eigen::Matrix4f guess_pose_for_ndt(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  setGuessPose(guess_pose_for_ndt);
  ndtRegistration(guess_pose_for_ndt, t_localizer);
  publishOutput(t_localizer);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ndt_matching");
  ros::NodeHandle mh;
  ros::NodeHandle private_mh("~");
  ndt_matching start_matching(mh, private_mh);

  ros::spin();
  return 0;
}
