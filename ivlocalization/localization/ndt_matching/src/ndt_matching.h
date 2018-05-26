#ifndef _NDT_MATCHING_H
#define _NDT_MATCHING_H
#pragma once

#include <iostream>
#include <sstream>  
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include "ros/console.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/ndt.h>

#ifdef USING_GPU_NDT_
#include "NormalDistributionsTransform.h"
#endif

#include "ivlocmsg/ndt_status.h"
#include "perception_msgs/ivmsgobjfus.h"
#include "ivlocmsg/ivsensorodom.h"
#include "../../../loclib/igeometry.hpp"
#include "../../../loclib/transform.h"

class ndt_matching
{
public:
	ndt_matching(ros::NodeHandle nh, ros::NodeHandle private_nh);
	~ndt_matching(){};
	void init_params();
	inline int FastRand();
	void removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, 
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output);
	void setGuessPose(Eigen::Matrix4f &output);
	bool setInput(const sensor_msgs::PointCloud2::ConstPtr& input);
	void ndtRegistration(Eigen::Matrix4f input_guess, Eigen::Matrix4f &output);
	void publishOutput(Eigen::Matrix4f input);
	void odomCalculate(ros::Time current_time);
	void bbxCallback(const perception_msgs::ivmsgobjfus::ConstPtr& input);
	void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& input);

private:
	pcl::PointCloud<pcl::PointXYZ> ndt_map;
	std::string POINTS_TOPIC;

#ifndef USING_GPU_NDT_
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
#else
	gpu::GNormalDistributionsTransform ndt;
#endif

	std::deque<nav_msgs::Odometry> odom_deq;
	// Default values
	int max_iter;
	float ndt_res;
	double step_size;
	double trans_eps;
	double exe_time;
	int iteration;
	double fitness_score ;
	double trans_probability;
	double trans_probability_adj;

	bool _get_height;
	bool _use_odom;
	bool initialFlag;
	bool map_loaded;
	bool odomFlag;
	bool initial_pose_flag;
	bool enable_syn_flag;
	double voxel_leaf_size;
	int fast_rand_seed;
	ros::Time current_scan_time;
	double min_scan_range;
	double max_scan_range;
	double min_scan_z;
	double align_time;
	unsigned int odomCacheSize;


	Eigen::Matrix4f initial_pose, previous_pose, guess_pose, guess_pose_odom, current_pose, delta_pose;
	loclib::transform<float, float> transform_;
	double angular_velocity, predict_pose_error;

	double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
	double input_x_, input_y_, input_z_, input_roll_, input_pitch_, input_yaw_;
	double input_res_, input_step_size_, input_trans_eps_, input_max_iter_, input_threshold_; 
	tf::StampedTransform local_transform;

	nav_msgs::Odometry odom;
	perception_msgs::ivmsgobjfus get_bbx;

	//publisher
	ivlocmsg::ndt_status ndt_stat_msg;
	geometry_msgs::PoseStamped ndt_pose_msg;
	geometry_msgs::PoseStamped adj_pose_msg;

	ros::Publisher ndt_stat_pub;
	ros::Publisher ndt_pose_pub;
    ros::Publisher adj_pose_pub;

	ros::Subscriber map_sub;
	ros::Subscriber initialpose_sub;
	ros::Subscriber points_sub;
	ros::Subscriber bbx_sub;
	ros::Subscriber odom_sub;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr current_map_ptr;
	pcl::PointCloud<pcl::PointXYZ>::Ptr last_map_ptr;
};

#endif
