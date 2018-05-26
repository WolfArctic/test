#ifndef _MAP_LOADER_H
#define _MAP_LOADER_H
#pragma once

#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <dirent.h>
#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <boost/functional/hash.hpp>

#include "ivlocmsg/ivmsglocpos.h"
#include "ivlocmsg/ndt_status.h"

using UnorderedPCD = std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>>;

class map_loader
{
public:
	map_loader(ros::NodeHandle n);
	~map_loader(){};
	int findSubFile(std::string path);
	void mergeCloud(int x, int y, int row, int col);
	bool checkPCD(int input);
	bool checkIndex(int input, int center_x, int center_y, int row, int col);
	void publish_pcd(ivlocmsg::ivmsglocpos loc_input);
	void callback_gps(const ivlocmsg::ivmsglocpos::ConstPtr &msg);
	void callback_status(const ivlocmsg::ndt_status::ConstPtr &msg);
	void callback_pose(const geometry_msgs::PoseStamped msg);
	void publish_adjacent_pcd(int numX, int numY);

	bool pos_flag;
	bool map_divide_flag;
	double gpsTime, last_gps_time;
	sensor_msgs::PointCloud2 outPointCloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
	ros::Publisher maps_pub;

private:
	int flagX, flagY, map_length, map_bit;  		
	ivlocmsg::ivmsglocpos get_gps_pos;
	bool gps_flag, initial_flag;
	std::vector<std::string> files;
	UnorderedPCD UnorderedPCD_;

	int rotate_img;
	double max_x, max_y, min_x, min_y;
	std::string pointmap_, maps_;
	int pcd_size;
	int map_number;
	ivlocmsg::ndt_status status_;

	ros::Subscriber sub_pose;
	ros::Subscriber sub_status;
	ros::Subscriber sub_gps;
};

#endif

