
#pragma once

#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
class Visualization
{
public:
  Visualization(ros::NodeHandle& nh);
  
  void initialize(ros::NodeHandle& nh);
  void showPoint(double x, double y) const;
  void showPoint(std::vector<double> const &p);
  void showPoint(double x, double y, int c) const;
  std_msgs::ColorRGBA getColor(int c);
  
protected:
  ros::Publisher marker_pub_; 
  ros::Publisher poses_pub_;
  bool initialized_;

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};
