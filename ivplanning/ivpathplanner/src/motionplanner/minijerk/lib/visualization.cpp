
#include "visualization.h"


Visualization::Visualization(ros::NodeHandle& nh) : initialized_(false)
{
  initialize(nh);
}

void Visualization::initialize(ros::NodeHandle& nh)
{
  if (initialized_)
    ROS_WARN("Visualization already initialized. Reinitalizing...");
  
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("looking_forward_point", 1000);
  poses_pub_ = nh.advertise<visualization_msgs::Marker>("looking_forward_pose",100);
  initialized_ = true; 
}

void Visualization::showPoint(double x, double y) const
{  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  // marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);

  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;
  marker.points.push_back(point);
  
  marker.scale.x = 5;
  marker.scale.y = 5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  marker_pub_.publish( marker );
}

void Visualization::showPoint(std::vector<double> const &p)
{  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "ShowMotionPath";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);
  // marker.ns = ns;
  int tebSize = p.size() - 2;
  if(tebSize <= 0)
    return;

  //   pathtr.setOrigin(tf::Vector3(0, 0, 0.0));
  // pathquater.setRPY(0, 0, 0);
  // pathtr.setRotation(pathquater);
  for(int i = 0; i< p.size()/2; i++)
  {
    geometry_msgs::Point point;
    point.x = p[2*i];
    point.y = p[2*i+1];
    // ROS_INFO("looking poistion1 = %f, position2 = %f", p[2*i],p[2*i+1]);
    point.z = 0;
    marker.points.push_back(point);
    marker.colors.push_back(getColor(i));
  }
  
  marker.scale.x = 2;
  marker.scale.y = 2;
  // marker.color.a = 1.0;
  // marker.color.r = 0.0;
  // marker.color.g = 0.0;
  // marker.color.b = 1.0;

  marker_pub_.publish( marker );
}

std_msgs::ColorRGBA Visualization::getColor(int c)
{
  std_msgs::ColorRGBA mColorRGBA;
  if(c == 0 )
  {
    mColorRGBA.r = 1.0;
    mColorRGBA.g = 1.0;
    mColorRGBA.b = 1.0;
    mColorRGBA.a = 1.0;
  }
  if(c == 1)
  {
    mColorRGBA.r = 0.0;
    mColorRGBA.g = 0.0;
    mColorRGBA.b = 1.0;
    mColorRGBA.a = 1.0;
    
  }
  if(c == 2)
  {
    mColorRGBA.r = 1.0;
    mColorRGBA.g = 0.0;
    mColorRGBA.b = 0.0;
    mColorRGBA.a = 1.0;
  }
  if(c == 3)
  {
    mColorRGBA.r = 0.0;
    mColorRGBA.g = 1.0;
    mColorRGBA.b = 0.0;
    mColorRGBA.a = 1.0;
  }
  if(c == 4)
  {
    mColorRGBA.r = 0.0;
    mColorRGBA.g = 0.0;
    mColorRGBA.b = 0.0;
    mColorRGBA.a = 1.0;
  }
  // else
  // {
  //   mColorRGBA.r = 1.0;
  //   mColorRGBA.g = 0.0;
  //   mColorRGBA.b = 0.0;
  //   mColorRGBA.a = 1.0;
  // }

   return mColorRGBA;
}


void Visualization::showPoint(double x, double y, int c) const
{  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "TestPath";
  // marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);

  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;
  marker.points.push_back(point);
  
  marker.scale.x = 5;
  marker.scale.y = 5;
  // marker.color.a = 1.0;
  // marker.color.r = 0.0;
  // marker.color.g = 0.0;
  // marker.color.b = 1.0;


    if(c == 0 )
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }
  if(c == 1)
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }
  if(c == 2)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }
  if(c == 3)
  {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  }
  else if(c == 4)
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }
  else
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.0;
  }



  poses_pub_.publish( marker );
}
