

#ifndef _MOTIONPLANNER_COM_H
#define _MOTIONPLANNER_COM_H
#pragma once


typedef struct sPoint {
  float32 length;
  float32 curvature;
  float32 y_fitting;
  ivpathplanner::pathpoint pathpoints_in;
} sPoint;



#endif //_MOTIONSPLANNER_COM_H
