

#ifndef _MOTIONPLANNER_COM_H
#define _MOTIONPLANNER_COM_H
#pragma once

typedef unsigned char uint8;
typedef float float32;
typedef double float64;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

typedef struct sPoint {
  float32 length;
  float32 curvature;
  float32 y_fitting;
  ivpathplanner::pathpoint pathpoints_in;
} sPoint;


struct Behavior_State{
	int state;
	float32 length;
	ivpredict::predictobj objpred; 
};


#endif //_MOTIONSPLANNER_COM_H
