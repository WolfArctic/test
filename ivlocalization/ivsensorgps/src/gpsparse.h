/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: WangXiao
 * This node is used to parse gps data
 * change log:
 * 20170419:
 * 1) change velocity from km/h -->m/s
 * * *************************************************************************
 * */
#pragma once
//avoslib
//node ivsensorgps
#include "../../../avoslib/geotool.h"

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64MultiArray.h"
//lib
#include "serial/serial.h"
// Custom message includes. Auto-generated from msg/ directory.
#include "ivlocmsg/ivsensorgps.h"
#include <time.h>

/* FOREGROUND */

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST
#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#define DPCAR 0
#define VREP 1
#define DEBUGHEAD "[ivsensorgps-->]"
using namespace std;

typedef struct Point2D{
	double x;
	double y;
	double lat;
	double lon;
}Point2D;

typedef struct sAddAttr{
  // int posattr;
  int roadattr;
}sAddAttr;

typedef struct sGpsPoint{
  double utctime;
  double lon;
  double lat;
  double heading;
  double velocity;
  unsigned int status;
  unsigned int satenum;
  unsigned int mode;

  float hdop;     //add for IMU/GPS by wanglj
  float height;
  float track_angle;

  float ve;
  float vn;
  float vu;

  int pos_status;
  int att_status;

}sGpsPoint;


class gpsparse
{
  public:
  //! Constructor.
  gpsparse(ros::NodeHandle nh);
  ~gpsparse(){}
  void run(); //add by wangxiao, 2016.07.30

  private:
  //! Timer callback for publishing message.
  void naviColKernel();
  void receiveData(unsigned char* str, int len);
  // parse gps datas base different protocol

  void parseGpvtg();
  void parseGpgga();
  void parseGprmc();
  void parseGphpr();
  void parseKSXT();

  void callback_vrep(const std_msgs::Float64MultiArray::ConstPtr msg);
  void publishMsg(sGpsPoint data);
  double getDist2(Point2D p1, Point2D p2);

  double GetUtcTime(double time);
  bool CheckSum(unsigned char *pstr);

  private:
  //! Message publisher.
  ros::Publisher pub_;
  ros::Subscriber sub_;
  //
  serial::Serial ser;
  //gps object
  sGpsPoint rp,lastRp;
  sAddAttr addattr;
  //for gps data parse
  unsigned char TempDataArray[4096];
  int ReceiverCurrentByteCount;
  int gpsDeviceName; //0-sinan 1-lianshi 2-qianxun
  int baudrate;
  int loopFrep;
  int runningmode;
  int bdebug;

  std::string serialport;
  std::string savePath;
  FILE* fp;
  //add in 20170412
  geotool gt;

  bool checksum_gga;
  bool checksum_rmc;
  bool checksum_hpr;
  bool checksum_ksxt;
  bool checksum_vtg;

  bool is_debug;
};

