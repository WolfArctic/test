/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: zhangbaofeng
 * This node is used to read serial data and publish the data content, and subscription data content into a serial port.
 * * *************************************************************************
 * */

#ifndef ACTUATOR_H
#define ACTUALOR_H

#include <stdio.h>
#include <sys/types.h>         
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "pthread.h"
#include "serial/serial.h"
#include "ivactuator/ivactuator.h"
#include "ivactuator/wheelspeed.h"
#include "ivactuator/lockstate.h"
#include "ivsteercontrol/ivsteercontrol.h"
#include "ivdrivercontrol/ivdrivercontrol.h"
#include "monitor_msgs/monitofaultreaction.h"
#include "ivapp/remotecontrol.h"
#include "ivpathplanner/carstateshow.h"
#include "ivapp/ivappopenbox.h"
#include "ivlocmsg/ivsensorgps.h"
#include "monitor_client.h"  
#include "globalvariables.h" 

#define AVCU_TX2_85 0x85
#define AVCU_XEP_86 0x86
#define AVCU_TX2_F5 0xF5
#define BCM_201     0x201
#define BCM_202     0x202
#define EPS_401     0x401
#define LIGHT_HORN_DELAY_TIME 5000  //ms
// #define RECV_MSG_LEN 25
// enum warnningType{
//     PERIPHERAL_FAULT = 1,     //Peripheral fault
//     FRONT_NODE_DATA_QUALITY,      //Data quality of front node
//     FRONT_NODE_ABSOLUTE_CYCLE,     //Absolute period of front node transmission
//     FRONT_NODE_RELATIVE_CYCLE,     //Relative period of front node transmission
//     THIS_NODE_ABSOLUTE_CYCLE,     //Absolute cycle of the node
//     THIS_NODE_RELATIVE_CYCLE      //Relative cycle of the node_
// };

typedef struct{
    ivsteercontrol::ivsteercontrol msg;
    bool isvalid;
}T_steerControlMsg;

typedef struct{
    ivdrivercontrol::ivdrivercontrol msg;
    bool isvalid;
}T_driverControlMsg;

// typedef struct{
//     ivapp::remotecontrol msg;
//     bool isvalid;
// }T_remoteControlMsg;

// typedef struct{
//     bool start;
//     int  steerAngle;
//     int  targetacc;
//     int  targettorque;
//     int  targetshiftposition;
//     int  actuatormode;
// }T_remoteControlInfo;

typedef struct{
    ivpathplanner::carstateshow msg;
    bool isvalid;
}T_carstateshowMsg;

typedef struct{
    ivapp::ivappopenbox msg;
    bool isvalid;
}T_ivappopenboxMsg;

typedef struct{
    monitor_msgs::monitofaultreaction msg;
    bool isvalid;
}T_monitofaultreactionMsg;

enum lightHornName{
    TURNLIGHTLEFT,
    TURNLIGHTRIGHT,
    BRAKELIGHT,
    HORN
};

typedef struct lightHornControl{
    bool status;
    unsigned short counter; 
    unsigned short delaytime;
}T_lightHornControl;

class actuator{
  public:
    actuator(ros::NodeHandle handle);
    ~actuator(){}  
    // ~actuator(){delete mcuMonitor;mcuMonitor = NULL;} 
    void run();
    void recvCarInfoKernel();
    void sendCarInfoKernel();
    void callback_sendthread();
    void callback_ivsteercontrol(const ivsteercontrol::ivsteercontrol::ConstPtr &msg);
    void callback_ivdrivercontrol(const ivdrivercontrol::ivdrivercontrol::ConstPtr &msg);
    void callback_carstateshow(const ivpathplanner::carstateshow::ConstPtr &msg);
    void callback_monitofaultreaction(const monitor_msgs::monitofaultreaction::ConstPtr &msg);
    void callback_ivappopenbox(const ivapp::ivappopenbox::ConstPtr &msg);

    // void serialRecvCarInfoKernel();
    // void serialSendCarInfoKernel();
    // void callback_serialsendthread();
    // void monitorFrontNodeData();
    // void monitorNodeAbsoluteCycle(float absoluteCycle,int warnningType,float N,float offset);
    // void monitorNodeRelativeCycle(float relativeCycle,int warnningType,float offset);
    // void sendWarnnigToMonitor(int monitorWarnningType,int monitorWarnningValue);
    // void callback_serialivsteercontrol(const ivsteercontrol::ivsteercontrol::ConstPtr &msg);
    // void callback_serialivdrivercontrol(const ivdrivercontrol::ivdrivercontrol::ConstPtr &msg);
    // void callback_serialivapp(const ivapp::remotecontrol::ConstPtr &msg);
  private:  
    // int  m_baudrate;
    // int  m_deviceName;
    // int  mcuCount;
    // int  steerControlDataCount;
    // int  driverControlDataCount;
    // int  receiverCurrentByteCount;    //The number of have received byte
    // bool steerTimeFlag;
    // bool driverTimeFlag;
    // unsigned char   tempDataArray[512];//Size of serial buffer
    // std::string     m_serialport;
    // Monitor         *mcuMonitor;
    // Monitor         *underMcuMonitor;
    // serial::Serial  ser;
    // ros::Time steerTimeTemp;
    // ros::Time driverTimeTemp;
    // T_remoteControlMsg t_remoteControlMsg;
    // T_remoteControlInfo t_remoteControlInfo;
    // ros::Subscriber sub_appremote;

    int  socketcan_fd;
    std::string can_serial_data_flag;
    ros::NodeHandle m_handle;
    ros::Publisher  pub_control;
    ros::Publisher  pub_wheelspeed;
    ros::Publisher  pub_lockstate;
    ros::Subscriber sub_steer;
    ros::Subscriber sub_driver;
    ros::Subscriber sub_carstateshow;
    ros::Subscriber sub_appopenbox;
    ros::Subscriber sub_monitofaultreaction;
    T_steerControlMsg        t_steerControlMsg;
    T_driverControlMsg       t_driverControlMsg;
    T_carstateshowMsg        t_carstateshowMsg;
    T_ivappopenboxMsg	     t_ivappopenboxMsg;
    T_monitofaultreactionMsg t_monitofaultreactionMsg;
    T_lightHornControl       t_lightHornControl[sizeof(enum lightHornName)];
    ivactuator::ivactuator ivactuatorMsg;
    ivactuator::wheelspeed wheelspeedMsg;
    ivactuator::lockstate  lockstateMsg;
};

#endif /*ACTUALOR_H*/
