/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: ivdrivercontrol
* FileName: ivdrivercontrol.cpp
*
* Description:
* 1. Using Pid algrithm to control actuator.
*
* History:
* yanbo         16/11/20    1.0.0    build this module.
******************************************************************************/
#include <iostream>
#include "ivdrivercontrol.h"

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ivdrivercontrol");
  ros::NodeHandle nh;
    std::string carname("xxxx");
    nh.param("carname", carname, carname);
    if ("omega" == carname)
    {
        drivercontrol node(nh);
        node.Run();
    }
    else if("cs55white" == carname)
    {
      drivercontrolXJ node(nh);
      node.Run();
    }
    else if("QYred_CS55" == carname)
    {
      drivercontrolXJ node(nh);
      node.Run();
    }
    else
    {
       ROS_ERROR_STREAM("ivdrivercontrol *********Car matching error!!!!!");
    }

  
  return 0;
}
