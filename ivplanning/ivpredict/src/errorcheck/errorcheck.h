/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: omegapredict.h
Description: 
1. library define
2. static varrible difine
3. omegapredict class declear

History:
<author>    <time>      <version>    <description> 
hui li      17/06/06    1.0.1       modified function names (delete _ )  

************************************************************/

#ifndef ERRORCHECK
#define ERRORCHECK
#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>



//msg
#include "ivmap/ivmapmsgobj.h"
#include "ivmap/ivmapmsgroad.h"
#include "ivmap/ivmapmsgvsp.h"
#include "ivmap/ivmapmsgvap.h"
#include "ivmap/ivmapmsgapp.h"
#include "ivmap/ivmapmsguserfun.h"
#include "ivmap/ivmapmsgtraffic.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgloctag.h"
#include "ivpredict/ivmsgpredict.h"
#include "ivpredict/ivmsgpredictdebug.h"
#include "ivpredict/predictobj.h"
#include "ivpredict/predictobjdebug.h"
#include "ivpredict/predictpos.h"
#include "ivpredict/ivpredictbuffer.h"
#include "ivpredict/objbuffer.h"

// #include "monitor_client.h"

using namespace std;

class errorcheck
{
  public:
    errorcheck();
    ~errorcheck();

    int BuffermsgCheck(ivpredict::ivpredictbuffer* buffer_check);
    int PredictmsgCheck(ivpredict::ivmsgpredict* predictobj_check);
    void Inputcheck(ivmap::ivmapmsgobj* inputobj_check);
    //Monitor *predictMonitor;
  private:
  	// Monitor *predictMonitor;
};



#endif
