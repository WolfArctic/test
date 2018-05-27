/*
 * Copyright (C) 2016, BeiJing ZhiXingZhe, Inc.
 *
 * Author Information:
 * fang zhang
 * zhangfang@idriverplus.com, 
 * 
 * Node Information:
 * 
 */

/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: ivpredict.cpp
Description: 
This node is used to predict the trajectory of surrounding objects.

History:
<author>    <time>      <version>    <description> 
fang zhang  16/09/06    1.0.0        creat the file 

************************************************************/

#include "ivpredict.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv,"ivpredict");
  ros::NodeHandle nh;
  PredictManager node(nh);
  node.run();      
  return 0;
}
