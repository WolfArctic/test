

/*
 * Copyright (C) 2016, BeiJing ZhiXingZhe, Inc.
 *
 * Author Information:
 * Fang Zhang
 * zhangfang@idriverplus.com,
 *
 * Node Information:
 * This node is used to plan iapath.
 */

#include "omegapathplanner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ivpathplanner");
    ros::NodeHandle nh;
    std::string avos("lsav");
    nh.param("avos", avos, avos);

    omegapathplanner node(nh);
    node.run();

    return 0;
}
