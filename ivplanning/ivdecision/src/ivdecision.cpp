

/*
 * Copyright (C) 2016, BeiJing ZhiXingZhe, Inc.
 *
 * Author Information:
 * Fang Zhang
 * zhangfang@idriverplus.com,
 *
 * Node Information:
 * This node is used for decision-making.
 */

#include "omegadecision.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ivdecision");
    ros::NodeHandle nh;
    std::string avos("lsav");
    nh.param("avos", avos, avos);
    // if ("lsav" == avos)
    // {
    omegadecision node(nh);
    node.run();
    // }
    // else if ("hsav" == avos)
    // {
    //     yuyandecision node(nh);
    //     node.run();
    // }

    return 0;
}
