
#include "odom_node_can.h"
#include "aodom.h"
#include <libgen.h>
#include <unistd.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ivsensorodom");
    ros::NodeHandle nh;

    std::string dataReadMode("serial");
    nh.param("odom_data_read_mode", dataReadMode, dataReadMode);
    OdomNodeCan node(nh);
    node.run();
               
    return 0;
}
