#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <commander/commander.h>

using namespace smarteye;

int main(int argc, char **argv)
{

    ROS_INFO("start commander_node process");
    ros::Time::init();
    //ros::init(argc, argv, "commander");
    commander this_commander(argc,argv,"commander");
    ros::spin();
    return 0;

}
