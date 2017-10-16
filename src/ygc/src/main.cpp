
#include "ygc.h"
#include <stdio.h>
#include <ros/ros.h>


using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start ygc node process");
    ros::Time::init();
    ygcClass this_formation(argc,argv,"ygc_node");
    ros::spin();
    return 0;
}
