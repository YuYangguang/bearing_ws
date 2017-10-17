#include <formation.h>


using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start formation node process");
    ros::Time::init();
    Formation this_formation(argc,argv,"");
    ros::spin();
    return 0;


}

