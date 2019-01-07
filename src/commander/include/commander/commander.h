#ifndef _COMMANDER_H_
#define _COMMANDER_H_

#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>
#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <bearing_common/HParamSetSrv.h>

namespace smarteye{

class commander
{
public:
    commander(int argc,char** argv,const char * name);
    ~commander();

private:
    boost::shared_ptr<ros::NodeHandle> nh;
    void update(const ros::TimerEvent& event);
    ros::Timer  commanderUpdateTimer;
    ros::ServiceClient* param_set_clinet;
    float hei_ki;
    float hei_kp1;
    float hei_kp2;
    float hei_kpdiv;
    float hei_kd;
    float hei_bias;
    float xy_ki;
    float xy_kp;
    float xy_kd;
    float xy_bias;
    float env_k_alpha;
    float env_k_beta;
    float env_k_gamma;
    float rotTheta;
    int uavNum;
    void initParam(void);
};



}

#endif


