/*
 * formation.h
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#ifndef FORMATION_H_
#define FORMATION_H_

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cstring>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mavros_msgs/CommandTOL.h>
#include <keyboard/Key.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "ygc/GroupBearing.h"
#include "Eigen/Eigen"

#define YGC_HOVER 0
#define YGC_TAKEOFF 1
#define YGC_LAND 2
#define YGC_ENCIRCLE 3
#define YGC_CIRCLE 4

namespace smarteye {
float ValueLimit(float value,float max,float min);
class heiCtrller
{
public:
    heiCtrller();
    ~heiCtrller();
    double currentHei;
    double targetHei;
    float hei_kp;
    float hei_ki;
    float hei_kd;
    float hei_bias;
    float previousErr;
    float ei;  //控制器积分项
    float cacOutput();
};
class Formation
{
public:
    Formation(int argc,char** argv,const char * name);
    ~Formation();
    double uavRollENU, uavPitchENU, uavYawENU;
    ros::Timer  formationUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    std::string num2str(int i);
    void ReceiveStateInfo(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveKeybdCmd(const keyboard::Key &key);
    ygc::GroupBearing mutualBearing;
    ygc::GroupBearing targetBearing;
    ygc::GroupBearing expBearing;
    heiCtrller heiCtr;

private:
    int updateHz;
    int systemID;
    float env_k_alpha;
    float env_k_beta;
    int uavState;
    ros::ServiceClient paramClient;
    ros::ServiceClient arming_client;
    ros::ServiceClient setModeClient;
    ros::ServiceClient takoffClient;
    mavros_msgs::State px4State;
    void update(const ros::TimerEvent& event);
    ros::Publisher setPositionPub;
    ros::Publisher setVelPub;
    ros::Subscriber keyboardSub;
    ros::Subscriber px4StateSub;
    ros::Subscriber localPoseSub;
    ros::Subscriber mutualBearingSub;
    ros::Subscriber targetBearingSub;
    ros::Subscriber expBearingSub;
    geometry_msgs::PoseStamped desiredPose;
    geometry_msgs::PoseStamped positionSet;
    geometry_msgs::TwistStamped velocitySet;
    geometry_msgs::PoseStamped localPose;
    void ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr& msg);
    void ReceiveMulBearing(const ygc::GroupBearingConstPtr &msg);
    void ReceiveTarBearing(const ygc::GroupBearingConstPtr &msg);
    void ReceiveExpBearing(const ygc::GroupBearingConstPtr &msg);
    void initParamServer();
    void takeoffCtr();
    void landCtr();
    void encircleCtr(double targetHei);  //输入，期望的高度


};

}



#endif /* FORMATION_H_ */
