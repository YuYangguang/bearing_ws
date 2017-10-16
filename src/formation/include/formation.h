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
namespace smarteye {
class Formation
{
public:
    Formation(int argc,char** argv,const char * name);
    ~Formation();
    double uavRollENU, uavPitchENU, uavYawENU;
    ros::Timer  formationUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    mavros_msgs::State stateFCU_;
    std::string num2str(int i);
    void ReceiveStateInfo(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveKeybdCmd(const keyboard::Key &key);
    ygc::GroupBearing mutualBearing;
    ygc::GroupBearing targetBearing;
private:
    int updateHz;
    int systemID;
    float k_alpha;
    float k_beta;
    ros::ServiceClient paramClient;
    ros::ServiceClient arming_client;
    ros::ServiceClient setModeClient;
    ros::ServiceClient takoffClient;
    mavros_msgs::State current_state;
    void update(const ros::TimerEvent& event);
    ros::Publisher setPositionPublisher;
    ros::Subscriber keyboardSub;
    ros::Subscriber state_sub;
    ros::Subscriber localPoseSub;
    ros::Subscriber mutualBearingSub;
    ros::Subscriber targetBearingSub;
    geometry_msgs::PoseStamped desiredPose;
    geometry_msgs::PoseStamped positionSet;
    geometry_msgs::PoseStamped localPose;
    void ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr& msg);
    void ReceiveMulBearing(const ygc::GroupBearingConstPtr &msg);
    void ReceiveTarBearing(const ygc::GroupBearingConstPtr &msg);

};

}



#endif /* FORMATION_H_ */
