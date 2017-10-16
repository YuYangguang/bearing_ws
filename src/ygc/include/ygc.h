/*
 * formation.h
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#ifndef FORMATION_H_
#define FORMATION_H_
//#include "bearing2D.h"


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
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "Eigen/Eigen"
#include "ygc/Bearing2D.h"
#include "ygc/GroupBearing.h"

#define YGC_IDLE 0
#define YGC_WORK 1
#define YGC_PI 3.1415926


namespace smarteye {


class ygcClass
{
public:
    ygcClass(int argc,char** argv,const char * name);
    ~ygcClass();

    int uavNum;
    int ygcMode;
    ros::Timer  ygcUpdateTimer;
    ros::Timer  bearingUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    std::string num2str(int i);
    geometry_msgs::Vector3* allPositions;
    geometry_msgs::Vector3  targetPos;
    double rotateTheta;
    float  rotateSpeed;
    ygc::GroupBearing mutualBearing;
    ygc::GroupBearing targetBearing;
    ygc::GroupBearing expBearing;  //期望的方位角信息
    void ReceiveStateInfo1(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveStateInfo2(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveStateInfo3(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveStateInfo4(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveStateInfo5(const mavros_msgs::State::ConstPtr& msg);
    void ReceiveKeybdCmd(const keyboard::Key &key);
    void ReceiveGazeboInfo(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:
    int updateHz;
    int systemID;
    ros::ServiceClient* paramClient;
    ros::ServiceClient* arming_client;
    ros::ServiceClient* setModeClient;
    ros::ServiceClient* takoffClient;
    mavros_msgs::State* currentState;
//    ros::Publisher* setPositionPublisher;
    ros::Publisher mutualBearingPub;
    ros::Publisher targetBearingPub;
    ros::Publisher expBearingPub;  //期望方位发布者
    ros::Subscriber keyboardSub;
    ros::Subscriber stateSub[5];   //若无人机数量超过5,此处需要修改
    ros::Subscriber gazeboInfoSub;
    geometry_msgs::PoseStamped positionSet;
    geometry_msgs::PoseStamped localPose;
    void update(const ros::TimerEvent& event);
    void bearingUpdate(const ros::TimerEvent& event);
    void cacExpBearing();
    void bearingInfoInit();
};


}


#endif /* FORMATION_H_ */
