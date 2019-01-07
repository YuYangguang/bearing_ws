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

#include "../../bearing_common/include/3rd_party_header.h"

#define YGC_PI 3.1415926
#define IDLE 0
#define CIRCLE 1
#define ENCIRCLE 2

namespace smarteye {


class ygcClass
{
public:
    ygcClass(int argc,char** argv,const char * name);
    ~ygcClass();

    int uavNum;
    int ygcMode;
    int uavForRec;   //需要记录数据的飞机号
    ros::Timer  ygcUpdateTimer;
    ros::Timer  bearingUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    std::string num2str(int i);
    geometry_msgs::Vector3* allPositions;
    nav_msgs::Odometry  targetPos;
    double rotateTheta;
    float  rotateSpeed;
    bearing_common::GroupBearing mutualBearing;
    bearing_common::GroupBearing mutualBearingErr; //事件驱动机制下的智能体间方位误差
    bearing_common::GroupBearing agentVel;
    bearing_common::GroupBearing targetBearing;
    bearing_common::GroupBearing targetBearingErr;  //事件驱动机制下智能体与目标间的方位误差
    bearing_common::GroupBearing lastMutuBearing;   //上一次更新的智能体之间的方位
    bearing_common::GroupBearing lastTargetBearing;  //上一次更新的智能体与目标之间的方位
    bearing_common::GroupBearing dotMutualBearing; //智能体与目标的方位变化率
    bearing_common::GroupBearing expBearing;  //期望的方位角信息
    bearing_common::DataRecord dataRec;
    bearing_common::AllPosition posRec;
    bearing_common::TriggerRec triggerRec; //统计事件触发情况，x表示触发次数，y表示此时走了一个控制周期
    double lastBearUpTime;   //上次方位刷新时间

    void ReceiveCmdInfo1(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void ReceiveCmdInfo2(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void ReceiveKeybdCmd(const keyboard::Key &key);
    void ReceiveGazeboInfo(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:

    int updateHz;
    int coordMethod;     //协同方式，是环航还是围捕
    bool IsUseEventTri;   //是否使用event-trigger机制
    bool IsUseSimu;     //是否是仿真模式
    bool IsInfoUpdate;
    float cir_sigma1;
    float cir_sigma2;
    float encir_sigma1;
    float encir_sigma2;
    int updateCount;
    ros::Publisher mutualBearingPub;
    ros::Publisher targetBearingPub;
    ros::Publisher expBearingPub;  //期望方位发布者
    ros::Publisher dataRecPub;
    ros::Publisher allPosiPub;
    ros::Publisher targetPosePub;
    ros::Publisher dotBearingPub;
    ros::Publisher trigRecPub;
    ros::Publisher agentVelPub;
    ros::Subscriber keyboardSub;
    ros::Subscriber velCmdSub;   //订阅无人机速度命令消息
    ros::Subscriber targetViconSub;  //订阅vicon系统下的目标位置
    double lastTargetTime;  //vicon系统下上一次接收到目标位置的时间，用于计算target的差分速度
    bearing_common::Vector2D lastTargetPose2D;
    ros::Subscriber uavPosSub1;
    ros::Subscriber uavPosSub2;
    ros::Subscriber uavPosSub3;
    ros::Subscriber gazeboInfoSub;
    geometry_msgs::PoseStamped positionSet;
    geometry_msgs::PoseStamped localPose;
    void update(const ros::TimerEvent& event);
    void bearingUpdate(const ros::TimerEvent& event);
    void cacExpBearing();
    void bearingInfoInit();
    void ReceiUavPos1(const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    void ReceiUavPos2(const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    void ReceiUavPos3(const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    void ReceiveTarViconPose(const geometry_msgs::TransformStampedConstPtr &vicon_msg);
};


}


#endif /* FORMATION_H_ */
