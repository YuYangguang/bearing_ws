/*
 * formation.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#include "ygc.h"
using namespace smarteye;

ygcClass::ygcClass(int argc, char **argv, const char *name)
{
    //sleep(1);
    uavNum = 5;
    systemID = -1;
    updateHz = 20;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "ygc_node");
    nh = boost::make_shared<ros::NodeHandle>("~");
    bool IsGotParam;
    IsGotParam = nh->getParam("uav_num",uavNum);
    if(IsGotParam)
    {
        ROS_INFO("got system id successfully and uav number is %d",uavNum);
    }
    else
    {
        ROS_ERROR("failed to get uav number and node is shut down");
        exit(0);
    }

    currentState = new mavros_msgs::State[uavNum];
    paramClient = new ros::ServiceClient[uavNum];
    arming_client = new ros::ServiceClient[uavNum];
    setModeClient = new ros::ServiceClient[uavNum];
//    setPositionPublisher = new ros::Publisher[uavNum];
    allPositions = new geometry_msgs::Vector3[uavNum];
    mutualBearing.bearings.resize(uavNum);
    targetBearing.bearings.resize(uavNum);
    expBearing.bearings.resize(uavNum*2);
    std::string uavName;
    for(int i=0;i<uavNum;i++)
    {
        uavName = "/uav"+num2str(i+1);
        arming_client[i] = nh->serviceClient<mavros_msgs::CommandBool>(uavName+"/mavros/cmd/arming");
        setModeClient[i] = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/set_mode");
//        setPositionPublisher[i] = nh->advertise<geometry_msgs::PoseStamped>
//                (uavName+"/mavros/setpoint_position/local",10);

    }
    targetPos.x = 0;
    targetPos.y = 0;
    targetPos.z = 0;
    rotateTheta = 0;
    rotateSpeed = YGC_PI/60;
    stateSub[0] = nh->subscribe("/uav1/mavros/state", 5,&ygcClass::ReceiveStateInfo1, this);
    stateSub[1] = nh->subscribe("/uav2/mavros/state", 5,&ygcClass::ReceiveStateInfo2, this);
    stateSub[2] = nh->subscribe("/uav3/mavros/state", 5,&ygcClass::ReceiveStateInfo3, this);
    stateSub[3] = nh->subscribe("/uav4/mavros/state", 5,&ygcClass::ReceiveStateInfo4, this);
    stateSub[4] = nh->subscribe("/uav5/mavros/state", 5,&ygcClass::ReceiveStateInfo5, this);
    mutualBearingPub = nh->advertise<ygc::GroupBearing>("/ygc/mutual_bearing",2);
    targetBearingPub = nh->advertise<ygc::GroupBearing>("/ygc/target_bearing",2);
    expBearingPub = nh->advertise<ygc::GroupBearing>("/ygc/expected_bearing",2);
    gazeboInfoSub = nh->subscribe("/gazebo/model_states", 5,&ygcClass::ReceiveGazeboInfo, this);
    keyboardSub= nh->subscribe("/keyboard/keydown",1,&ygcClass::ReceiveKeybdCmd,this);
    ygcUpdateTimer = nh->createTimer(ros::Duration(updateTime),&ygcClass::update, this);
    bearingUpdateTimer = nh->createTimer(ros::Duration(0.02),&ygcClass::bearingUpdate, this);
    bearingInfoInit();

}



ygcClass::~ygcClass()
{
    delete[] currentState;
    delete[] paramClient;
    delete[] arming_client;
    delete[] setModeClient;
//    delete[] setPositionPublisher;
    delete[] allPositions;
}



std::string ygcClass::num2str(int i)
{
    std::stringstream ss;
    ss<<i;
    return ss.str();
}

void ygcClass::ReceiveStateInfo1(const mavros_msgs::State::ConstPtr &msg)
{
    currentState[0] = *msg;
}

void ygcClass::ReceiveStateInfo2(const mavros_msgs::State::ConstPtr &msg)
{
    currentState[1] = *msg;
}

void ygcClass::ReceiveStateInfo3(const mavros_msgs::State::ConstPtr &msg)
{
    currentState[2] = *msg;
}

void ygcClass::ReceiveStateInfo4(const mavros_msgs::State::ConstPtr &msg)
{
    currentState[3] = *msg;
}

void ygcClass::ReceiveStateInfo5(const mavros_msgs::State::ConstPtr &msg)
{
    currentState[4] = *msg;
}

void ygcClass::ReceiveKeybdCmd(const keyboard::Key &key)
{
    switch(key.code)
    {
    case 'a':   //arm the vehicle
    {
        for(int i=0;i<uavNum;i++)
        {
            if(currentState[i].armed)
            {
                ROS_WARN(" vehicle %d is already armed!",i+1);
            }
            else
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if( arming_client[i].call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle %d armed",i+1);
                }
                else
                {
                    ROS_WARN("failed to arm vehicle");
                }
            }
        }
        break;
    }
    case 'd':   //disarm
    {
        for(int i=0;i<uavNum;i++)
        {
            if(!currentState[i].armed)
            {
                ROS_WARN("the vehicle %d is already disarmed",i+1);
            }
            else
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if( arming_client[i].call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle %d disarmed",i+1);
                }
                else
                {
                    ROS_WARN("failed to disarm vehicle %d",i+1);
                }

            }
        }
        break;
    }
    case 'o':   //set offboard mode
    {
        for(int i=0;i<uavNum;i++)
        {
            mavros_msgs::SetMode setmodeCMD;
            setmodeCMD.request.custom_mode = "OFFBOARD";
            if(setModeClient[i].call(setmodeCMD) && setmodeCMD.response.success)
            {
                ROS_INFO("the mode of vehicle %d has changed to offboard",i+1);
            }
            else
            {
                ROS_WARN("failed to set vehicle %d offboard mode!",i+1);
            }
        }
        break;
    }
//    case 't':   //take off
//    {
//        ygcMode = YGC_WORK;
//        positionSet.pose.position.x = 0;
//        positionSet.pose.position.y = 0;
//        positionSet.pose.position.z = 2;
//        break;
//    }
//    case 'l': //land
//    {
//        ygcMode = YGC_WORK;
//        positionSet.pose.position.x = 0;
//        positionSet.pose.position.y = 0;
//        positionSet.pose.position.z = 0;
//        break;
//    }

    }
}

/*此处容易出现bug,如用于其他用途，请仔细理解源码*/
void ygcClass::ReceiveGazeboInfo(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int dataSize = msg->pose.size();
    int numberPos = 6; //此处表示为model名字中数字所在的位置，因为本次实验命名规则为
    //iris_+ID号，因此这个变量为6,如命名规则变更，请变更数字。
    for(int i=0;i<dataSize;i++)
    {
        std::string nameModel ="iris_";
        std::string targetName = "targe";
        std::string temp= msg->name[i].substr(0,numberPos-1);
        if(strcmp(msg->name[i].substr(0,numberPos-1).c_str(),nameModel.c_str()) == 0)
        {
            std::string uavID_string=msg->name[i].substr(numberPos-1);
            int IDLength = uavID_string.length();
            char* uavID_char = new char[IDLength];
            uavID_string.copy(uavID_char,IDLength,0);
            int uavID = atoi(uavID_char);
            delete[] uavID_char;  //释放内存
            if(uavID >uavNum)
            {
                ROS_WARN("vehicle ID is beyond the number of vehicle");
            }
            else
            {
                allPositions[uavID-1].x = msg->pose[i].position.x;
                allPositions[uavID-1].y = msg->pose[i].position.y;
                allPositions[uavID-1].z = msg->pose[i].position.z;
            }

        }
        if(strcmp(msg->name[i].substr(0,numberPos-1).c_str(),targetName.c_str()) == 0)
        {
            targetPos.x = msg->pose[i].position.x;
            targetPos.y = msg->pose[i].position.y;
            targetPos.z = msg->pose[i].position.z;
        }
    }
}

void ygcClass::update(const ros::TimerEvent &event)
{
//    if(ygcMode == YGC_WORK)
//    {
//        for(int i=0;i<uavNum;i++)
//        {
//            positionSet.pose.position.z = 1;
//            setPositionPublisher[i].publish(positionSet);
//        }
//    }
    mutualBearingPub.publish(mutualBearing);
    targetBearingPub.publish(targetBearing);
    expBearingPub.publish(expBearing);

}

void ygcClass::bearingUpdate(const ros::TimerEvent &event)
{
    rotateTheta = rotateTheta + rotateSpeed*0.02;
    for(int i=0;i<uavNum;i++)
    {
        float dis;
        if(i==uavNum-1)
        {
            dis = sqrt(pow((allPositions[i].x -allPositions[0].x),2)
                    + pow((allPositions[i].y -allPositions[0].y),2)); //两点之间的距离
            mutualBearing.bearings[i].x = (allPositions[i].x -allPositions[0].x)/dis;
            mutualBearing.bearings[i].y = (allPositions[i].y -allPositions[0].y)/dis;
        }
        else
        {
            dis = sqrt(pow((allPositions[i].x -allPositions[i+1].x),2)
                    + pow((allPositions[i].y -allPositions[i+1].y),2));
            mutualBearing.bearings[i].x = (allPositions[i].x -allPositions[i+1].x)/dis;
            mutualBearing.bearings[i].y = (allPositions[i].y -allPositions[i+1].y)/dis;
        }
        dis = sqrt(pow((allPositions[i].x -targetPos.x),2)+pow((allPositions[i].y -targetPos.y),2));
        targetBearing.bearings[i].x = (allPositions[i].x-targetPos.x)/dis;
        targetBearing.bearings[i].y = (allPositions[i].y-targetPos.y)/dis;
    }
    cacExpBearing();
}

void ygcClass::cacExpBearing()
{

    Eigen::Vector2d desiredBearing;
    Eigen::Vector2d initDesBearing;  //初始期望方位
    Eigen::Vector2d initTarDesBearing;  //初始智能体与目标的期望方位
    Eigen::Matrix2d rotateMatrix;
    rotateMatrix << cos(rotateTheta),-sin(rotateTheta),
            sin(rotateTheta),cos(rotateTheta);
    for(int i=0;i<uavNum;i++)
    {
        double agentTheta = 2*YGC_PI*i/uavNum;
        double nextAgentTheta = 2*YGC_PI*(i+1)/uavNum;
        float dis =2*sin(YGC_PI/uavNum);  //单位圆上正多边形的边长
        if(i== (uavNum-1))
        {
            initDesBearing[0] = (sin(agentTheta)-sin(0))/dis;
            initDesBearing[1] = (cos(agentTheta)-cos(0))/dis;

        }
        else
        {
            initDesBearing[0] = (sin(agentTheta)-sin(nextAgentTheta))/dis;
            initDesBearing[1] = (cos(agentTheta)-cos(nextAgentTheta))/dis;
        }
        desiredBearing = rotateMatrix*initDesBearing;
        expBearing.bearings[i].x = desiredBearing[0];
        expBearing.bearings[i].y = desiredBearing[1];
        initTarDesBearing[0] = sin(agentTheta);
        initTarDesBearing[1] = cos(agentTheta);
        desiredBearing = rotateMatrix*initTarDesBearing;
        //与目标的期望方位从uavNum+1开始，到2×uavNum
        expBearing.bearings[i+uavNum].x = desiredBearing[0];
        expBearing.bearings[i+uavNum].y = desiredBearing[1];
    }

}


/***函数功能：初始化方位信息*******/
void ygcClass::bearingInfoInit()
{

    for(int i=0;i<uavNum;i++)
    {
        double agentTheta = 2*YGC_PI*i/uavNum;
        float dis =2*sin(YGC_PI/uavNum);  //单位圆上正多边形的边长
        double nextAgentTheta = 2*YGC_PI*(i+1)/uavNum;
        if(i== (uavNum-1))
        {

            expBearing.bearings[i].x = (sin(agentTheta)-sin(0))/dis;
            expBearing.bearings[i].y = (cos(agentTheta)-cos(0))/dis;

        }
        else
        {
            expBearing.bearings[i].x = (sin(agentTheta)-sin(nextAgentTheta))/dis;
            expBearing.bearings[i].y = (cos(agentTheta)-cos(nextAgentTheta))/dis;
        }

        mutualBearing.bearings[i].x = 0;
        mutualBearing.bearings[i].y = 0;
        targetBearing.bearings[i].x = 0;
        targetBearing.bearings[i].y = 0;
        allPositions[i].x = 0;
        allPositions[i].y = 0;
        allPositions[i].z = 0;
        expBearing.bearings[i+uavNum].x = sin(agentTheta);
        expBearing.bearings[i+uavNum].y = cos(agentTheta);
    }
}






