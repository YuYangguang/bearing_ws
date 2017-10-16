/*
 * formation.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#include "formation.h"




smarteye::Formation::Formation(int argc, char** argv, const char * name)
{
    systemID = -1;
    updateHz = 20;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "formation_node");
    nh = boost::make_shared<ros::NodeHandle>("~");
    bool IsGotParam;
    IsGotParam = nh->getParam("system_id",systemID);
    if(IsGotParam)
    {
        ROS_INFO("got system id successfully and id is %d",systemID);
    }
    else
    {
        ROS_ERROR("failed to get system id and node is shut down");
        exit(0);
    }
    std::string uavName;
    uavName = "/uav"+num2str(systemID);
    //    paramClient = nh->serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    //    mavros_msgs::ParamGet px4ParamSrv;
    //    int idGotCount = 1;
    //    ROS_INFO("attemp to get system id");
    //    while(systemID == -1)
    //    {
    //        px4ParamSrv.request.param_id="MAV_SYS_ID";
    //        if(paramClient.call(px4ParamSrv))
    //        {
    //            systemID = px4ParamSrv.response.value.integer;
    //            ROS_INFO("got system id successfully and id is %d",systemID);
    //        }
    //        idGotCount++;
    //        if((idGotCount%10) == 0)
    //        {
    //            ROS_ERROR("failed to get system id and node is shut down");
    //            exit(-1);
    //        }
    //    }
    setPositionPublisher = nh->advertise<geometry_msgs::PoseStamped>
            (uavName+"/mavros/setpoint_position/local",10);
    state_sub = nh->subscribe("/mavros/state", 10,&smarteye::Formation::ReceiveStateInfo, this);
    formationUpdateTimer = nh->createTimer(ros::Duration(updateTime),
                                           &Formation::update, this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>(uavName+"/mavros/cmd/arming");
    setModeClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/set_mode");
    takoffClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/cmd/takeoff");
    mutualBearingSub = nh->subscribe("/ygc/mutual_bearing",2,
                                     &smarteye::Formation::ReceiveMulBearing,this);
    targetBearingSub = nh->subscribe("/ygc/target_bearing",2,
                                     &smarteye::Formation::ReceiveMulBearing,this);
    keyboardSub= nh->subscribe("/keyboard/keydown",1,&Formation::ReceiveKeybdCmd,this);
    localPoseSub = nh->subscribe("mavros/local_position/pose", 10,
                                 &smarteye::Formation::ReceiveLocalPose,this);


}

smarteye::Formation::~Formation()
{

}

std::string smarteye::Formation::num2str(int i)
{
    std::stringstream ss;
    ss<<i;
    return ss.str();
}



void smarteye::Formation::ReceiveStateInfo(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void smarteye::Formation::ReceiveKeybdCmd(const keyboard::Key &key)
{
    switch(key.code)
    {
    case 'a':   //arm the vehicle
    {
        if(current_state.armed)
        {
            ROS_WARN("the vehicle is already armed!");
        }
        else
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            else
            {
                ROS_WARN("failed to arm vehicle");
            }
        }
        break;
    }
    case 'd':   //disarm
    {
        if(!current_state.armed)
        {
            ROS_WARN("the vehicle is already disarmed");
        }
        else
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
            else
            {
                ROS_WARN("failed to disarm vehicle");
            }

        }
        break;
    }
    case 'o':   //set offboard mode
    {
        mavros_msgs::SetMode setmodeCMD;
        setmodeCMD.request.custom_mode = "OFFBOARD";
        if(setModeClient.call(setmodeCMD) && setmodeCMD.response.success)
        {
            ROS_INFO("the mode of vehicle has changed to offboard");
        }
        else
        {
            ROS_WARN("failed to set offboard mode!  ");
        }
        break;
    }

    }
}

void smarteye::Formation::update(const ros::TimerEvent &event)
{
    positionSet.pose.position.x = 0;
    positionSet.pose.position.y = 0;
    positionSet.pose.position.z = 2;
    setPositionPublisher.publish(positionSet);

}

void smarteye::Formation::ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    localPose.pose.position.x = msg->pose.position.x;
    localPose.pose.position.y = msg->pose.position.y;
    localPose.pose.position.z = msg->pose.position.z;
    localPose.pose.orientation.x = msg->pose.orientation.x;
    localPose.pose.orientation.y = msg->pose.orientation.y;
    localPose.pose.orientation.z = msg->pose.orientation.z;
    localPose.pose.orientation.w = msg->pose.orientation.w;
    // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(localPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);

}

void smarteye::Formation::ReceiveMulBearing(const ygc::GroupBearingConstPtr &msg)
{
    mutualBearing = *msg;

}

void smarteye::Formation::ReceiveTarBearing(const ygc::GroupBearingConstPtr &msg)
{
    targetBearing = *msg;
}




