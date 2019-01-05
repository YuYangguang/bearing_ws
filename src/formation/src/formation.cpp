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
    ros::init(argc, argv, "");
    nh = boost::make_shared<ros::NodeHandle>("~");

    /*****初始化参数服务器************/
    initParamServer();
    /********初始化参数***************/
    uavState = YGC_HOVER;
    bool IsParamGot;
    bool IsAllParamGot = true;
    IsParamGot = ros::param::get("ENV_K_ALPHA",env_k_alpha);
    IsAllParamGot = IsAllParamGot && IsParamGot;
    IsParamGot = ros::param::get("ENV_K_BETA",env_k_beta);
    IsAllParamGot = IsAllParamGot && IsParamGot;
    IsParamGot = ros::param::get("HEI_KI",heiCtr.hei_ki);
    IsAllParamGot = IsAllParamGot && IsParamGot;
    IsParamGot = ros::param::get("HEI_KP",heiCtr.hei_kp);
    IsAllParamGot = IsAllParamGot && IsParamGot;
    IsParamGot = ros::param::get("HEI_KD",heiCtr.hei_kd);
    IsAllParamGot = IsAllParamGot && IsParamGot;
    if (IsAllParamGot == true)
    {
        ROS_INFO("Get param successfully");
    }
    else
    {
        ROS_ERROR("Fail to get param");
    }
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
    setPositionPub = nh->advertise<geometry_msgs::PoseStamped>
            (uavName+"/mavros/setpoint_position/local",10);
    px4StateSub = nh->subscribe(uavName+"/mavros/state", 10,&smarteye::Formation::ReceiveStateInfo, this);
    formationUpdateTimer = nh->createTimer(ros::Duration(updateTime),
                                           &Formation::update, this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>(uavName+"/mavros/cmd/arming");
    setModeClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/set_mode");
    takoffClient = nh->serviceClient<mavros_msgs::SetMode>(uavName+"/mavros/cmd/takeoff");
    mutualBearingSub = nh->subscribe("/ygc/mutual_bearing",2,
                                     &smarteye::Formation::ReceiveMulBearing,this);
    targetBearingSub = nh->subscribe("/ygc/target_bearing",2,
                                     &smarteye::Formation::ReceiveTarBearing,this);
    expBearingSub = nh->subscribe("/ygc/expected_bearing",2,
                                  &smarteye::Formation::ReceiveExpBearing,this);
    keyboardSub= nh->subscribe("/keyboard/keydown",1,&Formation::ReceiveKeybdCmd,this);
    localPoseSub = nh->subscribe(uavName+"/mavros/local_position/pose", 10,
                                 &smarteye::Formation::ReceiveLocalPose,this);
    setVelPub = nh->advertise<geometry_msgs::TwistStamped>(uavName+"/mavros/setpoint_velocity/cmd_vel", 10);
    positionSet = localPose;
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
    px4State = *msg;
}

void smarteye::Formation::ReceiveKeybdCmd(const keyboard::Key &key)
{

    switch(key.code)
    {
    case 'a':   //arm the vehicle
    {
        if(px4State.armed)
        {
            ROS_WARN("the vehicle %d is already armed!",systemID);
        }
        else
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle %d armed",systemID);
            }
            else
            {
                ROS_WARN("failed to arm vehicle %d",systemID);
            }
        }
        break;
    }
    case 'd':   //disarm
    {
        if(!px4State.armed)
        {
            ROS_WARN("the vehicle %d is already disarmed",systemID);
        }
        else
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle %d disarmed",systemID);
            }
            else
            {
                ROS_WARN("failed to disarm vehicle %d",systemID);
            }

        }
        break;
    }
    case 'o':   //set offboard mode
    {
        positionSet = localPose;
        uavState = YGC_HOVER;
        mavros_msgs::SetMode setmodeCMD;
        setmodeCMD.request.custom_mode = "OFFBOARD";
        if(setModeClient.call(setmodeCMD) && setmodeCMD.response.mode_sent)
        {
            ROS_INFO("the mode of vehicle %d has changed to offboard",systemID);
        }
        else
        {
            ROS_WARN("vehicle %d failed to set offboard mode!",systemID);
        }
        break;
    }
    case 't':     //takeoff
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d is going to take off",systemID);
        if(localPose.pose.position.z > 0.25)
        {
            ROS_WARN("the vehicle %d is already takeoff!",systemID);
        }
        else
        {
            positionSet.pose.position.x = localPose.pose.position.x;
            positionSet.pose.position.y = localPose.pose.position.y;
            uavState = YGC_TAKEOFF;
        }
        break;
    }
    case 'l':    //land
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d is going to land",systemID);
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        uavState = YGC_LAND;
        break;
    }
    case 'h':  //hover
    {
        heiCtr.ei = 0;
        ROS_INFO("vehicle %d mode is changed to hover",systemID);
        positionSet = localPose;
        uavState = YGC_HOVER;
        break;
    }
    case 'e':  //encirlcement
    {
        ROS_INFO("vehicle %d begins enciclement control",systemID);
        uavState = YGC_ENCIRCLE;
        break;
    }
    default:
    {
        ROS_WARN("vehicle %d receives unknown command!",systemID);
    }

    }
}

void smarteye::Formation::update(const ros::TimerEvent &event)
{

    // ROS_INFO("kp is %f,ki is %f,kd is %f, bias is %f",heiCtr.hei_kp,heiCtr.hei_ki,heiCtr.hei_kd,heiCtr.hei_bias);

    switch (uavState)
    {
    case YGC_HOVER:
    {
        setPositionPub.publish(positionSet);
        break;
    }
    case YGC_TAKEOFF:
    {
        takeoffCtr();
        break;
    }
    case YGC_LAND:
    {
        landCtr();
        break;
    }
    case YGC_ENCIRCLE:
    {
        encircleCtr(1.5);
        break;
    }
    default:
    {
        ROS_WARN("Unexpected state occurs in vehicle %d",systemID);
        break;
    }
    }
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

void smarteye::Formation::ReceiveExpBearing(const ygc::GroupBearingConstPtr &msg)
{
    expBearing = *msg;
}

void smarteye::Formation::initParamServer()
{
    ros::param::set("ENV_K_ALPHA", 5);
    ros::param::set("ENV_K_BETA", 5);
    ros::param::set("HEI_KP", 2);
    ros::param::set("HEI_KI", 0.02);
    ros::param::set("HEI_KD", 0.3);
    ros::param::set("HEI_BIAS",0.07);
}

void smarteye::Formation::takeoffCtr()
{
    //    if(localPose.pose.position.z<1.2)
    //    {
    //        positionSet.pose.position.z = localPose.pose.position.z+0.15;
    //    }
    //    else
    //    {
    //        positionSet.pose.position.z = 1.5;
    //    }
    positionSet.pose.position.z = 1.5;
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::landCtr()
{
    if(localPose.pose.position.z>0.3)
    {
        positionSet.pose.position.z = localPose.pose.position.z-0.15;
    }
    else
    {
        positionSet.pose.position.z = 0;
    }
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::encircleCtr(double targetHei)
{
    if(mutualBearing.bearings.size()!=0)
    {
        int uavNum = mutualBearing.bearings.size();
        //ROS_INFO("uavNum is %d",uavNum);
        Eigen::Vector2d nextBearing;  //和后一个智能体的方位
        Eigen::Vector2d preBearing;   //和前一个智能体的方位
        Eigen::Vector2d nextBea_star;  //和后一个智能体的期望方位
        Eigen::Vector2d preBea_star;   //和前一个智能体的期望方位
        Eigen::Vector2d tbearing;   //和目标的方位
        Eigen::Vector2d tbearing_star;   //和目标的方位
        Eigen::Matrix2d Id;
        Id <<1,0,
                0,1;
        if(systemID == 1)
        {
            nextBearing[0] = mutualBearing.bearings[systemID-1].x;
            nextBearing[1] = mutualBearing.bearings[systemID-1].y;
            preBearing[0] = mutualBearing.bearings[uavNum-1].x;
            preBearing[1] = mutualBearing.bearings[uavNum-1].y;
            nextBea_star[0] = expBearing.bearings[systemID-1].x;
            nextBea_star[1] = expBearing.bearings[systemID-1].y;
            preBea_star[0] = expBearing.bearings[uavNum-1].x;
            preBea_star[1] = expBearing.bearings[uavNum-1].y;
        }
        else
        {
            nextBearing[0] = mutualBearing.bearings[systemID-1].x;
            nextBearing[1] = mutualBearing.bearings[systemID-1].y;
            preBearing[0] = mutualBearing.bearings[systemID-2].x;
            preBearing[1] = mutualBearing.bearings[systemID-2].y;
            nextBea_star[0] = expBearing.bearings[systemID-1].x;
            nextBea_star[1] = expBearing.bearings[systemID-1].y;
            preBea_star[0] = expBearing.bearings[systemID-2].x;
            preBea_star[1] = expBearing.bearings[systemID-2].y;
        }
        tbearing[0] = targetBearing.bearings[systemID-1].x;
        tbearing[1] = targetBearing.bearings[systemID-1].y;
        tbearing_star[0] = expBearing.bearings[systemID-1+uavNum].x;
        tbearing_star[1] = expBearing.bearings[systemID-1+uavNum].y;
        Eigen::Vector2d ctrOutput;  //控制输出
        Eigen::Vector2d outFromNei;  //从邻居智能体得到的控制分量
        Eigen::Vector2d outFromTar;  //从目标得到的控制分量
        Eigen::Vector2d temp2 = (Id-preBea_star*preBea_star.transpose())*preBearing;
        Eigen::Vector2d temp1 = (Id-nextBea_star*nextBea_star.transpose())*nextBearing;
        outFromNei = env_k_alpha*(temp2 - temp1);
        //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
        //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
        //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
        //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
        outFromTar = -env_k_beta*(Id-tbearing_star*tbearing_star.transpose())*tbearing;
        ctrOutput = outFromNei + outFromTar;

        heiCtr.currentHei = localPose.pose.position.z;
        heiCtr.targetHei = targetHei;
        velocitySet.twist.linear.z = heiCtr.cacOutput();
//        if(systemID == 1)
//        {
//            ROS_INFO("current bearing: x is %f, y is %f",tbearing[0],tbearing[1]);
//            ROS_INFO("desired bearing: x is %f, y is %f",tbearing_star[0],tbearing_star[1]);
//            ROS_INFO("output: x is %f,y is %f",outFromTar[0],outFromTar[1]);
//            ROS_INFO("current position: x is %f, y is %f",localPose.pose.position.x,localPose.pose.position.y+10);
//        }
        Eigen::Matrix2d rotateMatrix;
        float rotTheta;
        ros::param::get("/dynamic/ROTT",rotTheta);
        rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                sin(rotTheta),cos(rotTheta);
        ctrOutput = rotateMatrix*ctrOutput;
        velocitySet.twist.linear.x = ctrOutput[0];
        velocitySet.twist.linear.y = ctrOutput[1];
//        velocitySet.twist.linear.x = 1;
//        velocitySet.twist.linear.y = 0;
        setVelPub.publish(velocitySet);
    }
    else
    {
        ROS_WARN("there is no bearing info found!");
        setPositionPub.publish(positionSet);
    }

}



smarteye::heiCtrller::heiCtrller()
{

}

smarteye::heiCtrller::~heiCtrller()
{

}

float smarteye::heiCtrller::cacOutput()
{
    ros::param::get("/dynamic/HEI_KI",hei_ki);
    ros::param::get("/dynamic/HEI_KP",hei_kp);
    ros::param::get("/dynamic/HEI_KD",hei_kd);
    ros::param::get("/dynamic/HEI_BIAS",hei_bias);
    float output;
    float err = targetHei - currentHei;
    ei = ei + hei_ki*err;
    ei = ValueLimit(ei,0.3,-0.3);
    output = hei_bias + hei_kp*err + ei + hei_kd*(err-previousErr);
    output = ValueLimit(output,1,-1);
    previousErr = err;
    return(output);
}


float smarteye::ValueLimit(float value, float max, float min)
{
    if(value > max)
        return max;
    else if(value < min)
        return min;
    else
        return value;
}
