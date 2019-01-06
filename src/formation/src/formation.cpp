/*
 * formation.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: zeroun
 */

#include "formation.h"




smarteye::Formation::Formation(int argc, char** argv, const char * name)
{
    sleep(1);
    systemID = -1;
    updateHz = 20;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "");
    nh = boost::make_shared<ros::NodeHandle>("~");
    std::string viconName;
    nh->getParam("viconName", viconName);
    /********初始化参数***************/
    uavState = YGC_HOVER;

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
    //后面此处应加入uav数量
    std::string neighUAVname;
    int neighUAVID;
<<<<<<< HEAD
    if(systemID == 3)
=======
    if(systemID == 5)
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    {
        neighUAVID = 1;
    }
    else
    {
        neighUAVID = systemID+1;
    }
    neighUAVname = "/uav"+num2str(neighUAVID);
    setPositionPub = nh->advertise<geometry_msgs::PoseStamped>
            (uavName+"/mavros/setpoint_position/local",10);
    px4StateSub = nh->subscribe(uavName+"/mavros/state", 10,&smarteye::Formation::ReceiveStateInfo, this);
    formationUpdateTimer = nh->createTimer(ros::Duration(updateTime),
                                           &Formation::update, this);
    viconUpdateTimer = nh->createTimer(ros::Duration(0.03),
                                       &Formation::viconUpdate, this);
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
<<<<<<< HEAD
    //    localPoseSub = nh->subscribe(uavName+"/mavros/local_position/pose", 10,
    //                                 &smarteye::Formation::ReceiveLocalPose,this);
=======
    localPoseSub = nh->subscribe(uavName+"/mavros/local_position/pose", 10,
                                 &smarteye::Formation::ReceiveLocalPose,this);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    targetInfoSub = nh->subscribe("/ygc/targetPose",2,
                                  &smarteye::Formation::ReceiveTargetInfo,this);
    setVelPub = nh->advertise<geometry_msgs::TwistStamped>(uavName+"/mavros/setpoint_velocity/cmd_vel", 10);
    allPoseSub = nh->subscribe("/ygc/allPosition",3,&smarteye::Formation::ReceiveAllPose,this);
<<<<<<< HEAD
    //dotBearingSub = nh->subscribe("/ygc/dot_bearing",3,&smarteye::Formation::ReceiveDotBearing,this);
    //    selfVelSub = nh->subscribe(uavName+"/mavros/local_position/velocity", 10,
    //                               &smarteye::Formation::ReceiveSelfVel,this);
    //    neighborVelSub = nh->subscribe(neighUAVname+"/mavros/local_position/velocity", 10,
    //                                   &smarteye::Formation::ReceiveNeighborVel,this);
    //agentVelSub = nh->subscribe("/ygc/agentVel",3,&smarteye::Formation::ReceiveAgentVel,this);
    viconPositionSubsciber = nh->subscribe(viconName, 10, &smarteye::Formation::viconPositionReceived,this);
    currentViconPositionPublisher = nh->advertise<geometry_msgs::PoseStamped>(uavName+"/mavros/mocap/pose",10);
    receiveHParamSrv = nh->advertiseService(uavName+"/formation/host_param/set_param"
                                            ,&smarteye::Formation::receiveHParamSetSrv,this);
    InitParam();
=======
    dotBearingSub = nh->subscribe("/ygc/dot_bearing",3,&smarteye::Formation::ReceiveDotBearing,this);
    positionSet = localPose;
    selfVelSub = nh->subscribe(uavName+"/mavros/local_position/velocity", 10,
                               &smarteye::Formation::ReceiveSelfVel,this);
    neighborVelSub = nh->subscribe(neighUAVname+"/mavros/local_position/velocity", 10,
                                   &smarteye::Formation::ReceiveNeighborVel,this);
    agentVelSub = nh->subscribe("/ygc/agentVel",3,&smarteye::Formation::ReceiveAgentVel,this);
    viconPositionSubsciber = nh->subscribe(viconName, 10, &smarteye::Formation::viconPositionReceived,this);
    currentViconPositionPublisher = nh->advertise<geometry_msgs::PoseStamped>(uavName+"/mavros/mocap/pose",10);
    timecount = 0;
    rotTheta = -0.63;
    receiveHParamSrv = nh->advertiseService("/formation/host_param/set_param"
                                            ,&smarteye::Formation::receiveHParamSetSrv,this);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc

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

bool smarteye::Formation::receiveHParamSetSrv(bearing_common::HParamSetSrv::Request &req, bearing_common::HParamSetSrv::Response &res)
{
    ROS_INFO("I've received new Param: name: %s value: %f",req.param_id.c_str(),req.param_value);
    res.paramset_ack = true;
    bool isAck = true;
    //    if(timecount==1)
    //    {
    //        ros::param::get("/dynamic/HEI_KI",heiCtr.hei_ki);
    //    }
    //    if(timecount==3)
    //    {
    //        ros::param::get("/dynamic/HEI_KP",heiCtr.hei_kp);
    //    }
    //    if(timecount == 6)
    //    {
    //        ros::param::get("/dynamic/HEI_KD",heiCtr.hei_kd);
    //    }
    //    if(timecount == 9)
    //    {
    //        ros::param::get("/dynamic/HEI_BIAS",heiCtr.hei_bias);
    //    }
    //    if(timecount==12)
    //    {
    //        ros::param::get("/dynamic/XY_KI",xCtr.xy_ki);
    //        yCtr.xy_ki = xCtr.xy_ki;
    //    }
    //    if(timecount==15)
    //    {
    //        ros::param::get("/dynamic/XY_KP",xCtr.xy_kp);
    //        yCtr.xy_kp = xCtr.xy_kp;
    //    }
    //    if(timecount==18)
    //    {
    //        ros::param::get("/dynamic/XY_KD",xCtr.xy_kd);
    //        yCtr.xy_kd = xCtr.xy_kd;
    //    }
    //    if(timecount>25)
    //    {
    //        ros::param::get("/dynamic/XY_BIAS",xCtr.xy_bias);
    //        timecount = 0;
    //        yCtr.xy_bias = xCtr.xy_bias;
    //    }
    if(req.param_id == "HEI_KI")
        heiCtr.hei_ki =req.param_value;
<<<<<<< HEAD
    else if(req.param_id == "HEI_KP1")
        heiCtr.hei_kp1 =req.param_value;
    else if(req.param_id == "HEI_KP2")
        heiCtr.hei_kp2 =req.param_value;
    else if(req.param_id == "HEI_KPDIV")
        heiCtr.hei_kpdiv =req.param_value;
    else if(req.param_id == "HEI_KD")
        heiCtr.hei_kd =req.param_value;
=======
    else if(req.param_id == "HEI_KP")
        heiCtr.hei_kp =req.param_value;
    else if(req.param_id == "HEI_KD")
        heiCtr.hei_kp =req.param_value;
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    else if(req.param_id == "HEI_BIAS")
        heiCtr.hei_bias =req.param_value;
    else if(req.param_id == "XY_BIAS")
    {
        xCtr.xy_bias = req.param_value;
        yCtr.xy_bias = req.param_value;
    }
    else if(req.param_id == "XY_KP")
    {
        xCtr.xy_kp = req.param_value;
        yCtr.xy_kp = req.param_value;
    }
    else if(req.param_id == "XY_KI")
    {
        xCtr.xy_ki = req.param_value;
        yCtr.xy_ki = req.param_value;
    }
    else if(req.param_id == "XY_KD")
    {
        xCtr.xy_kd = req.param_value;
        yCtr.xy_kd = req.param_value;
    }
    else if(req.param_id == "ROTT")
    {
        rotTheta = req.param_value;
    }
    else if(req.param_id == "ENV_K_ALPHA")
    {
        env_k_alpha = req.param_value;
    }
    else if(req.param_id == "ENV_K_BETA")
    {
        env_k_beta = req.param_value;
    }
<<<<<<< HEAD
    else if(req.param_id == "ENV_K_GAMMA")
    {
        env_k_gamma = req.param_value;
    }
=======
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    else
    {
        ROS_WARN("Unknown param set request received");
        res.paramset_ack = false;
        return false;
    }
    res.paramset_ack = isAck;
    return true;
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
            positionSet = localPose;
            positionSet.pose.position.z = -1;
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
        initialHeight = localPose.pose.position.z;
        ROS_INFO("initial height is %f",initialHeight);
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
            positionSet = localPose;
            positionSet.pose.position.z = -1;
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
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
        positionSet.pose.position.z = -1;
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
        if(localPose.pose.position.z > 0.4)
        {
            ROS_WARN("the vehicle %d is already takeoff!",systemID);
        }
        else
        {
            positionSet.pose.position.x = localPose.pose.position.x;
            positionSet.pose.position.y = localPose.pose.position.y;
            positionSet.pose.orientation.w = localPose.pose.orientation.w;
            positionSet.pose.orientation.x = localPose.pose.orientation.x;
            positionSet.pose.orientation.y = localPose.pose.orientation.y;
            positionSet.pose.orientation.z = localPose.pose.orientation.z;
            ROS_INFO("current x is %f",localPose.pose.position.x);
            ROS_INFO("current y is %f",localPose.pose.position.y);
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
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        ROS_INFO("current x is %f",localPose.pose.position.x);
        ROS_INFO("current y is %f",localPose.pose.position.y);
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
    case 'c':
    {
        ROS_INFO("vehicle %d begins circle control",systemID);
        uavState = YGC_CIRCLE;
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

    //    timecount++;

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
        positionSet.pose.position.x = localPose.pose.position.x;
        positionSet.pose.position.y = localPose.pose.position.y;
        positionSet.pose.orientation.w = localPose.pose.orientation.w;
        positionSet.pose.orientation.x = localPose.pose.orientation.x;
        positionSet.pose.orientation.y = localPose.pose.orientation.y;
        positionSet.pose.orientation.z = localPose.pose.orientation.z;
        landCtr();
        break;
    }
    case YGC_ENCIRCLE:
    {
<<<<<<< HEAD
        encircleCtr(1.3+initialHeight);
=======
        encircleCtr(1);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
        break;
    }
    case YGC_CIRCLE:
    {
<<<<<<< HEAD
        circleCtr(1.3+initialHeight);
=======
        circleCtr(1);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
        break;
    }
    default:
    {
        ROS_WARN("Unexpected state occurs in vehicle %d",systemID);
        break;
    }
    }
}

<<<<<<< HEAD
void smarteye::Formation::viconUpdate(const ros::TimerEvent &event)
=======


void smarteye::Formation::ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr &msg)
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
{
    uavCurrentViconPose.header.stamp = ros::Time::now();
    uavCurrentViconPose.header.frame_id = "/world";
    currentViconPositionPublisher.publish(uavCurrentViconPose);
}

<<<<<<< HEAD
//void smarteye::Formation::ReceiveLocalPose(const geometry_msgs::PoseStampedConstPtr &msg)
//{
//    localPose.pose.position.x = msg->pose.position.x;
//    localPose.pose.position.y = msg->pose.position.y;
//    localPose.pose.position.z = msg->pose.position.z;
//    localPose.pose.orientation.x = msg->pose.orientation.x;
//    localPose.pose.orientation.y = msg->pose.orientation.y;
//    localPose.pose.orientation.z = msg->pose.orientation.z;
//    localPose.pose.orientation.w = msg->pose.orientation.w;
//    // Using ROS tf to get RPY angle from Quaternion
//    tf::Quaternion quat;
//    tf::quaternionMsgToTF(localPose.pose.orientation, quat);
//    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);

//}

=======
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
void smarteye::Formation::ReceiveMulBearing(const bearing_common::GroupBearingConstPtr &msg)
{
    mutualBearing = *msg;

}

void smarteye::Formation::ReceiveTarBearing(const bearing_common::GroupBearingConstPtr &msg)
{
    targetBearing = *msg;
}

void smarteye::Formation::ReceiveExpBearing(const bearing_common::GroupBearingConstPtr &msg)
{
    expBearing = *msg;
}

void smarteye::Formation::ReceiveTargetInfo(const nav_msgs::OdometryConstPtr &msg)
{
    targetInfo = *msg;
}

void smarteye::Formation::ReceiveAllPose(const bearing_common::AllPositionConstPtr &msg)
{
    allUavPos = *msg;
}

void smarteye::Formation::ReceiveDotBearing(const bearing_common::GroupBearingConstPtr &msg)
{
    dotBearing = *msg;
}

void smarteye::Formation::ReceiveSelfVel(const geometry_msgs::TwistStampedConstPtr &msg)
<<<<<<< HEAD
{
    selfVel = *msg;
}

void smarteye::Formation::ReceiveNeighborVel(const geometry_msgs::TwistStampedConstPtr &msg)
{
=======
{
    selfVel = *msg;
}

void smarteye::Formation::ReceiveNeighborVel(const geometry_msgs::TwistStampedConstPtr &msg)
{
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    neighborVel = *msg;
}

void smarteye::Formation::ReceiveAgentVel(const bearing_common::GroupBearingConstPtr &msg)
{
    preAgeVelTruth = agentVelTruth;
    agentVelTruth = *msg;
}

void smarteye::Formation::viconPositionReceived(const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
<<<<<<< HEAD

    uavCurrentViconPose.pose.position.x = vicon_msg->transform.translation.x;
    uavCurrentViconPose.pose.position.y = vicon_msg->transform.translation.y;
    uavCurrentViconPose.pose.position.z = vicon_msg->transform.translation.z;
    uavCurrentViconPose.pose.orientation.w = vicon_msg->transform.rotation.w;
    uavCurrentViconPose.pose.orientation.x = vicon_msg->transform.rotation.x;
    uavCurrentViconPose.pose.orientation.y = vicon_msg->transform.rotation.y;
    uavCurrentViconPose.pose.orientation.z = vicon_msg->transform.rotation.z;
=======
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    localPose.pose.position.x = vicon_msg->transform.translation.x;
    localPose.pose.position.y = vicon_msg->transform.translation.y;
    localPose.pose.position.z = vicon_msg->transform.translation.z;
    localPose.pose.orientation.w = vicon_msg->transform.rotation.w;
    localPose.pose.orientation.x = vicon_msg->transform.rotation.x;
    localPose.pose.orientation.y = vicon_msg->transform.rotation.y;
    localPose.pose.orientation.z = vicon_msg->transform.rotation.z;
}

void smarteye::Formation::EstTarDis()
{
    Eigen::Matrix2d PgiT;
    Eigen::Matrix2d PgjT;
    Eigen::Matrix2d Pgij;
    Eigen::Vector2d tbearing;   //和目标的方位
    Eigen::Vector2d nextTbearing;   //邻居智能体和目标的方位
    Eigen::Vector2d gij;   //和邻居智能体的方位
    Eigen::Vector2d vi;
    Eigen::Vector2d vj;
    Eigen::Matrix2d Id;
    Eigen::Vector2d dotGij;
    Eigen::Vector2d vT;
    vT[0] = targetInfo.twist.twist.linear.x;
    vT[1] = targetInfo.twist.twist.linear.y;
    Id <<1,0,
            0,1;
    if(systemID == 5)
    {
        nextTbearing[0] = targetBearing.bearings[0].x;
        nextTbearing[1] = targetBearing.bearings[0].y;
        vj[0] = agentVelTruth.bearings[0].x;
        vj[1] = agentVelTruth.bearings[0].y;
        //        vj[0] =neighborVel.twist.linear.x;
        //        vj[1] =neighborVel.twist.linear.y;
    }
    else
    {
        vj[0] = agentVelTruth.bearings[systemID].x;
        vj[1] = agentVelTruth.bearings[systemID].y;
        //        vj[0] =neighborVel.twist.linear.x;
        //        vj[1] =neighborVel.twist.linear.y;
        nextTbearing[0] = targetBearing.bearings[systemID].x;
        nextTbearing[1] = targetBearing.bearings[systemID].y;
    }
    PgjT = Id-nextTbearing*nextTbearing.transpose();
    tbearing[0] = targetBearing.bearings[systemID-1].x;
    tbearing[1] = targetBearing.bearings[systemID-1].y;
    PgiT = Id-tbearing*tbearing.transpose();
    gij[0] = mutualBearing.bearings[systemID-1].x;
    gij[1] = mutualBearing.bearings[systemID-1].y;
    Pgij = Id-gij*gij.transpose();
    dotGij[0] = dotBearing.bearings[systemID-1].x;
    dotGij[1] = dotBearing.bearings[systemID-1].y;
    vi[0] = agentVelTruth.bearings[systemID-1].x;
    vi[1] = agentVelTruth.bearings[systemID-1].y;
    //    vi[0] = selfVel.twist.linear.x;
    //    vi[1] = selfVel.twist.linear.y;

    if(CacVector2DNorm(dotGij)<1E-5)
    {
        //dis2TarEst[] = -1;
        ROS_WARN("cannot caculate distance systemID is %d",systemID);
    }
    else
    {


        int zeroNum = 0;
        for(int k=0;k<4;k++)
        {
            if(dis2TarEst[k+1]<1E-2)   //看数组里面空的个数
                zeroNum++;
        }
        float temp = CacMatrix2DNorm(PgiT-Pgij)*CacVector2DNorm(Pgij*(vi-vj))/
                (CacMatrix2DNorm(PgjT-PgiT)*CacVector2DNorm(dotGij));
        if(zeroNum != 0)
        {
            for(int k=0;k<5;k++)
            {
                dis2TarEst[k+1]=dis2TarEst[k+2];
            }
            dis2TarEst[6] = temp;
            dis2TarEst[0] = ( dis2TarEst[2]+dis2TarEst[3]+dis2TarEst[4]+dis2TarEst[5]+dis2TarEst[6])/(6-zeroNum);
        }
        else   //此时数据比较多，比较稳定了，可以进行滤波，对差异较大的数据滤出去
        {
            float rspeed =tbearing[0]*(vi[0]-vT[0])+tbearing[1]*(vi[1]-vT[1]);
            if(abs(temp-dis2TarEst[0])<1.5+4*rspeed*rspeed)
            {
                for(int k=0;k<5;k++)
                {
                    dis2TarEst[k+1]=dis2TarEst[k+2];
                }
                if(systemID ==2)
                {
                    ROS_INFO("speed is %f",rspeed);
                }
                dis2TarEst[6] = temp;
                dis2TarEst[0] =( dis2TarEst[2]+dis2TarEst[3]+dis2TarEst[4]+dis2TarEst[5]+dis2TarEst[6])/5;
            }
        }

        if(systemID ==2)
        {
            ROS_WARN("1 is %f,2 is %f, 3 is %f, 4 is %f,5 is %f",dis2TarEst[1],dis2TarEst[2]
                    ,dis2TarEst[3],dis2TarEst[4],dis2TarEst[5]);
        }

    }



}

<<<<<<< HEAD

void smarteye::Formation::takeoffCtr()
{
    if(localPose.pose.position.z-initialHeight<0.8)
    {
        positionSet.pose.position.z = localPose.pose.position.z+0.15+initialHeight;
    }
    else
    {
        positionSet.pose.position.z = 1+initialHeight;
=======
//void smarteye::Formation::ReceiveTargetInfo(const nav_msgs::OdometryConstPtr &msg)
//{
//    targetInfo = *msg;
//}

//void smarteye::Formation::initParamServer()
//{
//    ros::param::set("ENV_K_ALPHA", 5);
//    ros::param::set("ENV_K_BETA", 5);
//    ros::param::set("HEI_KP", 2);
//    ros::param::set("HEI_KI", 0.02);
//    ros::param::set("HEI_KD", 0.3);
//    ros::param::set("HEI_BIAS",0.07);
//}

void smarteye::Formation::takeoffCtr()
{
    if(localPose.pose.position.z<0.7)
    {
        positionSet.pose.position.x = 0;
        positionSet.pose.position.y = 0;
        positionSet.pose.position.z = localPose.pose.position.z+0.15;
    }
    else
    {
        positionSet.pose.position.x = 0;
        positionSet.pose.position.y = 0;
        positionSet.pose.position.z = 1;
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    //    positionSet.pose.position.z = 1.5;
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::landCtr()
{
    if(localPose.pose.position.z>0.3)
    {
<<<<<<< HEAD

        positionSet.pose.position.z = localPose.pose.position.z-0.2;
    }
    else
    {
        positionSet.pose.position.z = -0.5;
=======
        positionSet.pose.position.x = 0;
        positionSet.pose.position.y = 0;
        positionSet.pose.position.z = localPose.pose.position.z-0.15;
    }
    else
    {
        positionSet.pose.position.x = 0;
        positionSet.pose.position.y = 0;
        positionSet.pose.position.z = 0;
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    setPositionPub.publish(positionSet);

}

void smarteye::Formation::encircleCtr(double targetHei)
{
<<<<<<< HEAD
//    heiCtr.currentHei = localPose.pose.position.z;
//    heiCtr.targetHei = targetHei;
//    velocitySet.twist.linear.z = heiCtr.cacOutput();
//    xCtr.currentPos = localPose.pose.position.x;
//    xCtr.targetPos = 0;
//    velocitySet.twist.linear.x = xCtr.cacOutput();

//    yCtr.currentPos = localPose.pose.position.y;
//    yCtr.targetPos = 0;
//    velocitySet.twist.linear.y = yCtr.cacOutput();
//    velocitySet.twist.angular.x = 0;
//    velocitySet.twist.angular.y = 0;
//    velocitySet.twist.angular.z = 0;
//    setVelPub.publish(velocitySet);
    if(systemID == 0)
=======
    heiCtr.currentHei = localPose.pose.position.z;
    heiCtr.targetHei = targetHei;
    velocitySet.twist.linear.z = heiCtr.cacOutput();
    xCtr.currentPos = localPose.pose.position.x;
    xCtr.targetPos = 0;
    velocitySet.twist.linear.x = xCtr.cacOutput();

    yCtr.currentPos = localPose.pose.position.y;
    yCtr.targetPos = 0;
    velocitySet.twist.linear.y = yCtr.cacOutput();
    velocitySet.twist.angular.x = 0;
    velocitySet.twist.angular.y = 0;
    velocitySet.twist.angular.z = 0;
    setVelPub.publish(velocitySet);
    //    if(systemID == 0)
    //    {
    //        velocitySet.twist.linear.x = 0.3;
    //        velocitySet.twist.linear.y = 0.3;
    //        setVelPub.publish(velocitySet);
    //    }
    //    else
    //    {
    //        //EstTarDis();
    //        if(mutualBearing.bearings.size()!=0)
    //        {
    //            int uavNum = mutualBearing.bearings.size();
    //            //ROS_INFO("uavNum is %d",uavNum);
    //            Eigen::Vector2d nextBearing;  //和后一个智能体的方位
    //            Eigen::Vector2d preBearing;   //和前一个智能体的方位
    //            Eigen::Vector2d nextBea_star;  //和后一个智能体的期望方位
    //            Eigen::Vector2d preBea_star;   //和前一个智能体的期望方位
    //            Eigen::Vector2d tbearing;   //和目标的方位
    //            Eigen::Vector2d tbearing_star;   //和目标的方位
    //            Eigen::Matrix2d Id;
    //            Id <<1,0,
    //                    0,1;
    //            if(systemID == 1)
    //            {
    //                nextBearing[0] = mutualBearing.bearings[systemID-1].x;
    //                nextBearing[1] = mutualBearing.bearings[systemID-1].y;
    //                preBearing[0] = mutualBearing.bearings[uavNum-1].x;
    //                preBearing[1] = mutualBearing.bearings[uavNum-1].y;
    //                nextBea_star[0] = expBearing.bearings[systemID-1].x;
    //                nextBea_star[1] = expBearing.bearings[systemID-1].y;
    //                preBea_star[0] = expBearing.bearings[uavNum-1].x;
    //                preBea_star[1] = expBearing.bearings[uavNum-1].y;
    //            }
    //            else
    //            {
    //                nextBearing[0] = mutualBearing.bearings[systemID-1].x;
    //                nextBearing[1] = mutualBearing.bearings[systemID-1].y;
    //                preBearing[0] = mutualBearing.bearings[systemID-2].x;
    //                preBearing[1] = mutualBearing.bearings[systemID-2].y;
    //                nextBea_star[0] = expBearing.bearings[systemID-1].x;
    //                nextBea_star[1] = expBearing.bearings[systemID-1].y;
    //                preBea_star[0] = expBearing.bearings[systemID-2].x;
    //                preBea_star[1] = expBearing.bearings[systemID-2].y;
    //            }
    //            tbearing[0] = targetBearing.bearings[systemID-1].x;
    //            tbearing[1] = targetBearing.bearings[systemID-1].y;
    //            tbearing_star[0] = expBearing.bearings[systemID-1+uavNum].x;
    //            tbearing_star[1] = expBearing.bearings[systemID-1+uavNum].y;
    //            Eigen::Vector2d ctrOutput;  //控制输出
    //            Eigen::Vector2d outFromNei;  //从邻居智能体得到的控制分量
    //            Eigen::Vector2d outFromTar;  //从目标得到的控制分量
    //            Eigen::Vector2d temp2 = (Id-preBea_star*preBea_star.transpose())*preBearing;
    //            Eigen::Vector2d temp1 = (Id-nextBea_star*nextBea_star.transpose())*nextBearing;
    //            outFromNei = env_k_alpha*(temp2 - temp1);
    //            //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
    //            //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
    //            //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
    //            //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
    //            outFromTar = -env_k_beta*(Id-tbearing_star*tbearing_star.transpose())*tbearing;
    //            ctrOutput = outFromNei + outFromTar;

    //            heiCtr.currentHei = localPose.pose.position.z;
    //            heiCtr.targetHei = targetHei;
    //            velocitySet.twist.linear.z = heiCtr.cacOutput();
    //            //        if(systemID == 1)
    //            //        {
    //            //            ROS_INFO("current bearing: x is %f, y is %f",tbearing[0],tbearing[1]);
    //            //            ROS_INFO("desired bearing: x is %f, y is %f",tbearing_star[0],tbearing_star[1]);
    //            //            ROS_INFO("output: x is %f,y is %f",outFromTar[0],outFromTar[1]);
    //            //            ROS_INFO("current position: x is %f, y is %f",localPose.pose.position.x,localPose.pose.position.y+10);
    //            //        }
    //            Eigen::Matrix2d rotateMatrix;
    //            rotateMatrix <<cos(rotTheta),-sin(rotTheta),
    //                    sin(rotTheta),cos(rotTheta);
    //            ctrOutput = rotateMatrix*ctrOutput;
    //            velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x;
    //            velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y;
    //            //        velocitySet.twist.linear.x = 1;
    //            //        velocitySet.twist.linear.y = 0;
    //            if(systemID == 0)
    //            {
    //                velocitySet.twist.linear.x = 0.1;
    //                velocitySet.twist.linear.y = 0.1;
    //            }
    //            setVelPub.publish(velocitySet);

    //        }
    //        else
    //        {
    //            ROS_WARN("there is no bearing info found!");
    //            setPositionPub.publish(positionSet);
    //        }
    //    }

}

void smarteye::Formation::circleCtr(double targetHei)
{


    if(systemID == 0)  //目标控制
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    {
        velocitySet.twist.linear.x = 0.3;
        velocitySet.twist.linear.y = 0.3;
        setVelPub.publish(velocitySet);
    }
    else
    {
<<<<<<< HEAD
        //EstTarDis();
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
            //ROS_INFO("real x is %f,y is %f",tbearing[0],tbearing[1]);
           // ROS_INFO("expected x is %f,y is %f",tbearing_star[0],tbearing_star[1]);
            Eigen::Vector2d ctrOutput;  //控制输出
            Eigen::Vector2d outFromNei;  //从邻居智能体得到的控制分量
            Eigen::Vector2d outFromTar;  //从目标得到的控制分量
            Eigen::Vector2d temp2 = (Id-preBea_star*preBea_star.transpose())*preBearing;
            Eigen::Vector2d temp1 = (Id-nextBea_star*nextBea_star.transpose())*nextBearing;
            outFromNei = env_k_alpha*(temp2 - temp1);
            outFromTar = -env_k_beta*(Id-tbearing_star*tbearing_star.transpose())*tbearing;
            ctrOutput = outFromNei + outFromTar;
            //ROS_INFO("outFromNei is %f,outFromTar is %f",outFromNei[0],outFromTar[0]);
            //ROS_INFO("ctrOutput is %f",ctrOutput[0]);

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
            rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                    sin(rotTheta),cos(rotTheta);
            ctrOutput = rotateMatrix*ctrOutput;
            velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x;
            velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y;
            if(systemID == 0)
            {
                velocitySet.twist.linear.x = 0.1;
                velocitySet.twist.linear.y = 0.1;
            }
            setVelPub.publish(velocitySet);

=======
        // EstTarDis();

        if(mutualBearing.bearings.size()!=0)
        {
            if(mutualBearing.bearings.size() == 1)
            {
                Eigen::Vector2d tbearing;   //和目标的方位
                Eigen::Vector2d tbearing_star;   //和目标的方位
                Eigen::Matrix2d Id;
                Id <<1,0,
                     0,1;
                tbearing[0] = targetBearing.bearings[0].x;
                tbearing[1] = targetBearing.bearings[0].y;
                tbearing_star[0] = expBearing.bearings[1].x;
                tbearing_star[1] = expBearing.bearings[1].y;
                Eigen::Vector2d ctrOutput;  //控制输出
                Eigen::Vector2d outFromTar;  //从目标得到的控制分量

                //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                outFromTar = env_k_beta*(Id-tbearing*tbearing.transpose())*tbearing_star;
                ctrOutput = outFromTar;

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
                rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                        sin(rotTheta),cos(rotTheta);
                ctrOutput = rotateMatrix*ctrOutput;
                float desiredRadius = 1;
                float currentRadius = pow((allUavPos.agentPosition[0].x-targetInfo.pose.pose.position.x),2)+
                        pow((allUavPos.agentPosition[0].y-targetInfo.pose.pose.position.y),2);
                velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[0];
                velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[1];
                ROS_INFO("current radius is %f",currentRadius);

                setVelPub.publish(velocitySet);


            }
            else
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
                Eigen::Vector2d temp2 = (Id-preBearing*preBearing.transpose())*preBea_star;
                Eigen::Vector2d temp1 = (Id-nextBearing*nextBearing.transpose())*nextBea_star;
                outFromNei = env_k_alpha*(temp1 - temp2);
                //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                outFromTar = env_k_beta*(Id-tbearing*tbearing.transpose())*tbearing_star;
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
                rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                        sin(rotTheta),cos(rotTheta);
                ctrOutput = rotateMatrix*ctrOutput;
                float desiredRadius = 10;
                float currentRadius = pow((allUavPos.agentPosition[systemID-1].x-targetInfo.pose.pose.position.x),2)+
                        pow((allUavPos.agentPosition[systemID-1].y-targetInfo.pose.pose.position.y),2);
                velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[0];
                velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[1];
                ROS_INFO("current radius is %f",currentRadius);
                if(systemID ==2)
                {
                    //ROS_INFO("Estimation is %f,groudtruth is %f",dis2TarEst[0],currentRadius);
                }
                setVelPub.publish(velocitySet);
            }
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
        }
        else
        {
            ROS_WARN("there is no bearing info found!");
            setPositionPub.publish(positionSet);
        }
<<<<<<< HEAD
    }

}

void smarteye::Formation::circleCtr(double targetHei)
{


    if(systemID == 0)  //目标控制
    {
        velocitySet.twist.linear.x = 0.3;
        velocitySet.twist.linear.y = 0.3;
        setVelPub.publish(velocitySet);
    }
    else
    {
        // EstTarDis();

        if(mutualBearing.bearings.size()!=0)
        {
            if(mutualBearing.bearings.size() == 1)
            {
                Eigen::Vector2d tbearing;   //和目标的方位
                Eigen::Vector2d tbearing_star;   //和目标的方位
                Eigen::Matrix2d Id;
                Id <<1,0,
                        0,1;
                tbearing[0] = targetBearing.bearings[0].x;
                tbearing[1] = targetBearing.bearings[0].y;
                tbearing_star[0] = expBearing.bearings[1].x;
                tbearing_star[1] = expBearing.bearings[1].y;
                Eigen::Vector2d ctrOutput;  //控制输出
                Eigen::Vector2d outFromTar;  //从目标得到的控制分量

                //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                outFromTar = env_k_beta*(Id-tbearing*tbearing.transpose())*tbearing_star;
                ctrOutput = outFromTar;
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
                rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                        sin(rotTheta),cos(rotTheta);
                ctrOutput = rotateMatrix*ctrOutput;
                float desiredRadius = 2;
                float currentRadius = pow((allUavPos.agentPosition[0].x-targetInfo.pose.pose.position.x),2)+
                        pow((allUavPos.agentPosition[0].y-targetInfo.pose.pose.position.y),2);
                velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[0]*env_k_gamma;
                velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[1]*env_k_gamma;
                setVelPub.publish(velocitySet);


            }
            else
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
                Eigen::Vector2d temp2 = (Id-preBearing*preBearing.transpose())*preBea_star;
                Eigen::Vector2d temp1 = (Id-nextBearing*nextBearing.transpose())*nextBea_star;
                outFromNei = env_k_alpha*(temp1 - temp2);
                //env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //       (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                //        outFromNei = env_k_alpha*((Id-preBea_star*preBea_star.transpose())*preBearing-
                //                                 (Id-nextBea_star*nextBea_star.transpose())*nextBearing);
                outFromTar = env_k_beta*(Id-tbearing*tbearing.transpose())*tbearing_star;
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
                rotateMatrix <<cos(rotTheta),-sin(rotTheta),
                        sin(rotTheta),cos(rotTheta);
                ctrOutput = rotateMatrix*ctrOutput;
                float desiredRadius = 2;
                float currentRadius = pow((allUavPos.agentPosition[systemID-1].x-targetInfo.pose.pose.position.x),2)+
                        pow((allUavPos.agentPosition[systemID-1].y-targetInfo.pose.pose.position.y),2);
                velocitySet.twist.linear.x = ctrOutput[0]+targetInfo.twist.twist.linear.x
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[0]*env_k_gamma;
                velocitySet.twist.linear.y = ctrOutput[1]+targetInfo.twist.twist.linear.y
                        -(sqrt(currentRadius)-desiredRadius)*tbearing[1]*env_k_gamma;
                ROS_INFO("current radius is %f",sqrt(currentRadius));
                if(systemID ==2)
                {
                    //ROS_INFO("Estimation is %f,groudtruth is %f",dis2TarEst[0],currentRadius);
                }
                setVelPub.publish(velocitySet);
            }
        }
        else
        {
            ROS_WARN("there is no bearing info found!");
            setPositionPub.publish(positionSet);
        }

    }
}

void smarteye::Formation::InitParam()
{
    env_k_alpha = 1;
    env_k_beta = 1;
    env_k_gamma = 1.4;
    timecount = 0;
    rotTheta = -0.1;
    positionSet = localPose;
    positionSet.pose.position.z = -1;
    initialHeight = -0.2;
=======

    }
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
}



smarteye::heiCtrller::heiCtrller()
{
    ei = 0;
<<<<<<< HEAD
    hei_kp1 = 2;
    hei_kp2 = 0.9;
    hei_kpdiv = 0.1;
    hei_kd = 0.15;
    hei_ki = 0.01;
    hei_bias = 0.001;
=======
    hei_kp = 0.26;
    hei_kd = 0.1;
    hei_ki = 0.005;
    hei_bias = 0.03;
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc

}

smarteye::heiCtrller::~heiCtrller()
{

}

float smarteye::heiCtrller::cacOutput()
{

    float output;
    float err = targetHei - currentHei;
    ei = ei + hei_ki*err;
    ei = ValueLimit(ei,0.3,-0.3);
    float kpOutput;
    if (fabs(err)>hei_kpdiv)
    {
        //如果在第二段内
        if (err>0)
            kpOutput = hei_kp1*hei_kpdiv+ hei_kp2*(err-hei_kpdiv);
        else
            kpOutput = hei_kp1*hei_kpdiv + hei_kp2*(err+hei_kpdiv);
    }
    else
    {
        //如果在第一段内
        kpOutput =hei_kp1*err;
    }
    output = hei_bias + kpOutput + ei + hei_kd*(err-previousErr);
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


float smarteye::CacMatrix2DNorm(Eigen::Matrix2d matrix)
{
    float norm=0.0;
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            norm=norm+matrix(i,j)*matrix(i,j);
        }
    }
    return(sqrt(norm));
}


float smarteye::CacVector2DNorm(Eigen::Vector2d vec)
{
    float norm=0.0;
    for(int i=0;i<2;i++)
    {
        norm=norm+vec[i]*vec[i];
    }
    return(sqrt(norm));
}


smarteye::xyCtrller::xyCtrller()
{
    ei = 0;
    xy_kp = 0.2;
    xy_ki = 0.003;
    xy_kd = 0.05;
    xy_bias = 0;
}

smarteye::xyCtrller::~xyCtrller()
{

}

float smarteye::xyCtrller::cacOutput()
{

    float output;
    float err = targetPos- currentPos;
    ei = ei + xy_ki*err;
    ei = ValueLimit(ei,0.3,-0.3);
    output = xy_bias + xy_kp*err + ei + xy_kd*(err-previousErr);
    output = ValueLimit(output,1,-1);
    previousErr = err;
    return(output);

}
