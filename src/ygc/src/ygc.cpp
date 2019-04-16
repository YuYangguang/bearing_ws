
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
    sleep(1);
    updateHz = 18;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "ygc_node");
    nh = boost::make_shared<ros::NodeHandle>("~");
    bool IsGotParam;
    updateCount = 0;
    IsGotParam = nh->getParam("uav_num",uavNum);
    if(IsGotParam)
    {
        ROS_INFO("got system id successfully and uav number is %d",uavNum);
        if(uavNum < 1)
        {
            ROS_ERROR("invalid uav total number, and the node is shut down");
            exit(0);
        }
    }
    else
    {
        ROS_ERROR("failed to get uav number and node is shut down");
        exit(0);
    }
    IsGotParam = nh->getParam("TriggerFlag",IsUseEventTri);
    if(IsGotParam)
    {
        if(IsUseEventTri)
        {
            ROS_INFO("This station is using event-trigger mechanism!");
        }
        else
        {
            ROS_INFO("This station is using time-trigger mechanism!");
        }
    }
    else
    {
        ROS_ERROR("failed to get trigger flag and node is shut down");
        exit(0);
    }
    IsGotParam = nh->getParam("SimuFlag",IsUseSimu);
    if(IsGotParam)
    {
        if(IsUseSimu) //仿真模式
        {
            ROS_INFO("This station is in simulation mode!");
            gazeboInfoSub = nh->subscribe("/gazebo/model_states", 5,&ygcClass::ReceiveGazeboInfo, this);
        }
        else //实物飞行模式
        {
            ROS_INFO("This station is in real fly mode");
            uavPosSub1= nh->subscribe("/vicon/nuflie02/nuflie02", 10,
                                      &ygcClass::ReceiUavPos1,this);
            uavPosSub2= nh->subscribe("/vicon/nuflie04/nuflie04", 10,
                                      &ygcClass::ReceiUavPos2,this);
            uavPosSub3= nh->subscribe("/vicon/nuflie03/nuflie03", 10,
                                      &ygcClass::ReceiUavPos3,this);
            targetViconSub = nh->subscribe("/vicon/target/target",2,&ygcClass::ReceiveTarViconPose,this);
        }
    }
    else
    {
        ROS_ERROR("failed to get simulation flag and node is shut down");
        exit(0);
    }


    allPositions = new geometry_msgs::Vector3[uavNum];
    mutualBearing.bearings.resize(uavNum);
    targetBearing.bearings.resize(uavNum);
    lastMutuBearing.bearings.resize(uavNum);
    lastTargetBearing.bearings.resize(uavNum);
    dotMutualBearing.bearings.resize(uavNum);
    agentVel.bearings.resize(uavNum);
    expBearing.bearings.resize(uavNum*2);
    std::string tempName;
    targetPos.pose.pose.position.x = 0;
    targetPos.pose.pose.position.y = 0;
    targetPos.pose.pose.position.z = 0;
    rotateTheta = 0;
    rotateSpeed = 0;
    posRec.agentPosition.resize(uavNum);
    mutualBearingPub = nh->advertise<bearing_common::GroupBearing>("/ygc/mutual_bearing",2);
    targetBearingPub = nh->advertise<bearing_common::GroupBearing>("/ygc/target_bearing",2);
    expBearingPub = nh->advertise<bearing_common::GroupBearing>("/ygc/expected_bearing",2);

    keyboardSub= nh->subscribe("/keyboard/keydown",1,&ygcClass::ReceiveKeybdCmd,this);
    ygcUpdateTimer = nh->createTimer(ros::Duration(updateTime),&ygcClass::update, this);
    bearingUpdateTimer = nh->createTimer(ros::Duration(0.035),&ygcClass::bearingUpdate, this);
    bearingInfoInit();
    uavForRec = 1;
    tempName = "/uav"+num2str(uavForRec);
    //velCmdSub = nh->subscribe(tempName+"/mavros/local_position/velocity", 5,&ygcClass::ReceiveCmdInfo1, this);

    //dataRecPub = nh->advertise<bearing_common::DataRecord>("/ygc/dataRecorded",5);
    targetPosePub = nh->advertise<nav_msgs::Odometry>("/ygc/targetPose",5);
    //dotBearingPub = nh->advertise<bearing_common::GroupBearing>("/ygc/dot_bearing",2);
    //agentVelPub = nh->advertise<bearing_common::GroupBearing>("/ygc/agentVel",2);
    allPosiPub = nh->advertise<bearing_common::AllPosition>("/ygc/allPosition",5);
    trigRecPub = nh->advertise<bearing_common::TriggerRec>("/ygc/triggerRec",5);
    lastBearUpTime = 0;
    IsInfoUpdate =false;
    triggerRec.trigNum = 0;
    triggerRec.sysTime = 0;
    coordMethod =IDLE;
}



ygcClass::~ygcClass()
{

    //    delete[] setPositionPublisher;
    delete[] allPositions;
}



std::string ygcClass::num2str(int i)
{
    std::stringstream ss;
    ss<<i;
    return ss.str();
}

void ygcClass::ReceiveCmdInfo1(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    dataRec.cmdVel.x = msg->twist.linear.x;
    dataRec.cmdVel.y = msg->twist.linear.y;
    //ROS_INFO("uav vel x is %f,y is %f",dataRec.cmdVel.x,dataRec.cmdVel.y);
    // ROS_WARN("ground truth vel x is %f,y is %f",dataRec.trueVel.x,dataRec.trueVel.y);
}

void ygcClass::ReceiveCmdInfo2(const geometry_msgs::TwistStamped::ConstPtr &msg)
{

}



void ygcClass::ReceiveKeybdCmd(const keyboard::Key &key)
{
    switch(key.code)
    {
    case 'c':   //arm the vehicle
    {
        coordMethod = CIRCLE;
        ROS_INFO("the formation begins to circle control!");
        break;
    }
    case 'e':
    {
        coordMethod = ENCIRCLE;
        ROS_INFO("the formation begins to encirclement control!");
        break;
    }
    case 'q':
    {
        if(fabs(rotateSpeed)<1E-6)
        {
            rotateSpeed = -YGC_PI/30;
        }
        break;
    }
    default:
    {
        coordMethod = IDLE;
        rotateSpeed = 0;
        rotateTheta = 0;

    }



    }
}

/*此处容易出现bug,如用于其他用途，请仔细理解源码*/
void ygcClass::ReceiveGazeboInfo(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    double updateTime = ros::Time::now().toNSec();
    double delta_time = (updateTime-lastBearUpTime)/1000000000;
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

            std::string uavID_string=msg->name[i].substr(numberPos-1,1);
            std::string ori = msg->name[i];
            int IDLength = uavID_string.length();

            char* uavID_char = new char[IDLength];
            uavID_string.copy(uavID_char,IDLength,0);
            char cID;
            cID = uavID_char[0];
            int uavID = atoi(&cID);
            //ROS_INFO("uavId is %d",uavID);
            delete[] uavID_char;  //释放内存
            if(uavID == uavForRec)
            {
                dataRec.trueVel.x = msg->twist[i].linear.x;
                dataRec.trueVel.y = msg->twist[i].linear.y;
            }
            if(uavID >uavNum&&(uavID!=9))
            {
                ROS_WARN("vehicle ID is beyond the number of vehicle");
                ROS_ERROR("uavID is %d",uavID);
                printf("original name is %s \n",ori.c_str());
            }
            else if(uavID == 9)  //目标ID号为0
            {
                targetPos.pose.pose.position.x = msg->pose[i].position.x;
                targetPos.pose.pose.position.y = msg->pose[i].position.y;
                targetPos.pose.pose.position.z = msg->pose[i].position.z;
                targetPos.twist.twist.linear.x = msg->twist[i].linear.x;
                targetPos.twist.twist.linear.y = msg->twist[i].linear.y;
                targetPos.twist.twist.linear.z = msg->twist[i].linear.z;
            }
            else
            {
                allPositions[uavID-1].x = msg->pose[i].position.x;
                allPositions[uavID-1].y = msg->pose[i].position.y;
                allPositions[uavID-1].z = msg->pose[i].position.z;
                agentVel.bearings[uavID-1].x  = msg->twist[i].linear.x;
                agentVel.bearings[uavID-1].y  = msg->twist[i].linear.y;

            }

        }
    }

    if(!IsUseEventTri)   //如果不使用事件驱动机制
    {

        if(delta_time<0.3 &&(delta_time>1E-4))  //主要防止delta_time为0的情况
        {
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
                dis = sqrt(pow((allPositions[i].x -targetPos.pose.pose.position.x),2)+
                           pow((allPositions[i].y -targetPos.pose.pose.position.y),2));
                targetBearing.bearings[i].x = (allPositions[i].x-targetPos.pose.pose.position.x)/dis;
                targetBearing.bearings[i].y = (allPositions[i].y-targetPos.pose.pose.position.y)/dis;
                float tempx = (mutualBearing.bearings[i].x-lastMutuBearing.bearings[i].x)/delta_time;
                float tempy = (mutualBearing.bearings[i].y-lastMutuBearing.bearings[i].y)/delta_time;
                float norm = sqrt(tempx*tempx+ tempy*tempy);
                //ROS_INFO("delta_time is %f,norm is %f",delta_time,norm);
                dotMutualBearing.bearings[i].x = tempx;
                dotMutualBearing.bearings[i].y = tempy;
            }

        }
        if(delta_time>1E-4)
        {
            lastBearUpTime = updateTime;
            lastMutuBearing = mutualBearing;
        }
    }



}

void ygcClass::update(const ros::TimerEvent &event)
{
    updateCount++;

    if(updateCount > 15)
    {
        updateCount = 0;
    }

    //********************所有飞机位置数据记录**************/
    for(int i=0;i<uavNum;i++)
    {
        posRec.agentPosition[i].x = allPositions[i].x;
        posRec.agentPosition[i].y = allPositions[i].y;
    }
    posRec.target_x = targetPos.pose.pose.position.x;
    posRec.target_y = targetPos.pose.pose.position.y;
    allPosiPub.publish(posRec);
    /************************************************/

    expBearingPub.publish(expBearing);
    //dataRecPub.publish(dataRec);
    targetPosePub.publish(targetPos);  //发布飞机速度
    //agentVelPub.publish(agentVel);
    if(uavNum == 1)
    {
        float dis = sqrt(pow((allPositions[0].x -targetPos.pose.pose.position.x),2)+
                pow((allPositions[0].y -targetPos.pose.pose.position.y),2));
        targetBearing.bearings[0].x = (allPositions[0].x-targetPos.pose.pose.position.x)/dis;
        targetBearing.bearings[0].y = (allPositions[0].y-targetPos.pose.pose.position.y)/dis;
    }
    if(coordMethod == IDLE)
    {
        mutualBearingPub.publish(mutualBearing);
        targetBearingPub.publish(targetBearing);
    }
    if(IsUseEventTri)
    {
        bool IsEventTriged = false; //是否满足事件触发条件
        Eigen::Matrix<double, 3, 3> H;
        H << 1 ,-1 ,0  ,
                0 ,1 ,-1 ,
                -1 ,0 ,1 ;
        Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(2,2);
        Eigen::MatrixXd H_bar(2*uavNum, 2*uavNum);
        H_bar = Eigen::kroneckerProduct(H,Id);
        Eigen::kroneckerProduct(H,Id);
        Eigen::MatrixXd diagPgk_star(2*uavNum,2*uavNum);
        Eigen::MatrixXd diagDb_star(2*uavNum,2*uavNum);
        diagDb_star = Eigen::MatrixXd::Zero(2*uavNum,2*uavNum);
        diagPgk_star = Eigen::MatrixXd::Zero(2*uavNum,2*uavNum);
        Eigen::Matrix2d Pgk_star;
        Eigen::Matrix2d Pgt_star;
        Eigen::Vector2d gij_star;
        Eigen::Vector2d git_star;
        Eigen::VectorXd gij;
        Eigen::VectorXd git;
        Eigen::VectorXd gij_event;
        Eigen::VectorXd git_event;
        gij.resize(2*uavNum,1);
        git.resize(2*uavNum,1);
        gij_event.resize(2*uavNum,1);
        git_event.resize(2*uavNum,1);
        for(int i=0;i<uavNum;i++)
        {

            gij_star[0] = expBearing.bearings[i].x;
            gij_star[1] = expBearing.bearings[i].y;
            gij[i*2] = mutualBearing.bearings[i].x;
            gij[i*2+1] = mutualBearing.bearings[i].y;
            git[i*2] = targetBearing.bearings[i].x;
            git[1+i*2] = targetBearing.bearings[i].y;
            gij_event[i*2] = lastMutuBearing.bearings[i].x;
            gij_event[i*2+1] = lastMutuBearing.bearings[i].y;
            git_event[i*2] = lastTargetBearing.bearings[i].x;
            git_event[1+i*2] = lastTargetBearing.bearings[i].y;
            if(gij_star.norm()>1.1)
            {
                ROS_ERROR("gij star is %f",gij_star.norm());
            }
            Pgk_star = Id-gij_star*gij_star.transpose();
            if(Pgk_star.norm()>4)
            {
                ROS_ERROR("Pgk_star %f",Pgk_star.norm());
            }
            diagPgk_star.block(i*2,i*2,2,2) = Pgk_star;
            git_star[0] = expBearing.bearings[i+uavNum].x;
            git_star[1] = expBearing.bearings[i+uavNum].y;
            Pgt_star = Id-git_star*git_star.transpose();
            diagDb_star.block(i*2,i*2,2,2) = Pgt_star;
        }
        if(coordMethod == CIRCLE)
        {
            triggerRec.sysTime++;
            if(updateCount == 10)
            {
                ros::param::get("/dynamic/CIR_SIGMA1",cir_sigma1);
                ros::param::get("/dynamic/CIR_SIGMA2",cir_sigma2);
            }
            Eigen::VectorXd  err1;
            Eigen::VectorXd  err2;
            Eigen::VectorXd  tempV;
            tempV.resize(2*uavNum);
            err1.resize(2*uavNum);
            err2.resize(2*uavNum);
            err1 = 2*H_bar.transpose()*diagPgk_star*(gij-gij_event);
            Eigen::VectorXd  err1V;
            err1V.resize(2*uavNum);
            err1V=(gij-gij_event);
            err2 = 2*H_bar.transpose()*diagDb_star*(git-git_event);
            tempV=(git-git_event);
            Eigen::VectorXd  event_con1;//事件触发条件1
            Eigen::VectorXd  event_con2;//事件触发条件2
            event_con1.resize(2*uavNum);
            event_con2.resize(2*uavNum);
            event_con1 = diagPgk_star*gij;
            event_con2 = diagDb_star*git;
            //ROS_INFO("err1 is %f,err2 is %f",err1.norm(),err2.norm());
            if((err1.norm()+err2.norm())>cir_sigma1*event_con1.norm())
            {
                IsEventTriged = true;
            }
            if((err1.norm()+err2.norm())>cir_sigma2*event_con2.norm())
            {
                IsEventTriged = true;
            }
            triggerRec.stateErr = (err1.norm()+err2.norm());
            //ROS_INFO("cir_sigma1 is %f,cir_sigma2 is %f",cir_sigma1,cir_sigma2);

        }
        if(coordMethod == ENCIRCLE)
        {
            triggerRec.sysTime++;
            if(updateCount == 10)
            {
                ros::param::get("/dynamic/ENCIR_SIGMA1",encir_sigma1);
                ros::param::get("/dynamic/ENCIR_SIGMA2",encir_sigma2);
            }
            Eigen::VectorXd  err;
            err.resize(2*uavNum);
            err = 2*H_bar.transpose()*diagPgk_star*(gij-gij_event)
                    -2*H_bar.transpose()*diagDb_star*(git-git_event);

            Eigen::VectorXd  event_con1;//事件触发条件1
            Eigen::VectorXd  event_con2;//事件触发条件2

            event_con1 = diagPgk_star*gij;
            event_con2 = diagDb_star*git;
            if((err.norm())>(encir_sigma1*event_con1.norm()/H_bar.norm()))
            {
                IsEventTriged = true;
            }
            if((err.norm())>encir_sigma2*event_con2.norm())
            {
                IsEventTriged = true;
            }
            triggerRec.stateErr = err.norm();
        }
        if(IsEventTriged)
        {
            //ROS_WARN("is triggered");
            lastMutuBearing = mutualBearing;  //更新输出
            lastTargetBearing = targetBearing;  //更新输出
            triggerRec.trigNum++;
            mutualBearingPub.publish(mutualBearing);
            targetBearingPub.publish(targetBearing);
        }

    }
    else
    {
        mutualBearingPub.publish(mutualBearing);
        targetBearingPub.publish(targetBearing);
        //dotBearingPub.publish(dotMutualBearing);
    }
    trigRecPub.publish(triggerRec);
}

void ygcClass::bearingUpdate(const ros::TimerEvent &event)
{

    rotateTheta = rotateTheta + rotateSpeed*1.0/updateHz;
    if(uavNum == 1)
    {
        float dis = sqrt(pow((allPositions[0].x -targetPos.pose.pose.position.x),2)+
                pow((allPositions[0].y -targetPos.pose.pose.position.y),2));
        if((dis-1E-5)>0)
        {
            targetBearing.bearings[0].x = (allPositions[0].x-targetPos.pose.pose.position.x)/dis;
            targetBearing.bearings[0].y = (allPositions[0].y-targetPos.pose.pose.position.y)/dis;
        }
    }
    else
    {
        if(IsUseEventTri)
        {
            double updateTime = ros::Time::now().toNSec();
            double delta_time = (updateTime-lastBearUpTime)/1000000000;
            for(int i=0;i<uavNum;i++)
            {
                float dis;
                if(i==uavNum-1)
                {
                    dis = sqrt(pow((allPositions[i].x -allPositions[0].x),2)
                            + pow((allPositions[i].y -allPositions[0].y),2)); //两点之间的距离
                    if((dis-1E-5)>0)
                    {
                        mutualBearing.bearings[i].x = (allPositions[i].x -allPositions[0].x)/dis;
                        mutualBearing.bearings[i].y = (allPositions[i].y -allPositions[0].y)/dis;
                    }
                }
                else
                {
                    dis = sqrt(pow((allPositions[i].x -allPositions[i+1].x),2)
                            + pow((allPositions[i].y -allPositions[i+1].y),2));
                    if((dis-1E-5)>0)
                    {
                        mutualBearing.bearings[i].x = (allPositions[i].x -allPositions[i+1].x)/dis;
                        mutualBearing.bearings[i].y = (allPositions[i].y -allPositions[i+1].y)/dis;
                    }
                }
                dis = sqrt(pow((allPositions[i].x -targetPos.pose.pose.position.x),2)+
                           pow((allPositions[i].y -targetPos.pose.pose.position.y),2));
                if((dis-1E-5)>0)
                {
                    targetBearing.bearings[i].x = (allPositions[i].x-targetPos.pose.pose.position.x)/dis;
                    targetBearing.bearings[i].y = (allPositions[i].y-targetPos.pose.pose.position.y)/dis;
                }
            }
        }
        else
        {
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
                dis = sqrt(pow((allPositions[i].x -targetPos.pose.pose.position.x),2)+
                           pow((allPositions[i].y -targetPos.pose.pose.position.y),2));
                targetBearing.bearings[i].x = (allPositions[i].x-targetPos.pose.pose.position.x)/dis;
                targetBearing.bearings[i].y = (allPositions[i].y-targetPos.pose.pose.position.y)/dis;
            }
        }
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

    if(uavNum == 1)
    {
        int i = 0;
        double agentTheta = 2*YGC_PI*i/3;
        double nextAgentTheta = 2*YGC_PI*(i+1)/3;
        expBearing.bearings[0].x = 0;
        expBearing.bearings[0].y = 1;
        initTarDesBearing[0] = sin(agentTheta);
        initTarDesBearing[1] = cos(agentTheta);
        desiredBearing = rotateMatrix*initTarDesBearing;
        expBearing.bearings[1].x = desiredBearing[0];
        expBearing.bearings[1].y = desiredBearing[1];

    }
    else
    {
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
            if(desiredBearing.norm()>1.01)
            {
                ROS_ERROR("the norm of desired Bearing is impossibel to be more than 1!");
            }
            //与目标的期望方位从uavNum+1开始，到2×uavNum
            expBearing.bearings[i+uavNum].x = desiredBearing[0];
            expBearing.bearings[i+uavNum].y = desiredBearing[1];
        }
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
        mutualBearing.bearings[i].y = 1;
        targetBearing.bearings[i].x = 0;
        targetBearing.bearings[i].y = 1;
        lastMutuBearing.bearings[i].x = 0;
        lastMutuBearing.bearings[i].y= 1;
        allPositions[i].x = 0;
        allPositions[i].y = 1;
        allPositions[i].z = 0;
        expBearing.bearings[i+uavNum].x = sin(agentTheta);
        expBearing.bearings[i+uavNum].y = cos(agentTheta);
    }
    lastMutuBearing = mutualBearing;
    lastTargetBearing = targetBearing;
    targetPos.pose.pose.position.x = 0;
    targetPos.pose.pose.position.y = 0;
    lastTargetTime = 0;
    lastTargetPose2D.x = 0;
    lastTargetPose2D.y = 0;
}

void ygcClass::ReceiUavPos1(const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    allPositions[0].x = vicon_msg->transform.translation.x;
    allPositions[0].y = vicon_msg->transform.translation.y;
}

void ygcClass::ReceiUavPos2(const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    if(uavNum <2)
    {
        ROS_WARN("the uav number set is smaller than the real, 2");
    }
    else
    {
        allPositions[1].x = vicon_msg->transform.translation.x;
        allPositions[1].y = vicon_msg->transform.translation.y;
    }

}

void ygcClass::ReceiUavPos3(const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    if(uavNum <3)
    {
        ROS_WARN("the uav number set is smaller than the real, 3");
    }
    else
    {
        allPositions[2].x = vicon_msg->transform.translation.x;
        allPositions[2].y = vicon_msg->transform.translation.y;
    }
}

void ygcClass::ReceiveTarViconPose(const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    targetPos.pose.pose.position.x = vicon_msg->transform.translation.x;
    targetPos.pose.pose.position.y = vicon_msg->transform.translation.y;
    targetPos.pose.pose.position.z = vicon_msg->transform.translation.z;
    double time = ros::Time::now().toSec();
    double delta_time = time - lastTargetTime;
    if(delta_time>1E-5)
    {
        targetPos.twist.twist.linear.x = (targetPos.pose.pose.position.x-lastTargetPose2D.x)/delta_time;
        targetPos.twist.twist.linear.y = (targetPos.pose.pose.position.y-lastTargetPose2D.y)/delta_time;
        lastTargetTime = time;
        lastTargetPose2D.x = targetPos.pose.pose.position.x;
        lastTargetPose2D.y = targetPos.pose.pose.position.y;
    }
    else
    {
        ROS_ERROR("the delta_time could not be zero!");
    }


}






