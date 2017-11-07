
/*****************************************************************************************
// Author:      Yu Yangguang
// Date:        2017.11.10
// Description: Manage nodes and tasks
*****************************************************************************************/
#include <commander/commander.h>



using namespace smarteye;

commander::commander(int argc, char **argv, const char *name)
{
    float updateHz = 20;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "");
    nh = boost::make_shared<ros::NodeHandle>("~");
    commanderUpdateTimer = nh->createTimer(ros::Duration(updateTime),&commander::update, this);
    param_to_formation = nh->serviceClient<smarteye_common::HParamSetSrv>("/formation/host_param/set_param");
    
}

commander::~commander()
{

}

void commander::update(const ros::TimerEvent &event)
{
    float tempParam;
    ros::param::get("/dynamic/HEI_KI",tempParam);
    if(abs(hei_ki-tempParam)>1e-3)
    {
        hei_ki = tempParam;
    }

}

void commander::initParam()
{
        bool IsParamGot;
        bool IsAllParamGot = true;
        IsParamGot = ros::param::get("/dynamic/HEI_KI",hei_ki);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/HEI_KP",hei_kp);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/HEI_KD",hei_kd);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/HEI_BIAS",hei_bias);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/XY_BIAS",xy_bias);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/XY_KP",xy_kp);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/XY_KD",xy_kd);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/XY_KI",xy_ki);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/ENV_K_ALPHA",env_k_alpha);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/ENV_K_BETA",env_k_beta);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/ROTT",rotTheta);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        if (IsAllParamGot == true)
        {
            ROS_INFO("Get param successfully");
        }
        else
        {
            ROS_ERROR("Fail to get param");
        }
}




