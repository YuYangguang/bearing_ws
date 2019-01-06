
/*****************************************************************************************
// Author:      Yu Yangguang
// Date:        2017.11.10
// Description: Manage nodes and tasks
*****************************************************************************************/
#include <commander/commander.h>



using namespace smarteye;

commander::commander(int argc, char **argv, const char *name)
{
    float updateHz = 2;
    float updateTime = 1.0/updateHz;
    ros::init(argc, argv, "");
    nh = boost::make_shared<ros::NodeHandle>("~");
    commanderUpdateTimer = nh->createTimer(ros::Duration(updateTime),&commander::update, this);
<<<<<<< HEAD
    param_to_formation = nh->serviceClient<bearing_common::HParamSetSrv>("/uav1/formation/host_param/set_param");
    param_to_formation2 = nh->serviceClient<bearing_common::HParamSetSrv>("/uav2/formation/host_param/set_param");
    param_to_formation3 = nh->serviceClient<bearing_common::HParamSetSrv>("/uav3/formation/host_param/set_param");
=======
    param_to_formation = nh->serviceClient<bearing_common::HParamSetSrv>("/formation/host_param/set_param");
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
}

commander::~commander()
{

}

void commander::update(const ros::TimerEvent &event)
{
    float tempParam;
    ros::param::get("/dynamic/HEI_KI",tempParam);
<<<<<<< HEAD
    bool IsSuccess;
=======
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc

    bearing_common::HParamSetSrv srv;
    if(fabs(hei_ki-tempParam)>1e-7)
    {
        hei_ki = tempParam;
        srv.request.param_id = "HEI_KI";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_KI");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_KI");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_KI");
        }
    }
    ros::param::get("/dynamic/HEI_KP1",tempParam);
    //ROS_INFO("hei_kp is %f,err is %f",tempParam,fabs(hei_kp-tempParam));

    if(fabs(hei_kp1-tempParam)>1e-7)
    {
        ROS_INFO("detect parameter HEI_KP1 is changed");
        hei_kp1 = tempParam;
        srv.request.param_id = "HEI_KP1";
        srv.request.param_value = tempParam;
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_KP1");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_KP1");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_KP1");
        }
    }
    ros::param::get("/dynamic/HEI_KP2",tempParam);
    if(fabs(hei_kp2-tempParam)>1e-7)
    {
        ROS_INFO("detect parameter HEI_KP2 is changed");
        hei_kp2 = tempParam;
        srv.request.param_id = "HEI_KP2";
        srv.request.param_value = tempParam;
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_KP2");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_KP2");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_KP2");
        }
    }
    ros::param::get("/dynamic/HEI_KPDIV",tempParam);
    if(fabs(hei_kpdiv-tempParam)>1e-7)
    {
        ROS_INFO("detect parameter HEI_KPDIV is changed");
        hei_kpdiv = tempParam;
        srv.request.param_id = "HEI_KPDIV";
        srv.request.param_value = tempParam;
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_KPDIV");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_KPDIV");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_KPDIV");
=======
        param_to_formation.call(srv);
    }
    ros::param::get("/dynamic/HEI_KP",tempParam);
    //ROS_INFO("hei_kp is %f,err is %f",tempParam,fabs(hei_kp-tempParam));

    if(fabs(hei_kp-tempParam)>1e-7)
    {
        ROS_INFO("detect parameter HEI_KP is changed");
        hei_kp = tempParam;
        srv.request.param_id = "HEI_KP";
        srv.request.param_value = tempParam;
        bool IsSuccess = param_to_formation.call(srv);
        if (IsSuccess)
        {
            ROS_INFO("parameter HEI_KP has updated");
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
        }
    }
    ros::param::get("/dynamic/HEI_KD",tempParam);
    if(fabs(hei_kd-tempParam)>1e-7)
    {
        hei_kd = tempParam;
        srv.request.param_id = "HEI_KD";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_KD");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_KD");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_KD");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/HEI_BIAS",tempParam);
    if(fabs(hei_bias-tempParam)>1e-7)
    {
        hei_bias = tempParam;
        srv.request.param_id = "HEI_BIAS";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param HEI_BIAS");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param HEI_BIAS");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param HEI_BIAS");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/XY_BIAS",tempParam);
    if(fabs(xy_bias-tempParam)>1e-7)
    {
        xy_bias = tempParam;
        srv.request.param_id = "XY_BIAS";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param XY_BIAS");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param XY_BIAS");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param XY_BIAS");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/XY_KP",tempParam);
    if(fabs(xy_kp-tempParam)>1e-7)
    {
        xy_kp = tempParam;
        srv.request.param_id = "XY_KP";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param XY_KP");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param XY_KP");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param XY_KP");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/XY_KI",tempParam);
    if(fabs(xy_ki-tempParam)>1e-7)
    {
        xy_ki = tempParam;
        srv.request.param_id = "XY_KI";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param XY_KI");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param XY_KI");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param XY_KI");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/XY_KD",tempParam);
    if(fabs(xy_kd-tempParam)>1e-7)
    {
        xy_kd = tempParam;
        srv.request.param_id = "XY_KD";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param XY_KD");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param XY_KD");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param XY_KD");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/ROTT",tempParam);
    if(fabs(rotTheta-tempParam)>1e-7)
    {
        rotTheta = tempParam;
        srv.request.param_id = "ROTT";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param ROTT");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param ROTT");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param ROTT");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/ENV_K_ALPHA",tempParam);
    if(fabs(env_k_alpha-tempParam)>1e-7)
    {
        env_k_alpha = tempParam;
        srv.request.param_id = "ENV_K_ALPHA";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param ENV_K_ALPHA");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param ENV_K_ALPHA");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param ENV_K_ALPHA");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }
    ros::param::get("/dynamic/ENV_K_BETA",tempParam);
    if(fabs(env_k_beta-tempParam)>1e-7)
    {
        env_k_beta = tempParam;
        srv.request.param_id = "ENV_K_BETA";
        srv.request.param_value = tempParam;
<<<<<<< HEAD
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param ENV_K_BETA");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param ENV_K_BETA");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param ENV_K_BETA");
        }
    }
    ros::param::get("/dynamic/ENV_K_GAMMA",tempParam);
    if(fabs(env_k_gamma-tempParam)>1e-7)
    {
        env_k_gamma = tempParam;
        srv.request.param_id = "ENV_K_GAMMA";
        srv.request.param_value = tempParam;
        IsSuccess=param_to_formation.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("fail to set param ENV_K_GAMMA");
        }
        IsSuccess=param_to_formation2.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("2 fail to set param ENV_K_GAMMA");
        }
        IsSuccess=param_to_formation3.call(srv);
        if(!IsSuccess)
        {
            ROS_WARN("3 fail to set param ENV_K_GAMMA");
        }
=======
        param_to_formation.call(srv);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
    }

}

void commander::initParam()
{
        bool IsParamGot;
        bool IsAllParamGot = true;
        IsParamGot = ros::param::get("/dynamic/HEI_KI",hei_ki);
        IsAllParamGot = IsAllParamGot && IsParamGot;
<<<<<<< HEAD
        IsParamGot = ros::param::get("/dynamic/HEI_KP1",hei_kp1);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/HEI_KP2",hei_kp2);
        IsAllParamGot = IsAllParamGot && IsParamGot;
        IsParamGot = ros::param::get("/dynamic/HEI_KPDIV",hei_kpdiv);
=======
        IsParamGot = ros::param::get("/dynamic/HEI_KP",hei_kp);
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
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




