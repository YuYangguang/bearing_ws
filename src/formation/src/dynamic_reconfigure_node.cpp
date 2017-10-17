#include <dynamic_reconfigure/server.h>  
#include "formation/formationCFGConfig.h"  
#include <ros/ros.h>  

void callback(formation::formationCFGConfig &config, uint32_t level)  
{  
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f",
           config.ENV_K_ALPHA,  
           config.ENV_K_BETA,  
           config.HEI_KP,  
           config.HEI_KI, 
            config.HEI_KD,   
            config.HEI_BIAS,
           config.ROTT);
  
  // do nothing for now  
}  
  
int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "dynamic");  
  dynamic_reconfigure::Server<formation::formationCFGConfig> srv;  
  dynamic_reconfigure::Server<formation::formationCFGConfig>::CallbackType f;  
  f = boost::bind(&callback, _1, _2);  
  srv.setCallback(f);  
  ROS_INFO("Starting to spin...");  
  ros::spin();  
  return 0;  
}  
