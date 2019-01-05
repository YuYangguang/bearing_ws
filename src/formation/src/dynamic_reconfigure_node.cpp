#include <dynamic_reconfigure/server.h>  
#include "formation/formationCFGConfig.h"  
#include <ros/ros.h>  

void callback(formation::formationCFGConfig &config, uint32_t level)  
{  
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
           config.CIR_SIGMA1,
config.CIR_SIGMA2,
config.ENCIR_SIGMA1,
config.ENCIR_SIGMA2,
           config.ENV_K_ALPHA,  
           config.ENV_K_BETA, 
          config.ENV_K_GAMMA, 
           config.HEI_KP1,
           config.HEI_KP2,
           config.HEI_KPDIV,
           config.HEI_KI, 
            config.HEI_KD,   
            config.HEI_BIAS,
           config.XY_KP,
           config.XY_KI,
            config.XY_KD,
            config.XY_BIAS,
           config.ROTT

);
  
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
