#include <dynamic_reconfigure/server.h>  
#include "formation/formationCFGConfig.h"  
#include <ros/ros.h>  

void callback(formation::formationCFGConfig &config, uint32_t level)  
{  
<<<<<<< HEAD
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
           config.CIR_SIGMA1,
config.CIR_SIGMA2,
config.ENCIR_SIGMA1,
config.ENCIR_SIGMA2,
=======
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
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
<<<<<<< HEAD
           config.ROTT

=======
           config.ROTT,
            config.CIR_SIGMA1,
config.CIR_SIGMA2,
config.ENCIR_SIGMA1,
config.ENCIR_SIGMA2
>>>>>>> 70c8a32f592250a9fe6be9b4e3ab95881aae7efc
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
