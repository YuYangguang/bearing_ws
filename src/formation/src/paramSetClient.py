#!/usr/bin/env python  
  
PACKAGE = 'formation'  
import roslib;roslib.load_manifest(PACKAGE)  
import rospy  
  
import dynamic_reconfigure.client  
  
def callback(config):  
    rospy.loginfo("Config set to {ENV_K_ALPHA}, {ENV_K_BETA}, {HEI_KP}, {HEI_KD}, {HEI_KI}, {HEI_BIAS}".format(**config))  
     
  
if __name__ == "__main__":  
    rospy.init_node("paramSetClient")  
  
# Client first argument is the node names  
    client = dynamic_reconfigure.client.Client("dynamic", timeout=30, config_callback=callback)  
  
    r = rospy.Rate(0.1)  
    #x = 0  
    #b = False   
    while not rospy.is_shutdown():  
        #x = x+1  
        #if x>10:  
        #    x=0  
        #b = not b  
       # client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})  
        r.sleep()  