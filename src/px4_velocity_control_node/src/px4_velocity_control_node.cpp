#include <ros/ros.h>

#include <keyboard/Key.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

geometry_msgs::TwistStamped vs;

geometry_msgs::PoseStamped ps;
geometry_msgs::PoseStamped uavCurrentLocalPose;
geometry_msgs::PoseStamped uavLockPose;
geometry_msgs::PoseStamped uavNextPose;

double uavRollENU, uavPitchENU, uavYawENU;
bool isSendNextPose;
bool isSendNextVelocity;
bool isSendNextPoseInitial;
bool isPositionReached;

int rosInfoTimeDealy;

mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void updateControlCommandStats()
{
  std::string controlCommandStatus = "";
  if (isSendNextVelocity)
    controlCommandStatus  += "[isSendNextVelocity = true]  ";
  else
    controlCommandStatus  += "[isSendNextVelocity = false]  ";

  if (isSendNextPose)
    controlCommandStatus  += "[isSendNextPose = true]  ";
  else
    controlCommandStatus  += "[isSendNextPose = false]  ";

  if (isSendNextPoseInitial)
    controlCommandStatus  += "[isSendNextPoseInitial = true]  ";
  else
    controlCommandStatus  += "[isSendNextPoseInitial = false]  ";
  ROS_INFO_STREAM(controlCommandStatus);

  std::string connectionStatus = "";
  if (current_state.connected)
    connectionStatus += "[Connected] ";
  else
    connectionStatus += "[Disconnected] ";
  
  if (current_state.armed)
    connectionStatus += "[Armed] ";
  else
    connectionStatus += "[Disarmed] ";

  if (current_state.mode == "OFFBOARD")
    connectionStatus += "[OFFBOARD] ";
  else
    connectionStatus += "[NOT OFFBOARD] ";
  ROS_INFO_STREAM(connectionStatus);
}


void sendCommand(const keyboard::Key &key)
{
  switch (key.code)
  {
    case 'q':
    {
      ROS_INFO_STREAM("up");
      vs.twist.linear.z += 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'a':
    {
      ROS_INFO_STREAM("down");
      vs.twist.linear.z -= 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'j':
    {
      ROS_INFO_STREAM("left");
      vs.twist.linear.y -= 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'l':
    {
      ROS_INFO_STREAM("right");
      vs.twist.linear.y += 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'i':
    {
      ROS_INFO_STREAM("forward");
      vs.twist.linear.x += 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'k':
    {
      ROS_INFO_STREAM("backward");
      vs.twist.linear.x -= 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'u':
    {
      ROS_INFO_STREAM("turn clockwise");
      vs.twist.angular.z += 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'o':
    {
      ROS_INFO_STREAM("turn anti-clockwise");
      vs.twist.angular.z -= 0.1;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'h':
    {
      ROS_INFO_STREAM("set all to zero");
      vs.twist.linear.x = 0.0;
      vs.twist.linear.y = 0.0;
      vs.twist.linear.z = 0.0; 
      vs.twist.angular.x = 0.0;
      vs.twist.angular.y = 0.0;
      vs.twist.angular.z = 0.0;
      isSendNextPose = false;      
      isSendNextVelocity = true; 
      updateControlCommandStats();
      break;
    }
    case 'n':
    {
      ROS_INFO_STREAM("get current lock position");
      uavLockPose.pose.position.x = uavCurrentLocalPose.pose.position.x;
      uavLockPose.pose.position.y = uavCurrentLocalPose.pose.position.y;
      uavLockPose.pose.position.z = uavCurrentLocalPose.pose.position.z;

      ROS_INFO("current lock position: Pos:[%0.3f, %0.3f, %0.3f]", uavLockPose.pose.position.x, uavLockPose.pose.position.y, uavLockPose.pose.position.z);

      uavNextPose.pose.position.x = uavLockPose.pose.position.x;
      uavNextPose.pose.position.y = uavLockPose.pose.position.y - 3.0;
      uavNextPose.pose.position.z = uavLockPose.pose.position.z + 3.0;

      ROS_INFO("next position: Pos:[%0.3f, %0.3f, %0.3f]", uavNextPose.pose.position.x, uavNextPose.pose.position.y, uavNextPose.pose.position.z);

      isSendNextPoseInitial = true;
      isSendNextPose = false;
      updateControlCommandStats();
      break;
    }
    case 'y':
    {
      isSendNextPose = true;      
      isSendNextVelocity = false;   
      break;
    }
    case 't':
    {
      offb_set_mode.request.custom_mode = "OFFBOARD";
      set_mode_client.call(offb_set_mode);

      if (offb_set_mode.response.success)
        ROS_WARN_STREAM("Offboard enabled");
      break;
    }
    case 'g':
    {
      arm_cmd.request.value = true;
      arming_client.call(arm_cmd);
      if (arm_cmd.response.success)
        ROS_WARN_STREAM("Vehicle armed");
      break;
    }
    case 'b':
    {
      arm_cmd.request.value = false;
      arming_client.call(arm_cmd);
      if (arm_cmd.response.success)
        ROS_WARN_STREAM("Vehicle disarmed");
      break;
    }
    default:
    {

    }
  }
}




void uavLocalPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
  
  //uavPose = *msg; 
  uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
  uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
  uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
  uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
  uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
  uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
  uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;
  
  // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
  tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);  
  tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);

  //ROS_INFO("local_position/local: Pos:[%0.3f, %0.3f, %0.3f], RPY:[%0.3f, %0.3f, %0.3f]", uavCurrentLocalPose.pose.position.x, uavCurrentLocalPose.pose.position.y, uavCurrentLocalPose.pose.position.z,uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);
  //ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926); 
  rosInfoTimeDealy++;
  if (rosInfoTimeDealy >=15)
  {
    ROS_INFO("local_position/local: Pos:[%0.3f, %0.3f, %0.3f], RPY:[%0.3f, %0.3f, %0.3f]", uavCurrentLocalPose.pose.position.x, uavCurrentLocalPose.pose.position.y, uavCurrentLocalPose.pose.position.z,uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);
    
    updateControlCommandStats();    
    rosInfoTimeDealy = 0;
  }

  if ( isSendNextPose )
  {
    double distance = sqrt((uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x) * (uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x)  +
    (uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y) * (uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y) + (uavCurrentLocalPose.pose.position.z - uavNextPose.pose.position.z) * (uavCurrentLocalPose.pose.position.z - uavNextPose.pose.position.z));
    double threshold = 0.2;
    if (distance < threshold)
    {
      ROS_INFO("Reached!!!");
      isSendNextPose = false;
      isSendNextVelocity = true;
      vs.twist.linear.x = 0.0;
      vs.twist.linear.y = 0.0;
      vs.twist.linear.z = 0.0;
      vs.twist.angular.x = 0.0;
      vs.twist.angular.y = 0.0;
      vs.twist.angular.z = 0.0;
    }
    else
    {
      double deltaX = uavCurrentLocalPose.pose.position.x - uavNextPose.pose.position.x;
      double deltaY = uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.y;
      double deltaZ = uavCurrentLocalPose.pose.position.y - uavNextPose.pose.position.z;
      ROS_INFO("Delta Pos:[%0.3f, %0.3f, %0.3f], Delta Distance:[%0.3f]", deltaX, deltaY, deltaZ, distance);
    }
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_velocity_control_node");
  ros::NodeHandle nodeHandle;
  ros::Publisher vel_sp_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/uav/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Publisher pos_sp_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/uav/mavros/setpoint_position/local",10);
  ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/uav/mavros/state", 10, state_cb);


  isSendNextPose = false;
  isSendNextPoseInitial = false;
  isSendNextVelocity = false;

  vs.twist.linear.x = 0.0;
  vs.twist.linear.y = 0.0;
  vs.twist.linear.z = 0.0; 
  vs.twist.angular.x = 0.0;
  vs.twist.angular.y = 0.0;
  vs.twist.angular.z = 0.0;

  rosInfoTimeDealy = 0;

  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown", 1, sendCommand);
  ros::Subscriber uavLocalPoseSubscriber = nodeHandle.subscribe("/mavros/local_position/pose", 1000, uavLocalPoseReceived);

  arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
  set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav/mavros/set_mode");


  ros::Rate loop_rate(20.0);

  while (ros::ok())
  {

    /*if ( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) 
    {
      if ( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) 
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } 
    else 
    {
      if ( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
      {
        if ( arming_client.call(arm_cmd) && arm_cmd.response.success) 
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }*/

    if (isSendNextVelocity == true)
    {
      vel_sp_pub.publish(vs);
    }

    if ((isSendNextPose == true) && (isSendNextPoseInitial == true))
    {
      ps.pose = uavNextPose.pose;
      ps.header.stamp = ros::Time::now();
      pos_sp_pub.publish(ps);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


}
