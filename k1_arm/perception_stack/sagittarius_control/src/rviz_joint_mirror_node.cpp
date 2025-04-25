/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */

/* 
 * Code adapted and maintained by Syed Abbas, syed.ahussain@ku.ac.ae — 2025 
 */


 #include <ros/ros.h>
 #include "sdk_sagittarius_arm/ArmRadControl.h"
 #include "sdk_sagittarius_arm/ArmInfo.h"
 #include <sensor_msgs/JointState.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/String.h>
 #include <std_srvs/Empty.h>
 #include <string>
 sensor_msgs::JointState joint_states;        
 
 /// @brief Joint state callback function
 /// @param msg - updated joint states
 void joint_state_cb(const sensor_msgs::JointState &msg)
 {
   joint_states = msg;
 }
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "rviz_joint_mirror");
   ros::NodeHandle n;
 
   // Subscribe to the main robotic arm's joint angles
   ros::Subscriber sub_positions = n.subscribe("joint_states", 100, joint_state_cb);
   ros::Publisher pub_positions = n.advertise<sdk_sagittarius_arm::ArmRadControl>("joint/commands", 100);
   ros::Publisher pub_gripper = n.advertise<std_msgs::Float64>("gripper/command", 100);
   ros::ServiceClient srv_robot_info = n.serviceClient<sdk_sagittarius_arm::ArmInfo>("get_robot_info");
   // Release the main robotic arm's torque lock (servo lock)
   ros::Publisher pub_torque = n.advertise<std_msgs::String>("control_torque", 1);
   ros::Rate loop_rate(100);
   bool success;
 
   // Wait until the SDK node starts subscribing before sending messages
   while ((pub_positions.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
   {
     ros::spinOnce();
     loop_rate.sleep();
   }
   while ((pub_torque.getNumSubscribers() < 1) && ros::ok())
   {
     ros::spinOnce();
     loop_rate.sleep();
   }
   std_msgs::String msg;
   msg.data = "close";
   pub_torque.publish(msg);
 
   // Get the number of joints on the robotic arm
   sdk_sagittarius_arm::ArmInfo robot_info_srv;
   success = srv_robot_info.call(robot_info_srv);
   if (!success)
   {
     ROS_ERROR("Could not get robot info.");
     return 1;
   }
 
   size_t cntr = 0;
   while (ros::ok())
   {
     sdk_sagittarius_arm::ArmRadControl pos_msg;
     for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
     {
       pos_msg.rad.push_back(joint_states.position.at(i));
     }
     pub_positions.publish(pos_msg);
 
     if (cntr == 10)
     {
       std_msgs::Float64 gpr_msg;
       gpr_msg.data = joint_states.position.at(joint_states.position.size()-1);
       pub_gripper.publish(gpr_msg);
       cntr = 0;
     }
     cntr++;
     ros::spinOnce();
     loop_rate.sleep();
   }
   return 0;
 }
 