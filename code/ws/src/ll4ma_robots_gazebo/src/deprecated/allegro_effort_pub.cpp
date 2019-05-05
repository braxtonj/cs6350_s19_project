#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <std_msgs/Char.h>
#include <urlg_msgs/JointCommands.h>
#include <vector>
#include <string>
#ifndef DEG2RAD
#define DEG2RAD 3.14/180
#endif
using namespace std;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"allegro_effort_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Publisher joint_pub=n.advertise<urlg_msgs::JointCommands>("allegro_hand_right/control/joint_cmd",1);
	ros::Publisher control_pub=n.advertise<std_msgs::Char>("allegro_hand_right/control/control_type",1);
	while(ros::ok())
	{
		
	  std_msgs::Char c_meth;
	  c_meth.data='e';
	  control_pub.publish(c_meth);
	  urlg_msgs::JointCommands j_cmd;
	  j_cmd.name={"joint5","joint4","joint0","joint1","joint2","joint3","joint6"};
	  j_cmd.position={1,0.4,0,0,0,0,0};
	  joint_pub.publish(j_cmd);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
	return(0);
}
