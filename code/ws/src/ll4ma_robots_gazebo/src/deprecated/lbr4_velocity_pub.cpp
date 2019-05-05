// Publishes velocity in N-m  to all the joints of KUKA lbr4 robot
// Author: bala@cs.utah.edu
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <std_msgs/Char.h>
#include <urlg_msgs/JointCommands.h>
#include <vector>
#include <string>

using namespace std;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"lbr4_velocity_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Publisher joint_pub=n.advertise<urlg_msgs::JointCommands>("lbr4/control/joint_cmd",1);
	ros::Publisher control_pub=n.advertise<std_msgs::Char>("lbr4/control/control_type",1);
	while(ros::ok())
	{
		
	  std_msgs::Char c_meth;
	  c_meth.data='v';
	  control_pub.publish(c_meth);
	  urlg_msgs::JointCommands j_cmd;
	  j_cmd.name={"joint5","joint4","joint0","joint1","joint2","joint3","joint6"};
	  j_cmd.velocity={10,0.4,1,1,1,1,1};
	  joint_pub.publish(j_cmd);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
	return(0);
}
