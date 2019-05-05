#ifndef LL4MA_PARALLEL_FORCE_POSITION_CONTROL
#define LL4MA_PARALLEL_FORCE_POSITION_CONTROL

#include "os_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <ll4ma_robot_control/OpSpaceCommand.h>


class ParallelForcePositionController: public controller_interface::OpSpaceController
{
private:
  // additional variables to define over base
  Eigen::Matrix<double, 6, 1> wrench_, prev_wrench_, wrench_des_, wrench_err_;
  Eigen::Matrix<double, 6, 1> Kf_, Ki_;

  std::string ft_link_;
  int ft_index_;
  
  // KDL variables for handling transforms
  KDL::Wrench wrench_kdl_, wrench_des_kdl_;
  KDL::FrameVel ft_x_xdot_kdl_;

  // ROS subscribers and publishers
  ros::Subscriber op_space_cmd_sub_, wrench_state_sub_;
  ros::Publisher joint_cmd_pub_;
  
  // callback functions
  void opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg);
  void wrenchStateCallback(geometry_msgs::WrenchStamped state_msg);
  
public:
  ParallelForcePositionController(double rate) : OpSpaceController(rate) {}

  bool init(ros::NodeHandle &nh);
  void starting(const ros::Time &time);
  void update(const ros::Time &time);
  void stopping(const ros::Time &time);
};

#endif
