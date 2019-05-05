#ifndef LL4MA_OS_FORCE_NULL_INV_DYN_CONTROL
#define LL4MA_OS_FORCE_NULL_INV_DYN_CONTROL

#include "os_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <ll4ma_robot_control/OpSpaceCommand.h>


class OpSpaceForceNullInvDynController: public controller_interface::OpSpaceController
{
private:
  bool use_constraint_frame_;
  std::vector<double> integral_lims_;
  double integral_decay_factor_;
  double w_error_scaling_;
  
  // KDL
  KDL::Wrench w_des_kdl_, w_err_kdl_, w_des_base_, w_base_;
  KDL::Frame ft_to_base_, cf_to_ft_, cf_to_base_, base_to_cf_;

  // Eigen
  Eigen::VectorXd w_err_, accum_w_err_, uf_, tau_position_, xdot_;
  Eigen::MatrixXd Kf_, Ki_, Kv_, S_, omega_, omega_tilde_, Jc_, Jc_tilde_, R_base_to_cf_;
  Eigen::MatrixXd Mxc_, Mxc_tilde_;
  Eigen::Affine3d base_to_cf_eigen_;

  // ROS
  ros::Subscriber os_cmd_sub_;
  ros::ServiceClient ft_client_;
  void opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg);
  void publishCurrentRobotState();
  
public:
OpSpaceForceNullInvDynController(double rate, ros::NodeHandle &nh) : OpSpaceController(rate, nh) {}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};

#endif
