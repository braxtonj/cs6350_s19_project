#ifndef LL4MA_HYBRID_FORCE_POSITION_CONTROL
#define LL4MA_HYBRID_FORCE_POSITION_CONTROL

#include "os_control.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <ll4ma_robot_control/OpSpaceCommand.h>


class HybridForcePositionController: public controller_interface::OpSpaceController
{
private:
  
  bool use_constraint_frame_, desired_in_cf_;  
  std::vector<double> integral_lims_;
  double integral_decay_factor_;
  double w_error_scaling_;

  // KDL
  KDL::Wrench w_des_kdl_, w_des_base_, w_des_tool_, w_des_cf_;
  KDL::Wrench w_raw_base_, w_raw_tool_, w_raw_cf_;
  KDL::Wrench w_filt_base_, w_filt_tool_, w_filt_cf_;
  KDL::Wrench w_err_kdl_;
  KDL::Frame ft_to_base_, cf_to_ft_, cf_to_base_, base_to_cf_, base_to_tool_;

  // Eigen
  Eigen::VectorXd w_des_, w_err_, w_err_scaled_, accum_w_err_, uf_, uf_damp_, xdot_;
  Eigen::MatrixXd Kf_, Ki_, Kv_, S_, omega_, omega_tilde_, R_base_to_cf_;
  Eigen::Affine3d base_to_cf_eigen_;

  // TEMPORARY for debugging Baxter gravity comp
  /* Eigen::VectorXd sea_spring_, sea_cross_, sea_gravity_, sea_gravity_model_; */

  
  // ROS
  ros::Subscriber os_cmd_sub_;
  ros::ServiceClient ft_client_;
  void opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg);
  void publishCurrentRobotState();
  
public:
HybridForcePositionController(double rate, ros::NodeHandle &nh) : OpSpaceController(rate, nh) {}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};

#endif
