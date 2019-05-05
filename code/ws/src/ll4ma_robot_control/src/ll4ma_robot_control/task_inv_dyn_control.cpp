#include "ll4ma_robot_control/task_inv_dyn_control.h"

#define POSE_3D_DOF 6


bool TaskInvDynController::configureHook()
{
  log("Initializing...");

  if (!Controller::configureHook())
    return false;

  // Initialize Eigen
  x_ = Eigen::Affine3d::Identity();
  x_des_ = Eigen::Affine3d::Identity();
  x_dot_.setZero(POSE_3D_DOF);
  x_dot_des_.setZero(POSE_3D_DOF);
  x_err_.setZero(POSE_3D_DOF);
  x_dot_err_.setZero(POSE_3D_DOF);
  q_null_des_.setZero(num_jnts_); 
  tau_posture_.setZero(num_jnts_);
  J_.setZero(POSE_3D_DOF, num_jnts_);
  Mq_.setZero(num_jnts_, num_jnts_);
  Mx_.setZero(POSE_3D_DOF, POSE_3D_DOF);
  Kp_.setZero(POSE_3D_DOF, POSE_3D_DOF);
  Kd_.setZero(POSE_3D_DOF, POSE_3D_DOF);
  Kp_null_.setZero(num_jnts_, num_jnts_);
  Kd_null_.setZero(num_jnts_, num_jnts_);
  I_N_.setZero(num_jnts_, num_jnts_);
  
  // Get params from ROS parameter server
  nh_.param<bool>("compensate_gravity", compensate_gravity_, true);
  nh_.param<bool>("use_posture", use_posture_, false);
  bool success = true;
  success &= getGainsFromParamServer("x_p_gains", nh_, Kp_);
  success &= getGainsFromParamServer("x_d_gains", nh_, Kd_);
  success &= getGainsFromParamServer("null_p_gains", nh_, Kp_null_);
  success &= getGainsFromParamServer("null_d_gains", nh_, Kd_null_);
  success &= nh_.getParam("task_des_topic", task_des_topic_);
  if (!success)
  {
    log("Could not load parameters from param server.", ERROR);
    return false;
  }

    // Initialize ROS
  task_des_sub_ = nh_.subscribe(task_des_topic_, 1, &TaskInvDynController::taskDesCallback, this);

  initialized_ = true;
  log("Initialization complete.");
  return true;
}


bool TaskInvDynController::startHook()
{
  log("Waiting for robot state...");
  while (nh_.ok() && !robot_state_received_)
  {
    ros::spinOnce();
    rate_.sleep();
  }
  log("Robot state received!");

  setCurrentConfigAsDesired();
  return true;
}


void TaskInvDynController::updateHook()
{
  // Get current dynamics control components
  robot_interface_->getJacobian(q_, J_);
  robot_interface_->getJointMass(q_, Mq_);
  robot_interface_->getTaskMass(Mq_, J_, Mx_);
  robot_interface_->getTaskError(q_, q_dot_, x_des_, x_dot_des_, x_err_, x_dot_err_);
  
  /* ======================================== Control Law ======================================== */
  
  // TODO add in desired acceleration and jdotqdot
  tau_ = J_.transpose() * Mx_ * (Kp_ * x_err_ + Kd_ * x_dot_err_);

  if (compensate_gravity_)
    robot_interface_->compensateGravity(tau_, q_);

  if (use_posture_)
  {
    tau_posture_ = Kp_null_ * (q_null_des_ - q_) - Kd_null_ * q_dot_;
    nullSpaceProjection(tau_, tau_posture_, J_, Mx_, Mq_);
  }
  
  /* ============================================================================================= */  
  
  robot_interface_->publishTorqueCommand(tau_);

  robot_interface_->publishRobotState(q_, q_dot_);
}


void TaskInvDynController::stopHook()
{
  log("Control loop exited.");
}


void TaskInvDynController::cleanupHook()
{
  // nothing to do here for now.
}


void TaskInvDynController::setCurrentConfigAsDesired()
{
  log("Setting current configuration as initial setpoint.");
  // set null space posture
  // for (int i = 0; i < num_jnts_; i++)
  //   q_null_des_[i] = q_[i];
  robot_interface_->getFK(q_, x_des_);
}


void TaskInvDynController::taskDesCallback(ll4ma_trajectory_msgs::TaskTrajectoryPoint cmd)
{
  x_des_ = Eigen::Translation3d(cmd.pose.position.x,
                                cmd.pose.position.y,
                                cmd.pose.position.z) *
           Eigen::Quaterniond(cmd.pose.orientation.w,
                              cmd.pose.orientation.x,
                              cmd.pose.orientation.y,
                              cmd.pose.orientation.z);
}


void TaskInvDynController::log(std::string msg)
{
  log(msg, INFO);
}


void TaskInvDynController::log(std::string msg, LogLevel level)
{
  switch(level)
  {
    case WARN :
    {
      ROS_WARN_STREAM("[TaskInvDynController] " << msg);
      break;
    }
    case ERROR :
    {
      ROS_ERROR_STREAM("[TaskInvDynController] " << msg);
      break;    
    }
    default:
    {
      ROS_INFO_STREAM("[TaskInvDynController] " << msg);
      break;    
    }
  }
}
