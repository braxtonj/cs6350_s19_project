#include "ll4ma_robot_control/joint_pd_control.h"


bool JointPDController::configureHook()
{
  log("Initializing...");

  if (!Controller::configureHook())
    return false;
    
  // Initialize Eigen
  q_des_.setZero(num_jnts_);
  q_dot_des_.setZero(num_jnts_);
  Kp_.setZero(num_jnts_, num_jnts_);
  Kd_.setZero(num_jnts_, num_jnts_);

  // Get params from ROS parameter server
  bool success = true;
  success &= nh_.getParam("jnt_des_topic", jnt_des_topic_);
  success &= getGainsFromParamServer("p_gains", nh_, Kp_);
  success &= getGainsFromParamServer("d_gains", nh_, Kd_);
  if (!success)
  {
    log("Could not load parameters from param server.", ERROR);
    return false;
  }
    
  // Initialize ROS
  jnt_des_sub_ = nh_.subscribe(jnt_des_topic_, 1, &JointPDController::jointDesCallback, this);
  reload_srv_ = nh_.advertiseService("/joint_pd_control/reload_parameters",
                                     &JointPDController::reloadControlParameters, this);
  
  initialized_ = true;
  log("Initialization complete.");
  return true;
}


bool JointPDController::startHook()
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


void JointPDController::updateHook()
{
  // compute torque command based on PD control law
  tau_ = Kp_ * (q_des_ - q_) + Kd_ * (q_dot_des_ - q_dot_);
  
  // add in gravity compensation
  robot_interface_->compensateGravity(tau_, q_);
  
  // publish torque command to robot
  robot_interface_->publishTorqueCommand(tau_);

  // publish robot state to other nodes
  robot_interface_->publishRobotState(q_, q_dot_);
}


void JointPDController::stopHook()
{
  log("Control loop exited.");
}


void JointPDController::cleanupHook()
{
  // nothing to do here for now
}


void JointPDController::jointDesCallback(sensor_msgs::JointState cmd)
{
  for (int i = 0; i < num_jnts_; ++i)
  {
    q_des_[i] = cmd.position[i];
    // TODO read in desired velocities (should error check)
  }
}


void JointPDController::setCurrentConfigAsDesired()
{
  log("Setting current configuration as initial setpoint.");
  for (int i = 0; i < num_jnts_; ++i)
    q_des_[i] = q_[i];
}


bool JointPDController::reloadControlParameters(std_srvs::Empty::Request &req,
                                                std_srvs::Empty::Response &resp)
{
  log("Reloading controller parameters...");

  bool success = true;
  success &= getGainsFromParamServer("p_gains", nh_, Kp_);
  success &= getGainsFromParamServer("d_gains", nh_, Kd_);
  if (success)
  {
    log("Successfully reset control parameters.");
    return true;
  }
  else
  {
    log("Failed to load parameters from param server.", WARN); 
    return false;
  }  
}


void JointPDController::log(std::string msg)
{
  log(msg, INFO);
}


void JointPDController::log(std::string msg, LogLevel level)
{
  switch(level)
  {
    case WARN :
    {
      ROS_WARN_STREAM("[JointPDController] " << msg);
      break;
    }
    case ERROR :
    {
      ROS_ERROR_STREAM("[JointPDController] " << msg);
      break;    
    }
    default:
    {
      ROS_INFO_STREAM("[JointPDController] " << msg);
      break;    
    }
  }
}
