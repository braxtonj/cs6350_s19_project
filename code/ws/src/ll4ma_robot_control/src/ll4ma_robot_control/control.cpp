#include "ll4ma_robot_control/control.h"


bool Controller::configureHook()
{
  log("Initializing...");
  initialized_ = false; // !!! Set this to true in configureHook of children classes.

  // Get params from ROS paramter server
  bool success = true;
  success &= nh_.getParam("compensate_gravity", compensate_gravity_);
  success &= nh_.getParam("jnt_state_topic", jnt_state_topic_);
  if (!success)
  {
    log("Failed to load params from param server.", ERROR);
    return false;
  }
  
  system_ok_ = true;
  robot_state_received_ = false;
  num_jnts_ = robot_interface_->getNumJoints();
  nh_.setParam("/num_joints", num_jnts_); // TODO should set on robot namespace
  
  // Initialize Eigen
  q_.setZero(num_jnts_);
  q_dot_.setZero(num_jnts_);
  tau_.setZero(num_jnts_);

  // Initialize ROS
  jnt_state_sub_ = nh_.subscribe(jnt_state_topic_, 1, &Controller::jointStateCallback, this);
  
  log("Initialization complete.");
  return true;
}


void Controller::jointStateCallback(sensor_msgs::JointState state)
{
  if (!robot_state_received_)    
    robot_state_received_ = true;

  for (int i = 0; i < num_jnts_; ++i)
  {
    q_[i] = state.position[i];
    q_dot_[i] = state.velocity[i];
  }
}


bool Controller::isInitialized()
{
  return initialized_;
}


void Controller::log(std::string msg)
{
  log(msg, INFO);
}


void Controller::log(std::string msg, LogLevel level)
{
  switch(level)
  {
    case WARN :
    {
      ROS_WARN_STREAM("[Controller] " << msg);
      break;
    }
    case ERROR :
    {
      ROS_ERROR_STREAM("[Controller] " << msg);
      break;    
    }
    default:
    {
      ROS_INFO_STREAM("[Controller] " << msg);
      break;    
    }
  }
}
