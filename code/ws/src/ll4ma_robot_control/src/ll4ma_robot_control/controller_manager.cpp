#include "ll4ma_robot_control/controller_manager.h"
#include "ll4ma_robot_control/joint_pd_control.h"
#include "ll4ma_robot_control/task_inv_dyn_control.h"


bool ControllerManager::init()
{
  log("Initializing controllers...");
  
  std::string control_type_str;
  if (!nh_.getParam("control_type", control_type_str))
  {
    log("Could not load parameters from param server.", ERROR);
    return false;
  }

  switch_control_srv_ = nh_.advertiseService("switch_controller",
                                             &ControllerManager::switchController, this);

  current_control_type_ = type_map_[control_type_str];
  switch(current_control_type_)
  {
    case JOINT_PD:
    {
      controller_.reset(new JointPDController(nh_.getNamespace(), robot_interface_, rate_val_));
      break;
    }
    case TASK_INV_DYN:
    {
      controller_.reset(new TaskInvDynController(nh_.getNamespace(), robot_interface_, rate_val_));
      break;
    }
    default:
    {
      log("Unknown control type.", ERROR);
      return false;
    }
  }
  
  if (!controller_->configureHook())
  {
    log("Could not initialize controller.", ERROR);
    return false;
  }

  do_update_ = true;
  log("Initialization complete.");
  return true;
}


void ControllerManager::run()
{
  log("Starting the controller...");
  if (controller_->startHook())
  {
    log("Control loop is running.");
    while (nh_.ok())
    {
      ros::spinOnce();
      if (do_update_)
        controller_->updateHook();
      rate_.sleep();
    }
  }
  else
    log("Could not start the controller.", ERROR);
}


bool ControllerManager::switchController(ll4ma_robot_control::SwitchControl::Request &req,
                                         ll4ma_robot_control::SwitchControl::Response &resp)
{
  log("Switching controller...");
  ControlType desired_control_type = type_map_[req.control_type];

  if (current_control_type_ == desired_control_type)
  {
    log("Specified controller is already active.", WARN);
    return true;
  }
      
  
  if (!resetControllerType(desired_control_type))
  {
    log("Could not reset the controller type.", ERROR);
    return false;
  }

  if (!desired_controller_->configureHook())
  {
    log("Could not initialize the new controller.", ERROR);
    return false;
  }

  if (!desired_controller_->startHook())
  {
    log("Could not start the new controller.", ERROR);
    return false;
  }

  do_update_ = false;
  controller_->stopHook();
  controller_->cleanupHook();
  controller_ = desired_controller_;
  do_update_ = true;
  
  log("Control loop is running.");
  log("Controller switched successfully.");
  return true;
}


bool ControllerManager::resetControllerType(ControlType control_type)
{
  switch(control_type)
  {
    case JOINT_PD:
    {
      desired_controller_.reset(new JointPDController(nh_.getNamespace(), robot_interface_,
                                                      rate_val_));
      current_control_type_ = JOINT_PD;
      break;
    }
    case TASK_INV_DYN:
    {
      desired_controller_.reset(new TaskInvDynController(nh_.getNamespace(), robot_interface_,
                                                         rate_val_));
      current_control_type_ = TASK_INV_DYN;
      break;
    }
    default:
    {
      log("Unknown control type.", ERROR);
      return false;
    }
  }
  return true;
}


void ControllerManager::log(std::string msg)
{
  log(msg, INFO);
}


void ControllerManager::log(std::string msg, LogLevel level)
{
  switch(level)
  {
    case WARN :
    {
      ROS_WARN_STREAM("[ControllerManager] " << msg);
      break;
    }
    case ERROR :
    {
      ROS_ERROR_STREAM("[ControllerManager] " << msg);
      break;    
    }
    default:
    {
      ROS_INFO_STREAM("[ControllerManager] " << msg);
      break;    
    }
  }
}
