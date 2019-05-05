#include "allegro_robot_interface/allegro_interface.h"


namespace robot_interface
{

  bool AllegroInterface::init()
  {
    log("Initializing...");
    
    if (!RobotInterface::init())
    {
      log("Base interface initialization FAILURE.", ERROR);
      return false;
    }
    jnt_names_.clear();
    // TODO getting the joint names this way is kind of hacked.
    // It's complicated because the loaded URDF can include both
    // the arm and the hand, and the full arm/hand chain is being
    // used to compute gravity comp. Ideally you wouldn't need arm
    // in there to do gravity, but then you have to account for
    // changing gravity vector. This works for now, but can be
    // simplified later if gravity comp is done differently in the future.
    int num_chains = kdl_->getNumChains(); 
    int start_idx = num_chains > 4 ? 1 : 0; // hack to work on real and sim robot
    for (int i = start_idx; i < num_chains; ++i) // if arm present: finger chains are 1, 2, 3, 4. 0 is arm
    {
      vector<std::string> jnt_names = kdl_->getJointNames(i);
      // iterate only over the finger joints and ignore the arm joints
      for (int j = jnt_names.size() - 4; j < jnt_names.size(); ++j)
	jnt_names_.push_back(jnt_names[j]);
    }
  
    num_jnts_ = jnt_names_.size();
    
    // Initialize Eigen
    tau_.setZero(num_jnts_);
    tau_g_.setZero(num_jnts_);
    finger_q_.setZero(4);
    finger_tau_g_.setZero(4);
    
    // Initialize ROS
    initJointState(jnt_cmd_);
    log("Initialization Complete");
    return true;
  }


  void AllegroInterface::compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &q)
  {
    for (int i = 0; i < 4; ++i) // for each finger on hand
    {
      for (int j = 0; j < 4; ++j) // for each joint on finger
	finger_q_[j] = q[i*4+j];
      kdl_->getGtau(i+1, finger_q_, finger_tau_g_); // i+1 to ignore arm-only chain
      for (int j = 0; j < 4; ++j)
	tau_g_[i*4+j] = finger_tau_g_[j];
    }
    tau_ += tau_g_;
  }
  
  
  void AllegroInterface::log(std::string msg)
  {
    log(msg, INFO);
  }
  
  
  void AllegroInterface::log(std::string msg, LogLevel level)
  {
    switch(level)
    {
      case WARN :
      {
	ROS_WARN_STREAM("[AllegroInterface] " << msg);
	break;
      }
      case ERROR :
      {
	ROS_ERROR_STREAM("[AllegroInterface] " << msg);
	break;    
      }
      default:
      {
	ROS_INFO_STREAM("[AllegroInterface] " << msg);
	break;    
      }
    }
  }
  

  void AllegroInterface::publishRealRobot(const Eigen::VectorXd &cmd,ControlMode &cm)
  {
    jnt_cmd_.position.resize(num_jnts_,0.0);
    jnt_cmd_.velocity.resize(num_jnts_,0.0);
    jnt_cmd_.effort.resize(num_jnts_,0.0);
    
    switch(cm)
    {
      case (POS): Eigen::VectorXd::Map(&jnt_cmd_.position[0],cmd.size())=cmd;
	jnt_cmd_.velocity.clear();
	jnt_cmd_.effort.clear();
	break;
      case (VEL): Eigen::VectorXd::Map(&jnt_cmd_.velocity[0],cmd.size())=cmd;
	jnt_cmd_.position.clear();
	jnt_cmd_.effort.clear();
	break;
      case (EFF): Eigen::VectorXd::Map(&jnt_cmd_.effort[0],cmd.size())=cmd;
	jnt_cmd_.position.clear();
	jnt_cmd_.velocity.clear();
	break;
    }
    jnt_cmd_.header.stamp=ros::Time();
    jnt_cmd_pub_.publish(jnt_cmd_);
  }
  
  
  void AllegroInterface::publishRealRobot(const Eigen::VectorXd &q_cmd,
					  const Eigen::VectorXd &q_dot_cmd)
  {
    jnt_cmd_.position.resize(num_jnts_,0.0);
    jnt_cmd_.velocity.resize(num_jnts_,0.0);
    Eigen::VectorXd::Map(&jnt_cmd_.position[0],q_cmd.size())=q_cmd;
    Eigen::VectorXd::Map(&jnt_cmd_.velocity[0],q_dot_cmd.size())=q_dot_cmd;
    jnt_cmd_.header.stamp=ros::Time();
    jnt_cmd_pub_.publish(jnt_cmd_);
  }

} // namespace robot_interface
