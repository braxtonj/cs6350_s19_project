#include "reflex_robot_interface/reflex_interface.h"


namespace robot_interface
{
  
  bool ReflexInterface::init()
  {
    log("Initializing...");

    if (!RobotInterface::init())
      return false;

    jnt_names_.clear();
    // TODO getting the joint names this way is kind of hacked. It's complicated because the loaded
    // URDF can include both the arm and the hand, and the full arm/hand chain is being used to
    // compute gravity comp. Ideally you wouldn't need arm in there to do gravity, but then you have
    // to account for changing gravity vector. This works for now, but can be simplified later if
    // gravity comp is done differently in the future.
    for (int i = 1; i <= 3; ++i) // if arm present: finger chains are 1, 2, 3. 0 is arm
    {
      vector<std::string> jnt_names = kdl_->getJointNames(i);
      if (i < 3)
      {
	for (int j = jnt_names.size() - 2; j < jnt_names.size(); ++j)
	  jnt_names_.push_back(jnt_names[j]);
      }
      else
      {
	for (int j = jnt_names.size() - 1; j < jnt_names.size(); ++j)
	  jnt_names_.push_back(jnt_names[j]);
      }
    }

    // sort the joint names
    std::sort(jnt_names_.begin(), jnt_names_.end());
  
    num_jnts_ = jnt_names_.size();
    
    // Initialize Eigen
    tau_.setZero(num_jnts_);
    tau_g_.setZero(num_jnts_);
    finger_1_q_.setZero(2);
    finger_2_q_.setZero(2);
    finger_3_q_.setZero(1);
    finger_1_tau_g_.setZero(2);
    finger_2_tau_g_.setZero(2);
    finger_3_tau_g_.setZero(1);
    
    // Initialize ROS
    initJointState(jnt_cmd_);
    
    log("Initialization complete.");
    return true;
  }


  void ReflexInterface::compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &q)
  {
    // This is all hardcoded, since there's only 5 joints. Otherwise indexing with loops is messy.

    // populate finger joint angles from angle vector
    finger_1_q_[0] = q[0]; // preshape 1
    finger_1_q_[1] = q[2]; // proximal 1
    finger_2_q_[0] = q[1]; // preshape 2
    finger_2_q_[1] = q[3]; // proximal 2
    finger_3_q_[0] = q[4]; // proximal 3

    // compute the gravity compensation for each finger
    kdl_->getGtau(1, finger_1_q_, finger_1_tau_g_);
    kdl_->getGtau(2, finger_2_q_, finger_2_tau_g_);
    kdl_->getGtau(3, finger_3_q_, finger_3_tau_g_);

    // populate torque vector from individual fingers
    tau_g_[0] = finger_1_tau_g_[0]; // preshape 1
    tau_g_[2] = finger_1_tau_g_[1]; // proximal 1
    tau_g_[1] = finger_2_tau_g_[0]; // preshape 2
    tau_g_[3] = finger_2_tau_g_[1]; // proximal 2
    tau_g_[4] = finger_3_tau_g_[0]; // proximal 2

    // add gravity into passed in torque
    tau_ += tau_g_;
  }


  void ReflexInterface::publishTorqueCommand(Eigen::VectorXd &torques)
  {
    saturateTorques(torques);
    
    for (int i = 0; i < num_jnts_; ++i)
      jnt_cmd_.effort[i] = torques[i];
      
    jnt_cmd_pub_.publish(jnt_cmd_);
  }


  void ReflexInterface::log(std::string msg)
  {
    log(msg, INFO);
  }


  void ReflexInterface::log(std::string msg, LogLevel level)
  {
    switch(level)
    {
      case WARN :
      {
        ROS_WARN_STREAM("[ReflexInterface] " << msg);
        break;
      }
      case ERROR :
      {
        ROS_ERROR_STREAM("[ReflexInterface] " << msg);
        break;    
      }
      default:
      {
        ROS_INFO_STREAM("[ReflexInterface] " << msg);
        break;    
      }
    }
  }

} // namespace robot_interface
