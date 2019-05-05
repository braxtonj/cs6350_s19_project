#include "baxter_robot_interface/baxter_interface.h"


namespace robot_interface
{

  bool BaxterInterface::init()
  {
    ROS_INFO("[BaxterInterface] Initializing...");

    if (!RobotInterface::init())
    {
      ROS_ERROR("[BaxterInterface] Base interface initialization FAILURE.");
      return false;
    }

    bool success = true;
    success &= nh_.getParam("sea_jnt_state_topic", sea_jnt_state_topic_);
    success &= nh_.getParam("disable_gravity_topic", disable_gravity_topic_);
    success &= nh_.getParam("spring_compensation_factor", spring_factor_);
    
    if (!success)
    {
      ROS_ERROR("[BaxterInterface] Initialization FAILURE. Could not load params from  parameter server.");
      return false;
    }

    jnt_cmd_pub_ = nh_.advertise<baxter_core_msgs::JointCommand>(jnt_cmd_topic_, 1);
    disable_baxter_gravity_pub_ = nh_.advertise<std_msgs::Empty>(disable_gravity_topic_, 1);
    sea_jnt_state_sub_ = nh_.subscribe(sea_jnt_state_topic_, 1,
                                       &BaxterInterface::seaJointStateCallback, this);

    tau_spring_.setZero(num_jnts_);
    tau_cross_.setZero(num_jnts_);
    gravity_only_.setZero(num_jnts_);
    gravity_model_effort_.setZero(num_jnts_);


    this->initJointCommand(jnt_cmd_);
    
    ROS_INFO("[BaxterInterface] Waiting for SEA joint state...");
    while (ros::ok() && !sea_jnt_state_received_)
      ros::spinOnce();
    ROS_INFO("[BaxterInterface] SEA joint state received.");
      
    ROS_INFO("[BaxterInterface] Initialization complete.");
    return true;
  }
  
  void BaxterInterface::publishTorqueCommand(Eigen::VectorXd &torques)
  {
    this->saturateTorques(torques);
    for (int i = 0; i < num_jnts_; i++)
      jnt_cmd_.command[i] = torques[i];
    jnt_cmd_pub_.publish(jnt_cmd_);
  }

  void BaxterInterface::compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &gravity)
  {
    // get rid of built-in Baxter gravity/spring compensation
    disable_baxter_gravity_pub_.publish(empty_msg_);
    // subtract off spring torques
    tau -= tau_spring_;
    // subtract off crosstalk
    tau -= tau_cross_;
    // add in computed gravity torques
    tau += gravity;
  }

  void BaxterInterface::initJointCommand(baxter_core_msgs::JointCommand &jnt_cmd)
  {
    ROS_INFO("[BaxterInterface] Initializing joint command...");
    jnt_cmd = baxter_core_msgs::JointCommand();
    jnt_cmd.mode = jnt_cmd.TORQUE_MODE;
    for (int i=0; i < num_jnts_; ++i)
    {
      jnt_cmd.command.push_back(0.0);
      jnt_cmd.names.push_back(jnt_names_[i]);
    }
    
    ROS_INFO("[BaxterInterface] Joint command initialization complete.");
  }

  void BaxterInterface::seaJointStateCallback(baxter_core_msgs::SEAJointState jnt_state)
  {
    // get torques for the spring model and crosstalk
    for (int i = 0; i < num_jnts_; i++)
    {
      tau_spring_[i] = spring_factor_ * jnt_state.hysteresis_model_effort[i];
      tau_cross_[i] = jnt_state.crosstalk_model_effort[i];
      // TODO this is for debugging the compensation torques
      gravity_only_[i] = jnt_state.gravity_only[i];
      gravity_model_effort_[i] = jnt_state.gravity_model_effort[i];
    }
    
    if (!sea_jnt_state_received_)
      sea_jnt_state_received_ = true;
  }

  void BaxterInterface::getSEAJointState(Eigen::VectorXd &spring, Eigen::VectorXd &cross, 
					 Eigen::VectorXd &gravity, Eigen::VectorXd &gravity_model)
  {
    for (int i = 0; i < num_jnts_; i++)
    {
      spring[i] = tau_spring_[i];
      cross[i] = tau_cross_[i];
      gravity[i] = gravity_only_[i];
      gravity_model[i] = gravity_model_effort_[i];
    }
  }

}
