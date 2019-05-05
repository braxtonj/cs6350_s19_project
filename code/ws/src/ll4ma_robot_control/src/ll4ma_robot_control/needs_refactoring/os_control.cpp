#include "os_control.h"
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolvervel_recursive.hpp>


namespace controller_interface
{
  bool OpSpaceController::configureHook()
  {
    ROS_INFO("[BaseOSControl] Initializing...");

    forces_zeroed_ = false;

    if (!BaseController::configureHook())
    {
      ROS_ERROR("[BaseOSControl] Parent controller initialization failed.");
      return false;
    }

    bool success = true;
    success &= nh_.getParam("posture", use_posture_);
    success &= nh_.getParam("use_ft_sensor", use_ft_sensor_);
    success &= nh_.getParam("transform_wrench", transform_wrench_);
    success &= nh_.getParam("ft_topic", ft_topic_);
    success &= nh_.getParam("filtered_ft_topic", filtered_ft_topic_);
    success &= nh_.getParam("os_cmd_topic", os_cmd_topic_);
    success &= nh_.getParam("force_limits", force_lims_);

    if (!success)
    {
      ROS_ERROR("[BaseOSControl] Failed to load parameters from param server.");
      return false;
    }
    
    // KDL
    J_kdl_.resize(num_jnts_);
    Mq_kdl_.resize(num_jnts_);
    w_raw_ = KDL::Wrench::Zero();
    w_filt_ = KDL::Wrench::Zero();
    jdot_qdot_kdl_ = KDL::Twist::Zero();
    xdot_des_kdl_ = KDL::Twist::Zero();
    xdotdot_des_kdl_ = KDL::Twist::Zero();

    // Eigen
    x_err_.setZero(6);
    x_err_accum_.setZero(6);
    xdot_err_.setZero(6);
    xdotdot_des_.setZero(6);
    Jdot_qdot_.setZero(6);
    ux_.setZero(6);
    q_.setZero(num_jnts_);
    qdot_.setZero(num_jnts_);
    q_null_des_.setZero(num_jnts_);
    tau_.setZero(num_jnts_);
    tau_posture_.setZero(num_jnts_);
    J_.setZero(6, num_jnts_);
    Mx_.setZero(6, 6);
    Kp_.setZero(6, 6);
    Kd_.setZero(6, 6);
    Mq_.setZero(num_jnts_, num_jnts_);
    Kp_null_.setZero(num_jnts_, num_jnts_);
    Kd_null_.setZero(num_jnts_, num_jnts_);

    I_N_.setIdentity(num_jnts_, num_jnts_);
    I_3_.setIdentity(3, 3);
    I_4_ = Eigen::Affine3d::Identity();
    I_6_.setIdentity(6, 6);

    // Initialize kinematics/dynamics solvers
    fk_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
    
    if (use_ft_sensor_)
    {
      nh_.getParam("ft_link", ft_link_);
      // find the index of the force sensor
      for (int i=0; i<kdl_chain_.getNrOfSegments(); i++)
	if (kdl_chain_.getSegment(i).getName() == ft_link_)
	  ft_index_ = i;
      ft_index_ += 1; // chain iteration is not inclusive, so add one to include FT link
    }
    
    // ROS subscribers and publishers
    os_cmd_sub_ = nh_.subscribe(os_cmd_topic_, 1, &OpSpaceController::opSpaceCmdCallback, this);
    jnt_state_sub_ = nh_.subscribe(jnt_state_topic_, 1, &OpSpaceController::jointStateCallback, this);
    wrench_state_sub_ = nh_.subscribe(ft_topic_, 1, &OpSpaceController::wrenchStateCallback, this);
    filtered_wrench_sub_ = nh_.subscribe(filtered_ft_topic_, 1,
					 &OpSpaceController::filteredWrenchStateCallback, this);
    robot_state_pub_ = nh_.advertise<ll4ma_robot_control::RobotState>("robot_state", 1);

    this->initJointState(robot_state_.joint_state);
    
    ROS_INFO("[BaseOSControl] Initialization complete.");

    return true;
  }  
  
  void OpSpaceController::setCurrentConfigAsDesired()
  {
    ROS_INFO("[BaseOSControl] Setting current configuration as initial setpoint.");
    for (int i = 0; i < num_jnts_; i++)
    {
      // set null space posture
      q_null_des_[i] = q_[i];
      // Eigen to KDL conversions
      q_qdot_kdl_.q(i) = q_[i];
      q_qdot_kdl_.qdot(i) = qdot_[i];
    }
    fk_solver_->JntToCart(q_qdot_kdl_, x_xdot_kdl_);
    x_des_kdl_ = x_xdot_kdl_.GetFrame();
  }

  void OpSpaceController::opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg)
  {
    tf::poseMsgToKDL(cmd_msg.pose, x_des_kdl_);
    tf::twistMsgToKDL(cmd_msg.twist, xdot_des_kdl_);
    tf::twistMsgToKDL(cmd_msg.accel, xdotdot_des_kdl_);

    // desired null posture
    // if (cmd_msg.null_posture.size() > 0)
    //   for (int i = 0; i < cmd_msg.null_posture.size(); i++)
    // 	q_null_des_(i) = cmd_msg.null_posture[i];  
  }
  
  void OpSpaceController::jointStateCallback(sensor_msgs::JointState jnt_msg)
  {    
    // save the current state
    for (int i = 0; i < num_jnts_; i++)
    {
      q_[i] = jnt_msg.position[i];
      qdot_[i] = jnt_msg.velocity[i];
      // Eigen to KDL conversions
      q_qdot_kdl_.q(i) = q_[i];
      q_qdot_kdl_.qdot(i) = qdot_[i];
    }
    
    // set current as desired if this is the first state received
    if (!jnt_state_received_)
    {
      this->setCurrentConfigAsDesired();
      jnt_state_received_ = true;
    }
  }
  
  void OpSpaceController::wrenchStateCallback(geometry_msgs::WrenchStamped wrench_stmp)
  {
    // wrench in force/torque sensor frame
    tf::wrenchMsgToKDL(wrench_stmp.wrench, w_raw_);
    // negate so it's force robot is applying instead of force being applied to robot
    w_raw_ = -w_raw_;
    // make sure forces don't exceed safety limit, kill control if so
    if (forces_zeroed_ && system_ok_)
    {
      for (int i = 0; i < 6; i++)
      {
	if (std::abs(w_raw_[i]) > force_lims_[i])
	{
	  system_ok_ = false;
	  ROS_ERROR("[BaseOSControl] Sensed forces exceeded safety limit:");
	  ROS_ERROR("[BaseOSControl] Forces:");
	  for (int j = 0; j < 6; j++)
	    ROS_ERROR_STREAM(w_raw_[j]);
	  ROS_ERROR("[BaseOSControl] Killing controller.");
	  break;
	}
      }
    }

    if (!wrench_state_received_)
      wrench_state_received_ = true;
  }

  void OpSpaceController::filteredWrenchStateCallback(geometry_msgs::WrenchStamped wrench_stmp)
  {
    // TODO it's probably temporary to have two different callbacks on wrench, I think it should
    // just be on this one (filtered) and the controller won't care about the raw values. But for
    // now I want to compare raw and filtered through the logger so temporarily having both.
    tf::wrenchMsgToKDL(wrench_stmp.wrench, w_filt_);
    // negate so it's force robot is applying instead of force being applied to robot
    w_filt_ = -w_filt_;

    // if (!wrench_state_received_)
    //   wrench_state_received_ = true; // assuming we want to use filtered forces
  }

  void OpSpaceController::publishCurrentRobotState()
  {
    tf::poseKDLToMsg(x_kdl_, robot_state_.pose);
    
    robot_state_.twist.linear.x = xdot_kdl_[0];
    robot_state_.twist.linear.y = xdot_kdl_[1];
    robot_state_.twist.linear.z = xdot_kdl_[2];
    robot_state_.twist.angular.x = xdot_kdl_[3];
    robot_state_.twist.angular.y = xdot_kdl_[4];
    robot_state_.twist.angular.z = xdot_kdl_[5];
    
    robot_state_.wrench.force.x = w_raw_[0];
    robot_state_.wrench.force.y = w_raw_[1];
    robot_state_.wrench.force.z = w_raw_[2];
    robot_state_.wrench.torque.x = w_raw_[3];
    robot_state_.wrench.torque.y = w_raw_[4];
    robot_state_.wrench.torque.z = w_raw_[5];
    
    for (int i = 0; i < num_jnts_; i++)
    {
      robot_state_.joint_state.position[i] = q_[i];
      robot_state_.joint_state.velocity[i] = qdot_[i];
    }

    robot_state_pub_.publish(robot_state_);
  }

  void OpSpaceController::blockTensorMatrix(Eigen::MatrixXd &M,
					    const Eigen::MatrixXd &R,
					    const Eigen::MatrixXd &S)
  {
    M.block<3,3>(0,0) = R.transpose() * S.block<3,3>(0,0) * R;
    M.block<3,3>(3,3) = R.transpose() * S.block<3,3>(3,3) * R;
  }
  
} // namespace controller_interface

