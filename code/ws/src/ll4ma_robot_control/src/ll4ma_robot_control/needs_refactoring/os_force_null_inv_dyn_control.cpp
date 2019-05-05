#include "os_force_null_inv_dyn_control.h"
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <optoforce_etherdaq_driver/ZeroForceSensor.h>


bool OpSpaceForceNullInvDynController::configureHook()
{
  ROS_INFO("[ForceNullPosition] Initializing...");

  // call base controller initialization
  if (!OpSpaceController::configureHook())
  {
    ROS_ERROR("[ForceNullPosition] Base instantiation FAILURE");
    return false;
  }

  w_err_.setZero(6);
  accum_w_err_.setZero(6);
  uf_.setZero(6);
  xdot_.setZero(6);
  S_.setIdentity(6, 6);
  omega_.setZero(6, 6);
  omega_tilde_.setZero(6, 6);
  Kf_.setZero(6, 6);
  Ki_.setZero(6, 6);
  Kv_.setZero(6, 6);
  Mxc_.setZero(6, 6);
  Mxc_tilde_.setZero(6, 6);
  Jc_.setZero(6, num_jnts_);
  Jc_tilde_.setZero(6, num_jnts_);
  tau_position_.setZero(num_jnts_);
  w_des_kdl_ = KDL::Wrench::Zero();
  w_err_kdl_ = KDL::Wrench::Zero();
  w_des_base_ = KDL::Wrench::Zero();


  // TODO need to fix this controller to be consistent with hybrid (forces in different frames)
  
  // w_base_ = KDL::Wrench::Zero();
  ft_to_base_ = KDL::Frame::Identity();
  cf_to_ft_ = KDL::Frame::Identity();
  cf_to_base_ = KDL::Frame::Identity();
  base_to_cf_ = KDL::Frame::Identity();
  base_to_cf_eigen_ = Eigen::Affine3d::Identity();
  R_base_to_cf_.setIdentity(3, 3);
  for (int i = 0; i < 6; i++)
    robot_state_.wrench_error.push_back(0.0);  
  
  // get parameters from param server
  bool success = true;
  success &= nh_.getParam("use_constraint_frame", use_constraint_frame_);
  success &= nh_.getParam("force_null_control/integral_limits", integral_lims_);
  success &= nh_.getParam("force_null_control/integral_decay_factor", integral_decay_factor_);
  success &= nh_.getParam("force_null_control/w_error_scaling", w_error_scaling_);
  success &= getGainsFromParamServer("force_null_control/x_p_gains", nh_, Kp_);
  success &= getGainsFromParamServer("force_null_control/x_d_gains", nh_, Kd_);
  success &= getGainsFromParamServer("force_null_control/f_p_gains", nh_, Kf_);
  success &= getGainsFromParamServer("force_null_control/f_i_gains", nh_, Ki_);
  success &= getGainsFromParamServer("force_null_control/f_v_gains", nh_, Kv_);
  success &= getGainsFromParamServer("force_null_control/null_p_gains", nh_, Kp_null_);
  success &= getGainsFromParamServer("force_null_control/null_d_gains", nh_, Kd_null_);

  if (!success)
  {
    ROS_ERROR("[ForceNullPosition] Failed to load params from param server.");
    return false;
  }

  // make sure integral decay is not amplifying accumulated error
  if (integral_decay_factor_ > 1.0 || integral_decay_factor_ < 0.0)
  {
    ROS_ERROR_STREAM("[ForceNullPosition] Integral decay factor " << integral_decay_factor_
		     << " is invalid. Must be in range [0.0, 1.0].");
    return false;
  }

  // make sure error scaling is not amplifying
  if (w_error_scaling_ > 1.0 || w_error_scaling_ < 0.0)
  {
    ROS_ERROR_STREAM("[ForceNullPosition] Wrench error scaling factor " << w_error_scaling_
		     << " is invalid. Must be in range [0.0, 1.0].");
    return false;
  }

  os_cmd_sub_ = nh_.subscribe(os_cmd_topic_, 1,
			      &OpSpaceForceNullInvDynController::opSpaceCmdCallback, this);

  // try to zero out the force sensor ASSUMING IT STARTS IN FREESPACE
  if (!is_sim_ && use_ft_sensor_)
  {
    ft_client_ = nh_.serviceClient<optoforce_etherdaq_driver::ZeroForceSensor>("/zero_force_sensor");
    ft_client_.waitForExistence();
    optoforce_etherdaq_driver::ZeroForceSensor ft_srv;
    ft_srv.request.set_zero = true;
    if (ft_client_.call(ft_srv))
    {
      forces_zeroed_ = true;
      ROS_INFO("[ForceNullPosition] Force sensor zeroed.");
    }
    else
    {
      ROS_ERROR("[ForceNullPosition] Force sensor could not be zeroed.");
      return false;
    }    
  }

  ROS_INFO_STREAM("[ForceNullPosition] Initialization complete.");
  return true;
}

bool OpSpaceForceNullInvDynController::startHook()
{
  ROS_INFO("[ForceNullPosition] Waiting for robot joint state...");
  ros::spinOnce();
  while (ros::ok() && !jnt_state_received_)
  {
    ros::spinOnce();
    rate_.sleep();
  }

  ROS_INFO("[ForceNullPosition] Joint state received.");

  if (use_ft_sensor_)
  {
    ROS_INFO("[ForceNullPosition] Waiting for wrench state...");
    while (ros::ok() && !wrench_state_received_)
    {
      ros::spinOnce();
      rate_.sleep();
    }
  }

  ROS_INFO("[ForceNullPosition] Wrench state received.");
  ROS_INFO("[ForceNullPosition] Robot state received.");

  setCurrentConfigAsDesired();
  
  ROS_INFO("[ForceNullPosition] Control loop is running.");
  ros::spinOnce();

  prev_time_ = ros::Time::now();
  
  while(ros::ok())
  {
    updateHook();
    ros::spinOnce();
    rate_.sleep();
  }

  ROS_INFO("[ForceNullPosition] Control loop complete. Exiting.");
  stopHook();
  return true;
}

void OpSpaceForceNullInvDynController::updateHook()
{
  time_ = ros::Time::now();
  dt_ = time_ - prev_time_;

  // use solvers to get current control components
  fk_solver_->JntToCart(q_qdot_kdl_, x_xdot_kdl_);
  jac_solver_->JntToJac(q_qdot_kdl_.q, J_kdl_);
  jac_dot_solver_->JntToJacDot(q_qdot_kdl_, jdot_qdot_kdl_);
  dyn_model_solver_->JntToMass(q_qdot_kdl_.q, Mq_kdl_);  
  dyn_model_solver_->JntToGravity(q_qdot_kdl_.q, G_kdl_);
  dyn_model_solver_->JntToCoriolis(q_qdot_kdl_.q, q_qdot_kdl_.qdot, C_kdl_);
  
  // pose and twist in base frame
  x_kdl_ = x_xdot_kdl_.GetFrame();
  xdot_kdl_ = x_xdot_kdl_.GetTwist();

  // compute transform (FT -> base)
  fk_solver_->JntToCart(q_qdot_kdl_, x_xdot_kdl_, ft_index_);
  ft_to_base_ = x_xdot_kdl_.GetFrame();
  // compute transform (CF -> base)
  cf_to_base_ = ft_to_base_ * cf_to_ft_;
  // compute transform (base -> CF)
  base_to_cf_ = cf_to_base_.Inverse();
  tf::transformKDLToEigen(base_to_cf_, base_to_cf_eigen_);
  R_base_to_cf_ = base_to_cf_eigen_.rotation();

  // transform forces to base frame
  w_des_base_ = ft_to_base_ * w_des_kdl_;
  // w_base_ = ft_to_base_ * w_filt_;
  
  // compute errors w.r.t. base frame (des - actual)
  x_err_kdl_ = KDL::diff(x_kdl_, x_des_kdl_);
  xdot_err_kdl_ = KDL::diff(xdot_kdl_, xdot_des_kdl_);
  // w_err_kdl_ = KDL::diff(w_base_, w_des_base_);

  // KDL to Eigen conversions
  J_ = J_kdl_.data;
  Mq_ = Mq_kdl_.data;
  G_ = G_kdl_.data;
  C_ = C_kdl_.data;
  
  for (int i = 0; i < 6; i++)
  {
    Jdot_qdot_[i] = jdot_qdot_kdl_[i];
    x_err_[i] = x_err_kdl_[i];
    xdot_err_[i] = xdot_err_kdl_[i];
    w_err_[i] = w_err_kdl_[i];
    xdot_[i] = xdot_kdl_[i];
    xdotdot_des_[i] = xdotdot_des_kdl_[i];
    // attenuate error when negative to reduce chance of popping off surface once contact is made    
    if (w_err_[i] * copysign(1.0, w_des_base_[i]) < 0.0)
      w_err_[i] *= w_error_scaling_;
  }

  // wrench error accumulation for integral term
  accum_w_err_ += w_err_ * dt_.toSec();
  // saturate to limit
  for (int i = 0; i < 6; i++)
    if (std::abs(accum_w_err_[i]) > integral_lims_[i])
      accum_w_err_[i] = copysign(integral_lims_[i], accum_w_err_[i]);

  // constraint Jacobians
  blockTensorMatrix(omega_, R_base_to_cf_, S_);
  blockTensorMatrix(omega_tilde_, R_base_to_cf_, I_6_ - S_);
  Jc_ = omega_ * J_;
  Jc_tilde_ = omega_tilde_ * J_;

  // task space inertia matrices
  getPseudoInverse(J_ * Mq_.inverse() * J_.transpose(), Mx_, 1.e-5);
  getPseudoInverse(Jc_ * Mq_.inverse() * Jc_.transpose(), Mxc_, 1.e-5);
  getPseudoInverse(Jc_tilde_ * Mq_.inverse() * Jc_tilde_.transpose(), Mxc_tilde_, 1.e-5);
  
  /* ======================================== Control Law ======================================== */
  
  // compute position and force control signals
  ux_ = Kp_ * x_err_ + Kd_ * xdot_err_ + xdotdot_des_; // + xdotdot_des_ - Jdot_qdot_; TODO reincorporate these

  // TODO figure out velocity damping
  uf_ = Kf_ * w_err_ + Ki_ * accum_w_err_ - Kv_ * Mxc_tilde_ * xdot_; // TODO feedforward desired?
  
  // joint space control signal (primary force)
  tau_ = Jc_tilde_.transpose() * uf_;
    
  // project secondary objective (position control) into null space of force control
  tau_position_ = Jc_.transpose() * Mxc_ * ux_;
  nullSpaceProjection(tau_, tau_position_, Jc_tilde_, Mxc_tilde_, Mq_);

  // project posture into remaining null space
  if (use_posture_)
  {
    tau_posture_ = Kp_null_ * (q_null_des_ - q_) - Kd_null_ * qdot_;
    nullSpaceProjection(tau_, tau_posture_, J_, Mx_, Mq_);
  }
  
  /* ============================================================================================= */

  // command joint torques if no safety checks have been violated
  if (system_ok_)
  {
    if (compensate_gravity_)
      robot_interface_->compensateGravity(tau_, G_);
    robot_interface_->publishTorqueCommand(tau_);
  }
  
  // publish current robot state
  publishCurrentRobotState();

  // decay integral error
  accum_w_err_ *= integral_decay_factor_;
  
  prev_time_ = time_;
}

void OpSpaceForceNullInvDynController::stopHook()
{
  // Nothing to do here for now.
}

void OpSpaceForceNullInvDynController::cleanupHook()
{
  // Nothing to do here for now.
}

void OpSpaceForceNullInvDynController::opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg)
{
  // Call base callback
  OpSpaceController::opSpaceCmdCallback(cmd_msg);

  // Desired wrench in FT sensor frame (assuming that's how it comes in)
  tf::wrenchMsgToKDL(cmd_msg.wrench, w_des_kdl_);

  // Constraint frame
  if (use_constraint_frame_)
    tf::poseMsgToKDL(cmd_msg.constraint_frame, cf_to_ft_);
  
  // Constraints selection
  if (cmd_msg.constraints.size() > 0)
    for (int i = 0; i < 6; i++)
      S_(i, i) = cmd_msg.constraints[i];
}

void OpSpaceForceNullInvDynController::publishCurrentRobotState()
{
  for (int i = 0; i < 6; i++)
    robot_state_.wrench_error[i] = w_err_[i];
  
  // call base implementation
  OpSpaceController::publishCurrentRobotState();
}
