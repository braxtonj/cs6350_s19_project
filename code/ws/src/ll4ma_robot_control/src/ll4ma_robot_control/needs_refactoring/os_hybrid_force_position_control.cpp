#include "os_hybrid_force_position_control.h"
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <optoforce_etherdaq_driver/ZeroForceSensor.h>


bool HybridForcePositionController::configureHook()
{
  ROS_INFO("[HybridForcePosition] Initializing...");
  
  // call base controller initialization
  if (!OpSpaceController::configureHook())
  {
    ROS_ERROR_STREAM("[HybridForcePosition] Base initialization FAILURE");
    return false;
  }

  desired_in_cf_ = false;

  // TEMPORARY
  // sea_spring_.setZero(7); 
  // sea_cross_.setZero(7); 
  // sea_gravity_.setZero(7); 
  // sea_gravity_model_.setZero(7);


  w_err_.setZero(6);
  w_err_scaled_.setZero(6);
  w_des_.setZero(6);
  accum_w_err_.setZero(6);
  uf_.setZero(6);
  uf_damp_.setZero(6);
  xdot_.setZero(6);
  S_.setIdentity(6, 6);
  omega_.setZero(6, 6);
  omega_tilde_.setZero(6, 6);
  Kf_.setZero(6, 6);
  Ki_.setZero(6, 6);
  Kv_.setZero(6, 6);
  w_des_kdl_ = KDL::Wrench::Zero();
  w_des_base_ = KDL::Wrench::Zero();
  w_des_tool_ = KDL::Wrench::Zero();
  w_des_cf_ = KDL::Wrench::Zero();
  w_raw_base_ = KDL::Wrench::Zero();
  w_raw_tool_ = KDL::Wrench::Zero();
  w_raw_cf_ = KDL::Wrench::Zero();
  w_filt_base_ = KDL::Wrench::Zero();
  w_filt_tool_ = KDL::Wrench::Zero();
  w_filt_cf_ = KDL::Wrench::Zero();
  w_err_kdl_ = KDL::Wrench::Zero();
  ft_to_base_ = KDL::Frame::Identity();
  cf_to_ft_ = KDL::Frame::Identity();
  cf_to_base_ = KDL::Frame::Identity();
  base_to_cf_ = KDL::Frame::Identity();
  base_to_tool_ = KDL::Frame::Identity();
  base_to_cf_eigen_ = Eigen::Affine3d::Identity();
  R_base_to_cf_.setIdentity(3, 3);
  for (int i = 0; i < 6; i++)
  {
    robot_state_.wrench_error.push_back(0.0);
    robot_state_.pose_error.push_back(0.0);
  }

  // get parameters from param server
  bool success = true;
  success &= nh_.getParam("use_constraint_frame", use_constraint_frame_);
  success &= nh_.getParam("hybrid_control/integral_limits", integral_lims_);
  success &= nh_.getParam("hybrid_control/integral_decay_factor", integral_decay_factor_);
  success &= nh_.getParam("hybrid_control/w_error_scaling", w_error_scaling_);
  success &= getGainsFromParamServer("hybrid_control/x_p_gains", nh_, Kp_);
  success &= getGainsFromParamServer("hybrid_control/x_d_gains", nh_, Kd_);
  success &= getGainsFromParamServer("hybrid_control/f_p_gains", nh_, Kf_);
  success &= getGainsFromParamServer("hybrid_control/f_i_gains", nh_, Ki_);
  success &= getGainsFromParamServer("hybrid_control/f_v_gains", nh_, Kv_);
  success &= getGainsFromParamServer("hybrid_control/null_p_gains", nh_, Kp_null_);
  success &= getGainsFromParamServer("hybrid_control/null_d_gains", nh_, Kd_null_);

  if (!success)
  {
    ROS_ERROR("[HybridForcePosition] Failed to load params from param server.");
    return false;
  }
  
  // make sure integral decay is not amplifying accumulated error
  if (integral_decay_factor_ > 1.0 || integral_decay_factor_ < 0.0)
  {
    ROS_ERROR_STREAM("[HybridForcePosition] Integral decay factor " << integral_decay_factor_
		     << " is invalid. Must be in range [0.0, 1.0].");
    return false;
  }

  // make sure error scaling is not amplifying
  if (w_error_scaling_ > 1.0 || w_error_scaling_ < 0.0)
  {
    ROS_ERROR_STREAM("[HybridForcePosition] Wrench error scaling factor " << w_error_scaling_
		     << " is invalid. Must be in range [0.0, 1.0].");
    return false;
  }
    
  os_cmd_sub_ = nh_.subscribe(os_cmd_topic_, 1,
			      &HybridForcePositionController::opSpaceCmdCallback, this);

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
  
  ROS_INFO("[HybridForcePosition] Initialization complete.");
  return true;
}

bool HybridForcePositionController::startHook()
{
  ROS_INFO("[HybridForcePosition] Waiting for robot joint state...");
  ros::spinOnce();
  while (ros::ok() && !jnt_state_received_)
  {
    ros::spinOnce();
    rate_.sleep();
  }

  ROS_INFO("[HybridForcePosition] Joint state received.");

  if (use_ft_sensor_)
  {
    ROS_INFO("[HybridForcePosition] Waiting for wrench state...");
    while (ros::ok() && !wrench_state_received_)
    {
      ros::spinOnce();
      rate_.sleep();
    }
  }

  ROS_INFO("[HybridForcePosition] Wrench state received.");
  ROS_INFO("[HybridForcePosition] Robot state received.");

  setCurrentConfigAsDesired();
  
  ROS_INFO("[HybridForcePosition] Control loop is running.");
  ros::spinOnce();

  prev_time_ = ros::Time::now();
  
  while(ros::ok() && system_ok_)
  {
    this->updateHook();
    ros::spinOnce();
    rate_.sleep();
  }

  ROS_INFO("[HybridForcePosition] Control loop complete. Exiting.");
  stopHook();
  return true;
}

void HybridForcePositionController::updateHook()
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
  base_to_tool_ = x_kdl_.Inverse();
  
  // compute transform (FT -> base)
  fk_solver_->JntToCart(q_qdot_kdl_, x_xdot_kdl_, ft_index_);
  ft_to_base_ = x_xdot_kdl_.GetFrame();
  // compute transform (CF -> base)
  cf_to_base_ = ft_to_base_ * cf_to_ft_;
  // compute transform (base -> CF)
  base_to_cf_ = cf_to_base_.Inverse();
  // create block rotation matrix
  tf::transformKDLToEigen(base_to_cf_, base_to_cf_eigen_);
  R_base_to_cf_ = base_to_cf_eigen_.rotation();

  // transform forces to various frames for logging
  w_raw_base_ = ft_to_base_ * w_raw_;
  w_raw_tool_ = base_to_tool_ * w_raw_base_;
  w_raw_cf_ = base_to_cf_ * w_raw_base_;
  w_filt_base_ = ft_to_base_ * w_filt_;
  w_filt_tool_ = base_to_tool_ * w_filt_base_;
  w_filt_cf_ = base_to_cf_ * w_filt_base_;
  
  // transform desired forces to various frames for logging
  if (desired_in_cf_)
  {
    w_des_cf_ = w_des_kdl_;
    w_des_base_ = cf_to_base_ * w_des_kdl_;
  }
  else
  {
    w_des_cf_ = base_to_cf_ * w_des_kdl_;
    w_des_base_ = w_des_kdl_;
  }
  w_des_tool_ = base_to_tool_ * w_des_base_;
  
  // compute errors w.r.t. constraint frame (des - actual)
  x_err_kdl_ = KDL::diff(x_kdl_, x_des_kdl_);
  xdot_err_kdl_ = KDL::diff(xdot_kdl_, xdot_des_kdl_);
  w_err_kdl_ = KDL::diff(w_filt_base_, w_des_base_);

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
    xdotdot_des_[i] = xdotdot_des_kdl_[i];
    w_des_[i] = w_des_base_[i];
    w_err_[i] = w_err_kdl_[i];
    // attenuate error when negative to reduce chance of popping off surface once contact is made
    if (w_err_[i] * copysign(1.0, w_des_base_[i]) < 0.0)
      w_err_scaled_[i] = w_err_[i] * w_error_scaling_;
    else
      w_err_scaled_[i] = w_err_[i];
  }
 
  // wrench error accumulation for integral term
  accum_w_err_ += w_err_scaled_ * dt_.toSec();
  // saturate to limit
  for (int i = 0; i < 6; i++)
    if (std::abs(accum_w_err_[i]) > integral_lims_[i])
      accum_w_err_[i] = copysign(integral_lims_[i], accum_w_err_[i]);
  
  // create constraint selection tensors
  blockTensorMatrix(omega_, R_base_to_cf_, S_);
  blockTensorMatrix(omega_tilde_, R_base_to_cf_, I_6_ - S_);

  // compute task space inertia matrix
  getPseudoInverse(J_ * Mq_.inverse() * J_.transpose(), Mx_, 1.e-5);

  /* ======================================== Control Law ======================================== */

  // compute position and force control signals
  ux_ = omega_ * (Kp_ * x_err_ + Kd_ * xdot_err_);// + xdotdot_des_; // - Jdot_qdot_);
  uf_ = omega_tilde_ * (Kf_ * w_err_ + Ki_ * accum_w_err_);
  uf_damp_ = omega_tilde_ * Kv_ * xdot_;

  // joint space control signal (hybrid force/position)
  tau_ = J_.transpose() * (uf_ + omega_tilde_ * w_des_ + Mx_ * (ux_ - uf_damp_));



  // TEMPORARY
  // uf_.setZero(6);
  // uf_damp_.setZero(6);
  // tau_ = J_.transpose() * (uf_ + Mx_ * (ux_ - uf_damp_));



  // project posture into null space
  if (use_posture_)
  {
    tau_posture_ = Kp_null_ * (q_null_des_ - q_) - Kd_null_ * qdot_;
    nullSpaceProjection(tau_, tau_posture_, J_, Mx_, Mq_);
  }
  
  /* ============================================================================================= */

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

void HybridForcePositionController::stopHook()
{
  // nothing to do here for now.
}

void HybridForcePositionController::cleanupHook()
{
  // Nothing to do here for now.
}

void HybridForcePositionController::opSpaceCmdCallback(ll4ma_robot_control::OpSpaceCommand cmd_msg)
{
  // Call base callback
  OpSpaceController::opSpaceCmdCallback(cmd_msg);

  // Desired wrench in FT sensor frame (assuming that's how it comes in)
  tf::wrenchMsgToKDL(cmd_msg.wrench, w_des_kdl_);
  desired_in_cf_ = cmd_msg.desired_in_cf;

  // Constraint frame
  if (use_constraint_frame_)
    tf::poseMsgToKDL(cmd_msg.constraint_frame, cf_to_ft_);
  
  // Constraints selection
  if (cmd_msg.constraints.size() > 0)
    for (int i = 0; i < 6; i++)
      S_(i, i) = cmd_msg.constraints[i];
}

void HybridForcePositionController::publishCurrentRobotState()
{
  for (int i = 0; i < 6; i++)
  {
    robot_state_.wrench_error[i] = w_err_kdl_[i];
    robot_state_.pose_error[i] = x_err_[i];
  }
  
  // call base implementation
  OpSpaceController::publishCurrentRobotState();
}
