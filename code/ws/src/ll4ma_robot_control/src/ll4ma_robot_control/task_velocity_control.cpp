#include "ll4ma_robot_control/task_velocity_control.h"
#include <lbr4_robot_interface/lbr4_interface.h>

#define POSE_3D_DOF 6

bool TaskVelocityController::configureHook()
{
  log("Initializing...");

  if (!Controller::configureHook())
    return false;

  // Initialize Eigen
  q_.setZero(num_jnts_);
  q_prime_.setZero(num_jnts_);
  q_dot_.setZero(num_jnts_);
  // q_des_.setZero(num_jnts_);
  q_dot_des_.setZero(num_jnts_);
  q_dot_posture_.setZero(num_jnts_);
  x_dot_des_.setZero(POSE_3D_DOF);
  I_N_.setIdentity(num_jnts_, num_jnts_);

  tf_ = shared_ptr<tf::TransformListener>(new tf::TransformListener());

  // ROS
  q_dot_des_ros_.position = std::vector<double>(num_jnts_);
  q_dot_des_ros_.velocity = std::vector<double>(num_jnts_);

  // Get root name from the robot interface
  base_link_ = robot_interface_->getRootNames()[robot_chain_idx_];

  // Get params from ROS parameter server
  nh_.param("pseudoinverse_tolerance", pseudoinverse_tolerance_, 1.e-5);
  nh_.param("dq_finite_diff", dq_, 1.e-5);
  nh_.param("use_redundancy_resolution", use_redundancy_resolution_, true);

  bool success = true;
  success &= nh_.getParam("task_des_topic", task_des_topic_);
  success &= nh_.getParam("joint_des_topic", joint_des_topic_);

  if (!success)
  {
    log("Could not load parameters from param server.", ERROR);
    return false;
  }

  // Initialize ROS
  task_des_sub_ = nh_.subscribe(task_des_topic_, 1, &TaskVelocityController::taskDesCallback, this);
  reload_srv_ = nh_.advertiseService("/task_velocity_control/reload_parameters",
                                     &TaskVelocityController::reloadControlParameters, this);
  joint_des_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_des_topic_,100);

  log("Initialization complete.");
  return true;
}


bool TaskVelocityController::startHook()
{
  log("Waiting for robot state...");
  while (nh_.ok() && !robot_state_received_)
  {
    ros::spinOnce();
    rate_.sleep();
  }
  log("Robot state received!");

  prev_time_ = ros::Time::now();
  log("Controller is running...");
  while (nh_.ok())
  {
    updateHook();
    ros::spinOnce();
    rate_.sleep();
  }

  log("Control loop exited.");
  return true;
}


void TaskVelocityController::updateHook()
{
  cur_time_ = ros::Time::now();
  dt_ = cur_time_ - prev_time_;

  // Get Jacobian and take pseudoinverse (J+)
  robot_interface_->kdl_->getJacobian(robot_chain_idx_, q_, J_);
  getPseudoInverse(J_, J_cross_, pseudoinverse_tolerance_);

  // Compute desired control
  q_dot_des_ = J_cross_*x_dot_des_;

  if (use_redundancy_resolution_)
  {
    // TODO: Select desired resolution method easily here
    computeManipulabilityRedundancyResolution(q_dot_posture_, J_);
    nullSpaceProjection(q_dot_des_, q_dot_posture_, J_, J_cross_);
  }

  // Convert to ROS type and publish q_des over topic
  for (int i =0; i < num_jnts_; ++i)
  {
    // update desired position to match current position + delta_t_*q_dot
    double delta_q = dt_.toSec()*q_dot_des_[i];
    q_dot_des_ros_.position[i] = q_[i] + delta_q;
    q_dot_des_ros_.velocity[i] = q_dot_des_[i];
  }

  // Set appropriate header info for ros msg
  q_dot_des_ros_.header.stamp = ros::Time::now();
  q_dot_des_ros_.header.frame_id = base_link_;
  joint_des_pub_.publish(q_dot_des_ros_);

  prev_time_ = cur_time_;
}


void TaskVelocityController::stopHook()
{
  // nothing to do here for now
}


void TaskVelocityController::cleanupHook()
{
  // nothing to do here for now
}


void TaskVelocityController::taskDesCallback(geometry_msgs::TwistStamped cmd)
{
  log("Updating x_dot_desired");
  // Convert into the root frame prior to setting x_dot_des_
  if (cmd.header.frame_id != base_link_)
  {
    // NOTE: Not realtime safe
    tf::Vector3 twist_rot(cmd.twist.angular.x,
                          cmd.twist.angular.y,
                          cmd.twist.angular.z);
    tf::Vector3 twist_vel(cmd.twist.linear.x,
                          cmd.twist.linear.y,
                          cmd.twist.linear.z);
    // log("Converting command velocity from frame " + cmd.header.frame_id + " to " + base_link_);
    try{
      tf_->lookupTransform(base_link_, cmd.header.frame_id, cmd.header.stamp, transform_);
      out_rot_ = transform_.getBasis()*twist_rot;
      out_vel_ = transform_.getBasis()*twist_vel + transform_.getOrigin().cross(out_rot_);
    } catch (const std::exception& e)
    {
      for (int i = 0; i < 3; ++i)
      {
        out_vel_[i] = 0.0;
        out_rot_[i] = 0.0;
      }
      log(e.what(), ERROR);
    }
  }
  x_dot_des_[0] = out_vel_[0];
  x_dot_des_[1] = out_vel_[1];
  x_dot_des_[2] = out_vel_[2];
  x_dot_des_[3] = out_rot_[0];
  x_dot_des_[4] = out_rot_[1];
  x_dot_des_[5] = out_rot_[2];
}


bool TaskVelocityController::reloadControlParameters(std_srvs::Empty::Request &req,
                                                     std_srvs::Empty::Response &resp)
{
  log("Reloading controller parameters...");

  bool success = true;
  // success &= getGainsFromParamServer("p_gains", nh_, Kp_);
  // success &= getGainsFromParamServer("d_gains", nh_, Kd_);
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

void TaskVelocityController::computeManipulabilityRedundancyResolution(Eigen::VectorXd &q_dot_posture_,
                                                                       Eigen::MatrixXd &J_)
{
  JJ_trans_ = J_*J_.transpose();
  double U = computeManipulabilityScore(JJ_trans_);

  // Update q_prime to current q_
  for (int i = 0; i < num_jnts_; ++i)
  {
    q_prime_[i] = q_[i];
  }

  for (int i = 0; i < num_jnts_; ++i)
  {
    // Adjust q_[i] by a small ammount to estimate the gradient
    q_prime_[i] = q_[i] + dq_;

    // Get Jacobian at q_prime_ and compute associated manipulability score
    robot_interface_->kdl_->getJacobian(robot_chain_idx_, q_prime_, J_prime_);
    JJ_trans_ = J_prime_*J_prime_.transpose();
    double U_prime = computeManipulabilityScore(JJ_trans_);
    // Compute gradient estimate
    q_dot_posture_[i] = (U-U_prime)/dq_;
    // Reset q_prime_ for next joint
    q_prime_[i] = q_[i];
  }
}


double TaskVelocityController::computeManipulabilityScore(Eigen::MatrixXd& JJ_trans_)
{
  return sqrt(JJ_trans_.determinant());
}


void TaskVelocityController::computeJointLimitRedundancyResolution(Eigen::VectorXd &q_dot_posture_,
                                                                   Eigen::MatrixXd &J_)
{
}


void TaskVelocityController::log(std::string msg)
{
  log(msg, INFO);
}


void TaskVelocityController::log(std::string msg, LogLevel level)
{
  switch(level)
  {
    case WARN :
    {
      ROS_WARN_STREAM("[TaskVelocityController] " << msg);
      break;
    }
    case ERROR :
    {
      ROS_ERROR_STREAM("[TaskVelocityController] " << msg);
      break;
    }
    default:
    {
      ROS_INFO_STREAM("[TaskVelocityController] " << msg);
      break;
    }
  }
}
