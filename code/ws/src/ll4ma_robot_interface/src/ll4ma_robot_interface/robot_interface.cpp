#include "ll4ma_robot_interface/robot_interface.h"


#define POSE_3D_DOF 6

namespace robot_interface
{

  bool RobotInterface::init()
  {
    log("Initializing...");

    // read paramaters from ROS parameter server
    bool success = true;
    success &= nh_.getParam("tip_links", tip_links_);
    success &= nh_.getParam("root_links", root_links_);
    success &= nh_.getParam("torque_limits", torque_lims_);
    nh_.param<std::string>("jnt_cmd_topic", jnt_cmd_topic_, "joint_cmd");
    nh_.param<std::string>("robot_state_topic", robot_state_topic_, "robot_state");
    if (!success)
    {
      log("Could not load parameters from param server.", ERROR);
      return false;
    }
    
    // Initialize KDL
    kdl_.reset(new manipulator_kdl::robotKDL("/robot_description", nh_, root_links_,
                                             tip_links_, gravity_));

    jnt_names_ = kdl_->getJointNames();
    num_jnts_ = jnt_names_.size();
    
    // Initialize Eigen
    tau_.setZero(num_jnts_);
    tau_g_.setZero(num_jnts_);

    // Initialize ROS
    initJointState(jnt_cmd_);
    initJointState(robot_state_.joint_state);
    jnt_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(jnt_cmd_topic_, 1);
    robot_state_pub_ = nh_.advertise<ll4ma_robot_control_msgs::RobotState>(robot_state_topic_, 1);

    log("Initialization complete.");
    return true;
  }


  void RobotInterface::saturateTorques(Eigen::VectorXd &torques)
  {
    for (int i = 0; i < num_jnts_; ++i)
      if (std::abs(torques[i]) > torque_lims_[i])
	torques[i] = copysign(torque_lims_[i], torques[i]);
  }


  void RobotInterface::publishTorqueCommand(Eigen::VectorXd &torques)
  {
    saturateTorques(torques);
    
    for (int i = 0; i < num_jnts_; ++i)
      jnt_cmd_.effort[i] = torques[i];
    jnt_cmd_pub_.publish(jnt_cmd_);
  }


  void RobotInterface::publishRobotState(Eigen::VectorXd &q, Eigen::VectorXd &q_dot)
  {
    for (int i = 0; i < num_jnts_; ++i)
    {
      robot_state_.joint_state.position[i] = q[i];
      robot_state_.joint_state.velocity[i] = q_dot[i];
    }

    robot_state_pub_.publish(robot_state_);
  }


  void RobotInterface::publishRobotState(Eigen::VectorXd &q, Eigen::VectorXd &q_dot,
                                         Eigen::Affine3d &x, Eigen::VectorXd &x_dot)
  {
    for (int i = 0; i < num_jnts_; ++i)
    {
      robot_state_.joint_state.position[i] = q[i];
      robot_state_.joint_state.velocity[i] = q_dot[i];
    }

    robot_state_.pose.position.x = x.translation()[0];
    robot_state_.pose.position.y = x.translation()[1];
    robot_state_.pose.position.z = x.translation()[2];
    Eigen::Quaterniond quat = (Eigen::Quaterniond)x.linear();
    robot_state_.pose.orientation.x = quat.x();
    robot_state_.pose.orientation.y = quat.y();
    robot_state_.pose.orientation.z = quat.z();
    robot_state_.pose.orientation.w = quat.w();
    // from eigen_conversions, I guess they're just standardizing choice between q and -q?
    if (robot_state_.pose.orientation.w < 0)
    {
      robot_state_.pose.orientation.x *= -1;
      robot_state_.pose.orientation.y *= -1;
      robot_state_.pose.orientation.z *= -1;
      robot_state_.pose.orientation.w *= -1;
    }

    robot_state_.twist.linear.x = x_dot[0];
    robot_state_.twist.linear.y = x_dot[1];
    robot_state_.twist.linear.z = x_dot[2];
    robot_state_.twist.angular.x = x_dot[3];
    robot_state_.twist.angular.y = x_dot[4];
    robot_state_.twist.angular.z = x_dot[5];
    
    robot_state_pub_.publish(robot_state_);
  }


  void RobotInterface::compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &q)
  {
    kdl_->getGtau(0, q, tau_g_);
    tau += tau_g_;
  }


  double RobotInterface::getNumJoints()
  {
    return num_jnts_;
  }

  
  std::vector<std::string> RobotInterface::getRootNames()
  {
    return root_links_;
  }

  
  std::vector<std::string> RobotInterface::getTipNames()
  {
    return tip_links_;
  }


  // void RobotInterface::getFK(Eigen::VectorXd &q, Eigen::Affine3d &x)
  // {
  //   getFK(0, q, x);
  // }


  void RobotInterface::getFK(Eigen::VectorXd &q, Eigen::Affine3d &x, const int &ch_idx)
  {
    kdl_->getFK(ch_idx, q, x);
  }


  // void RobotInterface::getJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &J)
  // {
  //   kdl_->getJacobian(0, q, J);
  // }


  void RobotInterface::getJacobian(Eigen::VectorXd &q, Eigen::MatrixXd &J, const int &ch_idx)
  {
    kdl_->getJacobian(ch_idx, q, J);
  }


  void RobotInterface::getJointMass(Eigen::VectorXd &q, Eigen::MatrixXd &Mq)
  {
    kdl_->getM(0, q, Mq);
  }


  void RobotInterface::getTaskMass(Eigen::MatrixXd &Mq, Eigen::MatrixXd &J, Eigen::MatrixXd &Mx)
  {
    getPseudoInverse(J * Mq.inverse() * J.transpose(), Mx, 1.e-5);
  }


  void RobotInterface::getTaskError(Eigen::VectorXd &q, Eigen::VectorXd &q_dot,
                                    Eigen::Affine3d &x_des, Eigen::VectorXd &x_dot_des,
                                    Eigen::VectorXd &x_err, Eigen::VectorXd &x_dot_err)
  {
    kdl_->getTaskError(0, q, q_dot, x_des, x_dot_des, x_err, x_dot_err);
  }


  void RobotInterface::initJointState(sensor_msgs::JointState &jnt_state)
  {
    jnt_state = sensor_msgs::JointState();
    for (int i = 0; i < num_jnts_; ++i)
    {
      jnt_state.name.push_back(jnt_names_[i]);
      jnt_state.position.push_back(0.0);
      jnt_state.velocity.push_back(0.0);
      jnt_state.effort.push_back(0.0);
    }
  }


  void RobotInterface::log(std::string msg)
  {
    log(msg, INFO);
  }


  void RobotInterface::log(std::string msg, LogLevel level)
  {
    switch(level)
    {
      case WARN :
      {
        ROS_WARN_STREAM("[RobotInterface] " << msg);
        break;
      }
      case ERROR :
      {
        ROS_ERROR_STREAM("[RobotInterface] " << msg);
        break;    
      }
      default:
      {
        ROS_INFO_STREAM("[RobotInterface] " << msg);
        break;    
      }
    }
  }

  
  void RobotInterface::publishRealRobot(const Eigen::VectorXd &cmd, const Eigen::VectorXd &dot_cmd)
  {
    ROS_ERROR_STREAM("Function not defined");
  }

  
  void RobotInterface::publishRealRobot(const Eigen::VectorXd &cmd, ControlMode &cm)
  {
    ROS_ERROR_STREAM("Function not defined");
  }

}
