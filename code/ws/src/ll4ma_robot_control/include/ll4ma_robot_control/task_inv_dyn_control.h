/**
 * @file   task_inv_dyn_control.h
 * @author Adam Conkey
 * @date   Sat Mar 24 15:08:30 2018
 * 
 * @brief  Task space inverse dynamics controller.
 */
#ifndef TASK_INV_DYN_CONTROLLER_H
#define TASK_INV_DYN_CONTROLLER_H

#include "control.h"
#include <ros/ros.h>
#include <ll4ma_trajectory_msgs/TaskTrajectoryPoint.h>


/**
 * @brief Task space inverse dynamics controller.
 * 
 * Implements a task space inverse dynamics controller that tracks desired poses and task space
 * velocities. 
 */
class TaskInvDynController: public Controller
{
private:
  
  bool use_posture_;		/**< @brief Use null-space projection to maintain desired joint 
				            posture if true */
  std::string task_des_topic_;  /**< @brief Name of desired task space command topic. */
  
  // Eigen
  Eigen::Affine3d x_;		/**< @brief Actual task space pose. */
  Eigen::Affine3d x_des_;	/**< @brief Desired task space pose. */
  Eigen::VectorXd x_dot_;	/**< @brief Actual task space velocity. */
  Eigen::VectorXd x_dot_des_;	/**< @brief Desired task space velocity. */
  Eigen::VectorXd x_err_;	/**< @brief Task space pose error. */
  Eigen::VectorXd x_dot_err_;	/**< @brief Task space velocity error. */
  Eigen::VectorXd q_null_des_;	/**< @brief Desired null space joint posture. */
  Eigen::VectorXd tau_posture_;	/**< @brief Joint torques associated with null space posture. */
  Eigen::MatrixXd J_;		/**< @brief Jacobian matrix. */
  Eigen::MatrixXd Mq_;		/**< @brief Joint space mass matrix. */
  Eigen::MatrixXd Mx_;		/**< @brief Task space mass matrix (joint space mass matrix reflected
                    			    into task space). */
  Eigen::MatrixXd Kp_;		/**< @brief Task space proportional control gains. */
  Eigen::MatrixXd Kd_;		/**< @brief Task space derivative control gains. */
  Eigen::MatrixXd Kp_null_;	/**< @brief Joint space null space proportional control gains. */
  Eigen::MatrixXd Kd_null_;	/**< @brief Joint space null space derivative control gains. */
  Eigen::MatrixXd I_N_;		/**< @brief Square identity matrix (num_jnts x num_jnts). */

  // ROS
  ros::Subscriber task_des_sub_; /**< @brief Subscriber to desired task space commands. */

  /**
   * @brief Sets current robot configuration as desired.
   *
   * Sets current task space pose (as computed from forward kinematics on current joint position)
   * as the desired pose. Should be called before initiating task space control execution. 
   */
  void setCurrentConfigAsDesired();

  /**
   * @brief Callback function for desired task space command.
   * 
   * @param cmd Desired task space state.
   */
  void taskDesCallback(ll4ma_trajectory_msgs::TaskTrajectoryPoint cmd);

  /**
   * @brief ROS logger wrapper to log a message at INFO level.
   * 
   * @param msg Message to be logged.
   */
  void log(std::string msg);

  /**
   * @brief ROS logger wrapper to log a message at the desired level.
   * 
   * Wrapper to the ROS logging framework to prepend the class name. This is helpful in discriminating
   * console output when launching multiple nodes from a single launch file.
   * 
   * @param msg Message to be logged.
   * @param level Log level (INFO, WARN, ERROR).
   */  
  void log(std::string msg, LogLevel level);

  /**
   * @brief Performs a null space projection in joint space. 
   * 
   * @param tau Joint torques to incorporate null space projection torques into. 
   * @param tau_null Null space joint torques.
   * @param J Jacobian matrix.
   * @param Mx Task space mass matrix.
   * @param Mq Joint space mass matrix.
   */
  template<typename Derived1, typename Derived2, typename Derived3, typename Derived4>
  void nullSpaceProjection(Eigen::MatrixBase<Derived1> &tau,
                           const Eigen::MatrixBase<Derived1> &tau_null,
                           const Eigen::MatrixBase<Derived2> &J,
                           const Eigen::MatrixBase<Derived3> &Mx,
                           const Eigen::MatrixBase<Derived4> &Mq)
  {
    tau += (I_N_ - ((Mq.inverse() * J.transpose() * Mx) * J).transpose()) * tau_null;
  }

public:

  /**
   * @brief Constructor taking node handle namespace and robot interface.
   * 
   * @param ns Namespace for the ROS NodeHandle to interface into ROS ecosystem.
   * @param robot_interface Robot interface to handle robot-specific control needs.
   * @param rate Rate governing controller operation frequency.
   */
  TaskInvDynController(std::string ns, robot_interface::RobotInterface *robot_interface, double rate)
    : Controller(ns, robot_interface, rate) {}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};

#endif
