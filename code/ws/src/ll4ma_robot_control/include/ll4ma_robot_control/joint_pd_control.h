/**
 * @file   joint_pd_control.h
 * @author Adam Conkey
 * @date   Sat Mar 24 14:37:32 2018
 * 
 * @brief  Joint space PD controller.
 */
#ifndef JOINT_PD_CONTROL_H
#define JOINT_PD_CONTROL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include "control.h"


/**
 * @brief Joint space PD controller.
 *
 * Implements a joint space PD control law that tracks desired joint positions and velocities. 
 */
class JointPDController : public Controller
{
private:
  
  std::string jnt_des_topic_; /**< @brief Name of topic to receive desired joint states. */
			      
  // Eigen		      
  Eigen::VectorXd q_des_;     /**< @brief Desired joint positions. */
  Eigen::VectorXd q_dot_des_; /**< @brief Desired joint velocities. */
  Eigen::MatrixXd Kp_;	      /**< @brief Proportional controller gains (diagonal matrix). */
  Eigen::MatrixXd Kd_;	      /**< @brief Derivative controller gains (diagonal matrix). */

  // ROS
  ros::Subscriber jnt_des_sub_;	  /**< @brief ROS subscriber to desired joint state topic. */
  ros::ServiceServer reload_srv_; /**< @brief ROS service to reload control parameters from server. */

  /**
   * @brief Stores desired joint states from ROS topic.
   * 
   * Stores desired joint positions and velocities from ROS topic.  
   * 
   * @param cmd Desired joint states.
   * 
   * @todo Right now only taking in joint positions. Need to add velocities with error checking since
   * they are not always commanded.
   */
  void jointDesCallback(sensor_msgs::JointState cmd);

  /**
   * @brief Sets the current joint positions as desired joint positions.
   */
  void setCurrentConfigAsDesired();

  /**
   * @brief Reloads controller parameters from ROS parameter server.
   *  
   * Reloads the controller parameters from the ROS parameter. New values for controller parameters
   * can be loaded to the parameter server at runtime (e.g. from a YAML configuration file through 
   * a launch file), and the new controller parameters can be read in and updated.
   * 
   * @param req Empty ROS service request.
   * @param resp Empty ROS service response.
   * 
   * @return true if reload operation was successful, flase otherwise.
   */
  bool reloadControlParameters(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

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

public:

  /**
   * @brief Constructor taking node handle namespace and robot interface.
   * 
   * @param ns Namespace for the ROS NodeHandle to interface into ROS ecosystem.
   * @param robot_interface Robot interface to handle robot-specific control needs.
   * @param rate Rate governing controller operation frequency.
   */
  JointPDController(std::string ns, robot_interface::RobotInterface *robot_interface, double rate)
    : Controller(ns, robot_interface, rate) {}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();
};

#endif
