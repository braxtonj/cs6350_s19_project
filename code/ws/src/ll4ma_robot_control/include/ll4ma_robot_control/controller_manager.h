/**
  * @file   controller_manager.h
  * @author Adam Conkey
  * @date   Sat Mar 24 13:40:20 2018
  * 
  * @brief  Controller manager to coordinate use of controllers at runtime.
  */
#ifndef LL4MA_CONTROLLER_MANAGER_H
#define LL4MA_CONTROLLER_MANAGER_H

#include <ros/ros.h>
#include <boost/assign.hpp>
#include <boost/shared_ptr.hpp>
#include <ll4ma_robot_control/SwitchControl.h>
#include <ll4ma_robot_interface/robot_interface.h>
#include <ll4ma_robot_control/control.h>

/**
 * @brief Manages coordination of multiple controller instances at runtime.
 * 
 * This class manages multiple controller instantiations, including initializing desired controllers,
 * starting and stopping controller execution, and switching controllers at runtime.
 */
class ControllerManager
{
private:

  /**
   * @brief Types of controllers offered by this controller framework.
   */
  enum ControlType
  {
    JOINT_PD, TASK_INV_DYN
  };

  /**
   * @brief Enumeration for logging levels (correspond to ROS logging levels).
   */
  enum LogLevel
  {
    INFO, WARN, ERROR
  };

  /**
   * @brief Map from control types as string to enums.
   * 
   * Map string names for control types to be mapped to enums. This is for reading control types
   * off of the ROS parameter server or from ROS messages/services, and being able to switch on
   * control type in a structured manner.
   */
  std::map<std::string, ControlType> type_map_ = boost::assign::map_list_of
                                                 ("joint_pd",     JOINT_PD)
                                                 ("task_inv_dyn", TASK_INV_DYN);

  robot_interface::RobotInterface *robot_interface_; /**< @brief Pointer to robot interface. */
  boost::shared_ptr<Controller> controller_;         /**< @brief Currently active controller. */
  boost::shared_ptr<Controller> desired_controller_; /**< @brief Controller being switched to. */
  ControlType current_control_type_; /**< @brief Type of currently active controller. */
  bool do_update_; /**< @brief Set false when controller is being switched, set true when it 
                               is safe to do a control loop execution.*/

  // ROS
  ros::NodeHandle nh_;		          /**< @brief Interface to ROS ecosystem. */
  ros::ServiceServer switch_control_srv_; /**< @brief Advertised service for switching controllers. */
  ros::Rate rate_;		          /**< @brief Rate to operate controllers at. */
  double rate_val_;                       /**< @brief Value of ROS rate (for instantiating new
                                                      controllers). */


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
   * @brief Switches controllers at runtime.
   * 
   * This function is offered as a ROS service and switches the current active controller to a 
   * different desired controller. Note that currently this is only intended for switching from
   * joint space control to task control, in order to put the robot in a joint configuration
   * more suitable for task space control (away from singularities). This will be expanded to
   * arbitrarily switch between controllers.
   * 
   * @param req ROS service request that includes name of controller to be switched to.
   * @param resp ROS service response indicating outcome of controller switch.
   * 
   * @return true if operation was successful, false otherwise.
   */
  bool switchController(ll4ma_robot_control::SwitchControl::Request &req,
                        ll4ma_robot_control::SwitchControl::Response &resp);

  /**
   * @brief Resets the controller pointer with a new desired controller.
   * 
   * @param control_type Type of controller to use for execution.
   * 
   * @return true if reset was successful, false otherwise.
   */
  bool resetControllerType(ControlType control_type);
  
public:
  /**
   * @brief Constructor taking node handle namespace and robot interface. 
   * 
   * @param ns Namespace for the ROS NodeHandle to interface into ROS ecosystem.
   * @param robot_interface Robot interface to handle robot-specific control needs.
   * @param rate Rate governing controller operation frequency.
   */
  ControllerManager(std::string ns, robot_interface::RobotInterface *robot_interface, double rate)
      : nh_(ns), robot_interface_(robot_interface), rate_val_(rate), rate_(rate) {}

  /**
   * @brief Initializes controllers associated with the controller manager.
   * 
   * Initializes controller to start execution with based on specified control type. Note that 
   * control type must be specified as "control_type" in the namespace of the node handle.
   * 
   * @return true if initialization was successful, false otherwise
   * 
   * @todo Can loosen the requirement to have control type specified on parameter server and create
   * a constructor that accepts control type as argument. Then only retrieve param from server if it
   * is verified to be loaded on server. Will need appropriate error checking.
   */    
  bool init();

  /**
   * @brief Starts the controller control loop.
   * 
   * Performs a switch based on controller type and runs the startHook of the associated controller.
   */
  void run();
};

#endif
