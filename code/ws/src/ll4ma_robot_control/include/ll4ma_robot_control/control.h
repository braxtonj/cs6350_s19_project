/**
 * @file   control.h
 * @author Adam Conkey
 * @date   Sat Mar 24 11:40:09 2018
 * 
 * @brief  Base controller class establishing basic controller functionality.
 */

#ifndef LL4MA_CONTROLLER_H
#define LL4MA_CONTROLLER_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <ll4ma_robot_interface/robot_interface.h>

#include <typeinfo>

/**
 * @brief Base controller class that other controllers extend.
 * 
 * This class is the base controller class which all implemented controllers should extend. 
 * It establishes basic functionality and variables that are common to all controllers running in
 * this framework. This class is modeled after <a href="http://www.orocos.org/stable/documentation/rtt/v2.x/api/html/classRTT_1_1TaskContext.html">RTT::TaskContext</a> from the Orocos real-time toolkit in 
 * order to make it so that controllers in this framework can be easily ported to Orocos real-time 
 * controllers. 
 */
class Controller
{
protected:
  /**
   * @brief Enumeration for logging levels (correspond to ROS logging levels).
   */
  enum LogLevel
  {
    INFO, WARN, ERROR
  };
  
  int num_jnts_;		/**< @brief Number of robot joints. */
  bool compensate_gravity_;	/**< @brief Gravity compensation active if true. */
  bool robot_state_received_;	/**< @brief Set true when robot state received from topic callback. */
  bool system_ok_;		/**< @brief Set false if control needs to be terminated. */
  bool initialized_;            /**< @brief Set true when controller has been initialized. */
  std::string jnt_state_topic_;	/**< @brief Name of joint state topic. */
  
  // RobotInterface
  robot_interface::RobotInterface *robot_interface_; /**< @brief Pointer to RobotInterface 
                                                                 (or subclass). */
  
  // ROS
  ros::Subscriber jnt_state_sub_; /**< @brief Subscriber to robot joint states. */
  ros::Rate rate_;		  /**< @brief Rate to manage controller operation rate. */
  ros::NodeHandle nh_;		  /**< @brief NodeHandle for interfacing with ROS ecosystem. */
  ros::Time cur_time_, prev_time_;
  ros::Duration dt_;
  
  int robot_chain_idx_; 

  // Eigen
  Eigen::VectorXd q_;	  /**< @brief Actual joint positions. */
  Eigen::VectorXd q_dot_; /**< @brief Actual joint velocities. */
  Eigen::VectorXd tau_;	  /**< @brief Commanded joint torques. */

  /**
   * @brief Retrieves control gain parameters from ROS parameter server.
   * 
   * Retrieves control gain parameters loaded to the ROS parameter server in list format (e.g from
   * a YAML configuration file) and stores them in an Eigen matrix as a diagonal matrix.
   * 
   * @param param Parameter name under which gains are stored on the ROS parameter server.
   * @param nh NodeHandle reference to ROS ecosystem.
   * @param M Matrix to store gain parameters.
   * 
   * @return true if retrieval was successful, false otherwise.
   */
  template<typename Derived>
  bool getGainsFromParamServer(std::string param, ros::NodeHandle &nh, Eigen::MatrixBase<Derived> &M)
  {
    // Currently this sets the diagonal elements to the gain values
    // Check if the vector is a 1D array and setup 
    std::vector<double> temp;
    bool success = nh.getParam(param, temp);

    if(typeid(Derived) == typeid(Eigen::Matrix<double, Eigen::Dynamic, 1>))
    {
      // store in vector
      assert(M.size() == temp.size());
      for(int i = 0; i < temp.size(); ++i)
        M(i) = temp[i]; 
    }
    else
    {
      // store in diagonal matrix
      // TODO: assert to make sure v has the same size?
      for (int i = 0; i < temp.size(); i++)
        M(i,i) = temp[i];
    }

    return success;
  }

  /**
   * @brief Robot joint state callback function.
   * 
   * Saves the current robot joint state and indicates that robot joint state was received.
   * 
   * @param state Robot joint state message to save data from.
   */
  void jointStateCallback(sensor_msgs::JointState state);

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
  Controller(std::string ns, robot_interface::RobotInterface *robot_interface, double rate, int robot_chain_idx=0)
    : robot_chain_idx_(robot_chain_idx), nh_(ns), robot_interface_(robot_interface), rate_(rate) {}

  /**
   * @brief Configuration operations performed prior to starting the controller.
   * 
   * Configuration operations to perform prior to starting the controller include loading needed
   * parameters from the ROS parameter server, initializing a robot interface, setting up ROS
   * publishers, subscribers, and services, and any other operations needed to put the controller
   * in a state ready to be initiated.
   * 
   * @return true if configuration was successful, false otherwise.
   */
  virtual bool configureHook();

  /**
   * @brief Starts the robot controller.
   * 
   * This should ensure that the controller is in a state ready to be initiated (e.g. robot state
   * is being monitored, control variables are properly initialized, etc.), and start the 
   * controller loop.
   * 
   * @return true if controller was started successfully, false otherwise.
   */
  virtual bool startHook() = 0;

  /**
   * @brief Implementation of the control loop.
   *
   * This will be called at every iteration of the control loop and should implement the core logic
   * of the controller. It will (via the robot interface) publish control commands to the robot and
   * expose the robot state to the ROS ecosystem for logging.
   */
  virtual void updateHook() = 0;

  /**
   * @brief Handles termination of the control loop. 
   */
  virtual void stopHook() = 0;

  /**
   * @brief Performs any cleanup operations needed after control has stopped.
   */
  virtual void cleanupHook() = 0;

  /**
   * @brief Checks if controller has been successfully initialized.
   * 
   * @return true if controller is successfully initialized, false otherwise.
   */
  bool isInitialized();
};    
  
#endif
