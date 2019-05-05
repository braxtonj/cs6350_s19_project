#ifndef ALLEGRO_INTERFACE_H
#define ALLEGRO_INTERFACE_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <ll4ma_robot_interface/robot_interface.h>


namespace robot_interface
{
  class AllegroInterface : public RobotInterface
  {
  protected:

    // Eigen
    Eigen::VectorXd finger_q_, finger_tau_g_;
    
    void log(std::string msg);
    void log(std::string msg, LogLevel level);

  public:
    AllegroInterface(std::string ns) : RobotInterface(ns) {}
    bool init();
    void compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &gravity);
    
    /**
     * @brief Publishes joint state command to allegro_driver which has its own low level controller.
     * 
     * The allegro_driver package for the real robot takes as input a joint state command 
     * and uses PD if position is present in the joint state and if position is empty,
     * effort is directly sent in open loop.
     * @todo: Need to verify velocity controller
     * @param cmd the joint command values
     * @param cm enum to switch between [position,velocity, effort]
     */
    void publishRealRobot(const Eigen::VectorXd &cmd, ControlMode &cm) override;

    /** 
     * This function publishes joint position and velocity, the allegro_driver switches to traj_PD mode and expects a smooth trajectory to follow
     * 
     * @param q_cmd 
     * @param q_dot_cmd 
     */
    void publishRealRobot(const Eigen::VectorXd &q_cmd, const Eigen::VectorXd &q_dot_cmd);

  };
}

#endif
