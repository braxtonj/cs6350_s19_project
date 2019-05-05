#ifndef LL4MA_BAXTER_INTERFACE
#define LL4MA_BAXTER_INTERFACE

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <Eigen/Geometry>

#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SEAJointState.h>

#include <ll4ma_robot_interface/robot_interface.h>


namespace robot_interface
{
  class BaxterInterface : public RobotInterface
  {
  protected:
    baxter_core_msgs::JointCommand jnt_cmd_;
    std_msgs::Empty empty_msg_;
    ros::Subscriber sea_jnt_state_sub_;
    ros::Publisher disable_baxter_gravity_pub_;
    Eigen::VectorXd tau_spring_, tau_cross_, gravity_only_, gravity_model_effort_;
    bool sea_jnt_state_received_;
    std::string sea_jnt_state_topic_;
    std::string disable_gravity_topic_;
    double spring_factor_;

    void initJointCommand(baxter_core_msgs::JointCommand &jnt_cmd);
    void seaJointStateCallback(baxter_core_msgs::SEAJointState jnt_state);
    
  public:
    BaxterInterface(std::string ns) : RobotInterface(ns) {}
    bool init();
    void publishTorqueCommand(Eigen::VectorXd &torques);
    void compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &gravity);
    void getSEAJointState(Eigen::VectorXd &spring, Eigen::VectorXd &cross, 
			  Eigen::VectorXd &gravity, Eigen::VectorXd &gravity_model);

  };
}

#endif
