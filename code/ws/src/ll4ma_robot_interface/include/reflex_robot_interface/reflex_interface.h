#ifndef LL4MA_REFLEX_INTERFACE
#define LL4MA_REFLEX_INTERFACE

#include <ll4ma_robot_interface/robot_interface.h>


namespace robot_interface
{
  class ReflexInterface : public RobotInterface
  {
  protected:

    // Eigen
    Eigen::VectorXd finger_1_q_, finger_2_q_, finger_3_q_;
    Eigen::VectorXd finger_1_tau_g_, finger_2_tau_g_, finger_3_tau_g_;

    void compensateGravity(Eigen::VectorXd &tau, Eigen::VectorXd &q);
    void log(std::string msg);
    void log(std::string msg, LogLevel level);
    
  public:
    ReflexInterface(std::string ns) : RobotInterface(ns) {}
    bool init();
    void publishTorqueCommand(Eigen::VectorXd &torques);
  };
}

#endif
