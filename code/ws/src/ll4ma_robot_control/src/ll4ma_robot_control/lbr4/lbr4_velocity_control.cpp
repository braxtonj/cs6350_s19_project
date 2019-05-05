#include "ll4ma_robot_control/task_velocity_control.h"
#include <lbr4_robot_interface/lbr4_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lbr4_task_velocity_controller");
  std::string ns = "lbr4";
  robot_interface::RobotInterface *robot_interface;
  robot_interface = new robot_interface::LBR4Interface(ns);
  robot_interface->init();

  // Make node
  TaskVelocityController controller(ns, robot_interface, 500);
  // Run node
  controller.configureHook();
  controller.startHook();
  controller.cleanupHook();
  return 0;
}
