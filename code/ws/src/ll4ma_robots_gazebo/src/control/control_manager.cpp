#include <ros/ros.h>

#include <ll4ma_robot_control/controller_manager.h>

// Robot interfaces
#include <ll4ma_robot_interface/robot_interface.h>
#include <lbr4_robot_interface/lbr4_interface.h>
#include <allegro_robot_interface/allegro_interface.h>
#include <reflex_robot_interface/reflex_interface.h>

#ifdef BAXTER_DEPENDS_FOUND
#include <baxter_robot_interface/baxter_interface.h>
#endif


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_control_manager");
  // assuming namespace and robot name passed as args from launch, TODO error check
  std::string robot_type = argv[1];
  std::string robot_name = argv[2];
  double rate;
  robot_interface::RobotInterface *robot_interface;

  // Get the appropriate robot interface
  if (robot_type == "lbr4")
    robot_interface = new robot_interface::LBR4Interface(robot_name);
  #ifdef BAXTER_DEPENDS_FOUND
  else if (robot_type == "baxter")
    robot_interface = new robot_interface::BaxterInterface(robot_name);
  #endif
  else if (robot_type == "allegro")
    robot_interface = new robot_interface::AllegroInterface(robot_name);
  else if (robot_type == "reflex")
    robot_interface = new robot_interface::ReflexInterface(robot_name);
  else
  {
    ROS_ERROR_STREAM("[GazeboControllerManager] Unknown robot type: " << robot_type);
    return EXIT_FAILURE;
  }
  robot_interface->init();

  ros::NodeHandle nh;
  nh.param<double>("rate", rate, 100);

  // Setup and start the controller manager
  ControllerManager manager(robot_name, robot_interface, rate);
  if (manager.init())
  {
    manager.run();
    return EXIT_SUCCESS;
  }
  else
    return EXIT_FAILURE;
}
