#include "lbr4_robot_interface/lbr4_interface.h"


// TODO right now there is nothing special about the LBR4 interface over the base robot interface,
// so this is not currently adding functionality. FRI on real robot is likely to add some complexity
// so still letting this be its own class.

namespace robot_interface
{

  bool LBR4Interface::init()
  {
    log("Initializing...");

    if (!RobotInterface::init())
      return false;

    log("Initialization complete.");
    return true;
  }


  void LBR4Interface::log(std::string msg)
  {
    log(msg, INFO);
  }
  
  
  void LBR4Interface::log(std::string msg, LogLevel level)
  {
    switch(level)
    {
      case WARN :
      {
        ROS_WARN_STREAM("[LBR4Interface] " << msg);
        break;
      }
      case ERROR :
      {
        ROS_ERROR_STREAM("[LBR4Interface] " << msg);
        break;    
      }
      default:
      {
        ROS_INFO_STREAM("[LBR4Interface] " << msg);
        break;    
      }
    }
  }
  
} // namespace robot_interface
