#ifndef LL4MA_LBR4_INTERFACE
#define LL4MA_LBR4_INTERFACE

#include <ll4ma_robot_interface/robot_interface.h>

// TODO not currently being used as the base RobotInterface suffices for LBR4 right now. Will maybe
// need specialization on real robot though, so leaving this class for now.


namespace robot_interface
{
  class LBR4Interface : public RobotInterface
  {
  protected:

    void log(std::string msg);
    void log(std::string msg, LogLevel level);
    
  public:
    LBR4Interface(std::string ns) : RobotInterface(ns) {}
    bool init();
  };
}

#endif
