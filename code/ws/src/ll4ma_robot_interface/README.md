# LL4MA Robot Interface

This is a C++ library that provides an abstraction layer for robot communication. Its primary use is for robot control and works in conjunction with [LL4MA Robot Control](https://bitbucket.org/robot-learning/ll4ma_robot_control) package. By implementing robot-specific interfaces that inherit from a common interface, the robot controllers can be implemented such that they are agnostic to the particular robot being controlled, allowing for general-use control laws to be applied on a wide variety of different robot platforms. The main motivation was to avoid implementing separate controllers for every robot we use in our lab just because they have different communication requirements (e.g. different message types for sending commands, different gravity compensation interfaces, differences in real and simulated robots, etc.).

## Dependencies

This package requires our lab's [LL4MA KDL](https://bitbucket.org/robot-learning/ll4ma_kdl) package which provides convenience functions for computing kinematic and dynamics components needed for robot control. Also required is our [ll4ma_msgs](https://bitbucket.org/robot-learning/ll4ma_msgs) package which has message definitions for simplifying dependencies over multiple packages.

## Current Interfaces

We have implemented a number of interfaces for the robots we use in our lab. So far we have

* KUKA LBR4+ (simulation)
* Baxter (real and simulation)
* Allegro hand (real and simulation)
* ReFlex hand (simulation)

The hope is to have real and simulation interfaces for all of our robots - these will be implemented when they are needed. If you have a need for one of these interfaces or you would like to implement an interface for one of your own robots, please reach out to Adam Conkey (adam.conkey@utah.edu), he will be able to instruct you on how to move forward.


