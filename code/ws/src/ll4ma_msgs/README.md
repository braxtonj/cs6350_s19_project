# LL4MA Messages

This repo stores .msg, .srv, and .action files that are used between different LL4MA packages. This is intended to simplify package dependencies by not requiring package A to have package B as a dependency just to have access to package B's message files. The repo stores multiple packages that are divided by domain of use.

## Packages

### ll4ma_trajectory_msgs
ROS resources related to robot trajectories that are not well-represented by the standard ROS packages. For example, ROS has an existing representation for [joint space trajectories](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html), but lacks an analogous task space version (they say [this](http://docs.ros.org/jade/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html) is the analog, but its name is bound to cause confusion). 

### ll4ma_robot_control_msgs
ROS resources related to robot control. For example, a robot state message that includes both joint and task space components that can be advertised to nodes from a number of different packages.
