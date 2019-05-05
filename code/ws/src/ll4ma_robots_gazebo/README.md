# LL4MA Robots Gazebo
This package allows for simulation of robots used by the [Utah Learning Laboratory for Manipulation Autonomy (LL4MA Lab)](https://robot-learning.cs.utah.edu) at the University of Utah. We use the [Gazebo simulator](http://gazebosim.org/) with [DART](https://dartsim.github.io/) as the physics engine. 

***

## Installation
Clone this repository into the `src` folder of a catkin workspace as usual

    git clone https://bitbucket.org/robot-learning/ll4ma_robots_gazebo # https
    git clone git@bitbucket.org:robot-learning/ll4ma_robots_gazebo.git # SSH

We have a few internal dependencies you will need in order to use this package. We have created an installation script that utilizes [wstool](http://wiki.ros.org/wstool) (`sudo apt install python-wstool` if you don't have it). Simply navigate to the root of this repository and execute

    ./install_dependencies.sh

More detailed documentation on the dependencies and how the install script can be configured is available on [our wiki](https://robot-learning.cs.utah.edu/ll4ma_robots_gazebo#package_installation).

## Usage
Each robot has a single launch file that serves as the user interface to selecting different options for simulating that robot. For example, the KUKA LBR4 can be launched with

    roslaunch ll4ma_robots_gazebo lbr4.launch

This will launch the LBR4 arm model in simulation with controllers enabled but with no end-effector attached to it. If for some reason you don't want the controllers running, use

    roslaunch ll4ma_robots_gazebo lbr4.launch control:=false

To load the LBR4 for control with the Allegro hand attached, use

    roslaunch ll4ma_robots_gazebo lbr4.launch end_effector:=allegro

To test that the controllers are working properly, you can run `joint_tester.py`, which builds a simple joint trajectory and executes it on the robot:

    rosrun ll4ma_robots_gazebo joint_tester.py
    
For more detailed usage instructions, including a comprehensive description of launch file parameters, see [our wiki page](https://robot-learning.cs.utah.edu/ll4ma_robots_gazebo).


## Topics
| Topic                        | Description                                                                                                              | Type                     |
|------------------------------|--------------------------------------------------------------------------------------------------------------------------|--------------------------|
| `/lbr4/joint_cmd`            | Control commands sent to this topic are split by joint state splitter and sent to ROS controller for each LBR4 joint.    | `sensor_msgs/JointState` |
| `/lbr4/joint_des_cmd`        | Send desired control commands (e.g. computed joint torques) to this topic for LBR4.                                      | `sensor_msgs/JointState` |
| `/lbr4/joint_states`         | LBR4 joint states are published to this topic.                                                                           | `sensor_msgs/JointState` |
| `/allegro/joint_cmd`         | Control commands sent to this topic are split by joint state splitter and sent to ROS controller for each Allegro joint. | `sensor_msgs/JointState` |
| `/allegro/joint_des_cmd`     | Send desired control commands (e.g. computed joint torques) to this topic for Allegro.                                   | `sensor_msgs/JointState` |
| `/allegro/joint_states`      | Allegro joint states are published to this topic.                                                                        | `sensor_msgs/JointState` |
| `/lbr4_allegro/joint_states` | Combined joint states for LBR4 and Allegro.                                                                              | `sensor_msgs/JointState` |


## Acknowledgement
The URDF for the KUKA LBR4 was modified by Antoine Hoarau <hoarau.robotics@gmail.com> to work with Gazebo and DART.

 
