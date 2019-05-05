# LL4MA Robot Control
This package hosts controllers for both simulated and real robots. The controllers defined here replicate the commonly used functions from the [RTT::TaskContext](http://www.orocos.org/stable/documentation/rtt/v2.x/api/html/classRTT_1_1TaskContext.html) of the [Orocos framework](http://www.orocos.org/). This is done with the prospect of easily porting the controllers to Orocos real-time controllers.

***

## Installation
Clone this repository into the `src` folder of your catkin workspace as usual

	git clone git@bitbucket.org:robot-learning/ll4ma_robot_control.git

There is one internal dependency for robot interfaces:

    git clone git@bitbucket.org:robot-learning/ll4ma_robot_interface.git
	
The robot interface package allows the controllers to be abstracted away from any robot specifics, making this controller package suitable for general use. If you do not see an appropriate interface for your robot, feel free to add another one in the [ll4ma_robot_interface](https://bitbucket.org/robot-learning/ll4ma_robot_interface) package that suits your needs.

## Available Controllers
| File                       | Description                                                                                                                                                        |
|----------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `joint_pd_control.cpp`     | Joint space PD control.                                                                                                                                            |
| `task_inv_dyn_control.cpp` | Task space inverse dynamics control with generalized pseudo-inverse (see [Khatib 1987](https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1987_RA.pdf)) |

## Adding New Controllers
See one of the existing controllers for how you should structure things. The following are required for integrating a new controller into the package if you want to use it with the controller manager:

1. Define your controller in `your_control.cpp(h)`. It should extend the Controller class `control.h` and implement the presecribed virtual functions.
2. Add a ControlType enum element for your controller in `controller_manager.h`, and add it to `type_map_`.
3. Augment all of the switch statements in `controller_manager.cpp` so you can start and switch to your controller through the manager.