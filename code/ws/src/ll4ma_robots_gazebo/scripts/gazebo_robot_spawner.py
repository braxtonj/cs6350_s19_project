#!/usr/bin/env python
"""
Script to spawn a robot in Gazebo with a desired joint configuration. 
This is a work-around for this issue:

    https://github.com/ros-simulation/gazebo_ros_pkgs/issues/93

"""
import sys
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import (
    SetModelConfiguration,
    SetModelConfigurationRequest,
    SpawnModel,
    SpawnModelRequest
)


rospy.init_node('gazebo_robot_spawner')

# Pause the simulation
try:
    pause_sim = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    pause_sim()
    rospy.loginfo("Simulation paused.")
except rospy.ServiceException, e:
    rospy.logerr("Could not pause simulation physics.")
    sys.exit()

# Spawn the robot model with desired joint configuration
spawn_config = SpawnModelRequest()
spawn_config.model_name = "lbr4"
spawn_config.model_xml = rospy.get_param("robot_description")
spawn_config.robot_namespace = "lbr4"
spawn_config.initial_pose.orientation.w = 1

try:
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    spawn_model(spawn_config)
    rospy.loginfo("Robot model spawned.")
except rospy.ServiceException, e:
    rospy.logerr("Could not spawn robot model.")
    sys.exit()

# Set desired joint configuration
model_config = SetModelConfigurationRequest()
model_config.model_name = "lbr4"
model_config.urdf_param_name = "robot_description"
model_config.joint_names = ["lbr4_j0"]
model_config.joint_positions = [1.0]

try:
    set_joints = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
    resp = set_joints(model_config)
    if resp.success:
        rospy.loginfo("Joint configuration set.")
    else:
        rospy.logerr("Could not set joint configuration: %s" % resp.message)
        sys.exit()
    
except rospy.ServiceException, e:
    rospy.logerr("Could not set joint configuration.")
    sys.exit()


    

#Unpause the simulation
try:
    unpause_sim = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    unpause_sim()
    rospy.loginfo("Simulation unpaused.")
except rospy.ServiceException, e:
    rospy.logerr("Could not unpause simulation physics.")
    sys.exit()



    
