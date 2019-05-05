#!/usr/bin/env python

#from std_srvs.srv import Empty
import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline import srv
from grasp_pipeline.srv import *
import os

def handle_launch_urlg_gazebo(req):
    if req.launch_urlg_gazebo:
        launch_urlg_cmd = 'roslaunch urlg_robots_gazebo lbr4_allegro_control_position.launch'
        os.system(launch_urlg_cmd)
        roslog.info('urlg_robots_gazebo launched.')

    if req.launch_urlg_gazebo and req.launch_moveit:
        rospy.sleep(10)

    if req.launch_moveit:
        launch_moveit_cmd = 'roslaunch lbr4_moveit moveit_lbr4_allegro.launch'
        os.system(launch_moveit_cmd)
        roslog.info('lbr4_moveit launched.')


def launch_urlg_gazebo_server():
    rospy.init_node('launch_urlg_gazebo_moveit_server')
    launch_service = rospy.Service('launch_urlg_gazebo_moveit', LaunchUrlgGazeboMoveit, handle_launch_urlg_gazebo)
    print 'Ready to launch urlg_robots_gazebo and lbr4_moveit.'
    rospy.spin()

if __name__ == '__main__':
    launch_urlg_gazebo_server()
