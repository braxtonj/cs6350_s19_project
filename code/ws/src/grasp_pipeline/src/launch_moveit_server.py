#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline import srv
from grasp_pipeline.srv import *
import os

def handle_launch_moveit(req):
    if req.launch_moveit:
        launch_moveit_cmd = 'roslaunch lbr4_moveit moveit_lbr4_allegro.launch'
        os.system(launch_moveit_cmd)
        roslog.info('lbr4_moveit launched.')


def launch_moveit_server():
    rospy.init_node('launch_moveit_server')
    launch_service = rospy.Service('launch_moveit', LaunchMoveit, handle_launch_moveit)
    rospy.loginfo('Ready to launch lbr4_moveit.')
    rospy.spin()

if __name__ == '__main__':
    launch_moveit_server()
