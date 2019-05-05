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
        rospy.sleep(10)

def launch_urlg_gazebo_server():
    rospy.init_node('launch_urlg_gazebo_server')
    launch_service = rospy.Service('launch_urlg_gazebo', LaunchUrlgGazebo, handle_launch_urlg_gazebo)
    rospy.loginfo('Ready to launch urlg_robots_gazebo.')
    rospy.spin()

if __name__ == '__main__':
    launch_urlg_gazebo_server()
