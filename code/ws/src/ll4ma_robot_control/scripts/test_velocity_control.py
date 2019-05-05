#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

class TaskVelocityTester:

    def __init__(self, ns, listen=False):
        self.vel_pub = rospy.Publisher(ns+'/task_des_cmd', TwistStamped, queue_size=1)
        rospy.loginfo('Waiting to establish communication')
        rospy.sleep(2.0)

        # TODO: track task space pose using a joint state listener and FK

    def run(self):
        # Set appropriate task frame
        cmd_twist = TwistStamped()
        cmd_twist.header.frame_id = 'world'
        cmd_twist.twist.linear.x = 0.0
        cmd_twist.twist.linear.y = 0.0
        cmd_twist.twist.linear.z = -5.0
        cmd_twist.twist.angular.x = 0
        cmd_twist.twist.angular.y = 0.00
        cmd_twist.twist.angular.z = 0
        self.vel_pub.publish(cmd_twist)
        rospy.loginfo('Published command ' + str(cmd_twist))
        rospy.loginfo('Waiting')
        rospy.sleep(10.0)
        rospy.loginfo('Stopping')
        self.stop()
        rospy.loginfo('Stopped')

    def stop(self):
        cmd_stop_twist = TwistStamped()
        cmd_stop_twist.header.frame_id = 'lbr4_0_link'
        cmd_stop_twist.twist.linear.x = 0.0
        cmd_stop_twist.twist.linear.y = 0.0
        cmd_stop_twist.twist.linear.z = 0.0
        cmd_stop_twist.twist.angular.x = 0.0
        cmd_stop_twist.twist.angular.y = 0.0
        cmd_stop_twist.twist.angular.z = 0.0
        self.vel_pub.publish(cmd_stop_twist)

if __name__ == '__main__':
    rospy.init_node('velocity_tester')
    ns = 'lbr4'
    tester = TaskVelocityTester(ns)
    tester.run()
