#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty


if __name__ == '__main__':
    rospy.init_node('disable_baxter_gravity')
    rate = rospy.Rate(10)
    gravity_pub = rospy.Publisher("/robot/limb/right/suppress_gravity_compensation", Empty, queue_size=1)
    empty = Empty()
    rospy.loginfo("[DisableGravity] Built-in gravity compensation is disabled.")
    while not rospy.is_shutdown():
        gravity_pub.publish(empty)
        rate.sleep()
    rospy.loginfo("[DisableGravity] Built-in gravity compensation is enabled.")
        
