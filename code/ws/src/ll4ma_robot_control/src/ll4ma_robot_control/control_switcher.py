#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ll4ma_robot_control.srv import SwitchControl, SwitchControlRequest, JointPosition


# TODO this is a quick version just to get up and running, can make this more intelligent later


class ControllerSwitcher:

    def __init__(self):
        self.current = None
        self.robot_name = rospy.get_param("robot_name")
        self.jnt_des_topic = rospy.get_param("/%s/jnt_des_topic" % self.robot_name)
        self.jnt_state_topic = rospy.get_param("/%s/jnt_state_topic" % self.robot_name)
        self.jnt_des_pub = rospy.Publisher(self.jnt_des_topic, JointState, queue_size=1)
        self.jnt_state_sub = rospy.Subscriber(self.jnt_state_topic, JointState, self._joint_state_cb)
        self.rate = rospy.Rate(100)
        
    def move_to_position(self, req):
        cmd = JointState()
        trajectory = self._interpolate(self.current, list(req.desired))
        for p in trajectory:
            cmd.position = p
            self.jnt_des_pub.publish(cmd)
            self.rate.sleep()
        while np.linalg.norm(np.array(req.desired) - np.array(self.current)) > 0.01:
            self.jnt_des_pub.publish(cmd)
            self.rate.sleep()
        self.switch_control()
        return True

    def switch_control(self):
        try:
            switch = rospy.ServiceProxy("/" + self.robot_name + "/switch_controller", SwitchControl)
            req = SwitchControlRequest()
            req.control_type = "task_inv_dyn"
            switch(req)
            rospy.loginfo("[ControllerSwitcher] Controller switched.")
        except rospy.ServiceException, e:
            rospy.logwarn("[ControllerSwitcher] Could not switch controller:")
            rospy.logwarn(e)
        
    def run(self):        
        rospy.loginfo("[ControllerSwitcher] Waiting for robot state...")
        while not rospy.is_shutdown() and self.current is None:
            self.rate.sleep()
        rospy.loginfo("[ControllerSwitcher] Robot state received.")

        self.srv = rospy.Service("/move_to_position", JointPosition, self.move_to_position)

        while not rospy.is_shutdown():
            self.rate.sleep()
                    
    def _joint_state_cb(self, joint_state_msg):
        self.current = joint_state_msg.position

    def _interpolate(self, p1, p2, n_pts=500):
        dims = []
        for a, b in zip(p1, p2):
            dims.append(np.linspace(a, b, n_pts))
        return [pt for pt in zip(*dims)]
    

if __name__ == '__main__':
    rospy.init_node('move_to_position')
    switcher = ControllerSwitcher()
    switcher.run()
