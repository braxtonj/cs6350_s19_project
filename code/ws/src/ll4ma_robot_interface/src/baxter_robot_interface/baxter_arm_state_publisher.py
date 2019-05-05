#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState


class BaxterArmStatePublisher:

    def __init__(self):
        rospy.loginfo("[BaxterArmStatePublisher] Initializing...")
        self.robot_name = rospy.get_param("/robot_name", "baxter")
        self.arm = rospy.get_param("/arm")
        self.rate = rospy.Rate(rospy.get_param("/robot_state_publisher/publish_frequency", 1000))
        self.joint_state_pub = rospy.Publisher("/%s/%s_arm/joint_states"
                                               % (self.robot_name, self.arm),
                                               JointState, queue_size=1)

        # TODO hacking hardcoding
        rospy.Subscriber("/robot/joint_states", JointState, self.joint_state_cb)
        # TODO can get this through baxter_interface
        self.joint_names = ["%s_s0" % self.arm, "%s_s1" % self.arm, "%s_e0" % self.arm,
                            "%s_e1" % self.arm, "%s_w0" % self.arm, "%s_w1" % self.arm,
                            "%s_w2" % self.arm]
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state.position = [0.0] * len(self.joint_names)
        self.joint_state.velocity = [0.0] * len(self.joint_names)
        self.joint_state.effort = [0.0] * len(self.joint_names)
        rospy.loginfo("[BaxterArmStatePublisher] Initialization complete.")

    def joint_state_cb(self, baxter_joint_state):
        position_dict = dict(zip(baxter_joint_state.name, baxter_joint_state.position))
        velocity_dict = dict(zip(baxter_joint_state.name, baxter_joint_state.velocity))
        effort_dict = dict(zip(baxter_joint_state.name, baxter_joint_state.effort))
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in baxter_joint_state.name:
                self.joint_state.position[i] = position_dict[joint_name]
                self.joint_state.velocity[i] = velocity_dict[joint_name]
                self.joint_state.effort[i] = effort_dict[joint_name]
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state)

    def run(self):
        rospy.loginfo("[BaxterArmStatePublisher] Relaying joint states if available...")
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.loginfo("[BaxterArmStatePublisher] Complete.")

            
if __name__ == '__main__':
    rospy.init_node('baxter_arm_state_publisher')
    basp = BaxterArmStatePublisher()
    try:
        basp.run()
    except (ROSException):
        pass
