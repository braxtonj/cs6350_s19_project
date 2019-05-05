#!/usr/bin/env python
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import JointState


_DISPLAY_RATE = 50
class JointTester:

    def __init__(self, node_name='joint_tester', listen_prefix='', listen=False,
                 publish_prefix='/lbr4'):
        rospy.init_node(node_name)

        self.hand_name = rospy.get_param("hand_name", "NO_HAND")
        if self.hand_name in ["allegro", "allegro_hand_right"]:
            allegro_jnt_cmd_topic = rospy.get_param("/{}/jnt_des_topic".format(self.hand_name))
            self.allegro_jnt_cmd_pub = rospy.Publisher(allegro_jnt_cmd_topic,
                                                       JointState, queue_size=5)        
        
        self.joint_cmd_pub = rospy.Publisher(publish_prefix+'/joint_cmd', JointState, queue_size=5)
        self.ctrl_type_pub = rospy.Publisher(publish_prefix+'/control_type', Char, queue_size=5)

        if listen:
            rospy.Subscriber(listen_prefix+'/joint_states', JointState, self.joint_state_cb)

        self.reflex_jnt_cmd_pub = rospy.Publisher('/reflex/joint_des_cmd', JointState, queue_size=5)

        self.run_rate=rospy.Rate(100)
        
    def send_joint_command(self,pos):
        jc = JointState()
        jc.name = ['lbr4_j0','lbr4_j1','lbr4_j2','lbr4_j3','lbr4_j4','lbr4_j5','lbr4_j6']
        jc.velocity = [0.0]*7
        # TODO: set this to work for velocity and effort

        jc.position = pos#[0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in xrange(100):
            self.joint_cmd_pub.publish(jc)
            self.run_rate.sleep()
        pass

    def joint_state_cb(self, cur_joint_state):
        if cur_joint_state.header.seq % _DISPLAY_RATE == 0:
            print cur_joint_state.name

    def run_lbr4_test(self, control_type='p'):
        run_rate = rospy.Rate(50)
        #self.ctrl_type_pub.publish(Char(data=ord(control_type)))
        run_rate.sleep()

        jc = JointState()
        jc.name = ['lbr4_j0','lbr4_j1','lbr4_j2','lbr4_j3','lbr4_j4','lbr4_j5','lbr4_j6']
        # TODO: set this to work for velocity and effort

        jc.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # these are added so it doesn't throw off Orocos controller when testing in sim
        jc.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        jc.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in xrange(300):
            self.joint_cmd_pub.publish(jc)
            for j in xrange(len(jc.name)):
                jc.position[j]+=0.005
            run_rate.sleep()
        rospy.sleep(2.0)
        for i in xrange(300):
            self.joint_cmd_pub.publish(jc)
            for j in xrange(len(jc.name)):
                jc.position[j]-=0.005
            run_rate.sleep()

    def run_allegro_test(self, control_type='p'):
        run_rate = rospy.Rate(100)
        #self.allegro_ctrl_type_pub.publish(Char(data=ord(control_type)))
        run_rate.sleep()
        print 'joint control type published'
        jc = JointState()
        jc.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']

        if(control_type=='p'):
            # TODO: set this to work for velocity
            jc.position = [ 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0]

            delta_position = 0.015

            for i in xrange(100):
                self.allegro_jnt_cmd_pub.publish(jc)

                for j in xrange(len(jc.name)):
                    jc.position[j] += delta_position
                run_rate.sleep()

            for i in xrange(100):
                self.allegro_jnt_cmd_pub.publish(jc)
                for j in xrange(len(jc.name)):
                    jc.position[j] -= delta_position
                run_rate.sleep()

    def run_reflex_test(self, control_type='p'):
        run_rate = rospy.Rate(100)
        #self.allegro_ctrl_type_pub.publish(Char(data=ord(control_type)))
        run_rate.sleep()
        jc = JointState()
        jc.name = ['preshape_1', 'preshape_2',
                   'proximal_joint_1', 'proximal_joint_2', 'proximal_joint_3']

        if(control_type=='p'):
            # TODO: set this to work for velocity
            jc.position = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]

            delta_position = 0.01

            for i in xrange(120):
                self.reflex_jnt_cmd_pub.publish(jc)
                for j in xrange(len(jc.name)):
                    if j > 1:
                        jc.position[j] += delta_position
                run_rate.sleep()

            for i in xrange(120):
                self.reflex_jnt_cmd_pub.publish(jc)
                for j in xrange(len(jc.name)):
                    if j <= 1:
                        jc.position[j] += delta_position
                run_rate.sleep()

            rospy.sleep(1.0)
                
            for i in xrange(120):
                self.reflex_jnt_cmd_pub.publish(jc)
                for j in xrange(len(jc.name)):
                    if j <= 1:
                        jc.position[j] -= delta_position
                run_rate.sleep()

            for i in xrange(120):
                self.reflex_jnt_cmd_pub.publish(jc)
                for j in xrange(len(jc.name)):
                    if j > 1:
                        jc.position[j] -= delta_position
                run_rate.sleep()
                
def test_lbr4_position_control(listen=False, publish_prefix='/lbr4', listen_prefix='/lbr4'):
    jt = JointTester(listen=listen, listen_prefix=listen_prefix, publish_prefix=publish_prefix)
    return jt
    #jt.run_lbr4_test('p')


def test_allegro_position_control(listen=False, publish_prefix='/allegro', listen_prefix=''):

    jt = JointTester(listen=listen, listen_prefix=listen_prefix, publish_prefix=publish_prefix)
    rospy.loginfo('Running allegro control test')
    jt.run_allegro_test('p')

    
def test_allegro_effort_control(listen=False, publish_prefix='', listen_prefix=''):
    jt = JointTester(listen=listen, listen_prefix='', publish_prefix=publish_prefix)
    jt.run_allegro_test('e')

    
def test_lbr4_go_home():
    jt = JointTester(publish_prefix='/lbr4')
    jc = JointState()
    jc.name = ['lbr4_j0','lbr4_j1','lbr4_j2','lbr4_j3','lbr4_j4','lbr4_j5','lbr4_j6']
    # TODO: set this to work for velocity and effort
    jc.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    jt.joint_cmd_pub.publish(jc)


    
if __name__=='__main__':
    joint_tester = test_lbr4_position_control()
    joint_tester.run_lbr4_test()
    if joint_tester.hand_name in ["allegro", "allegro_hand_right"]:
        rospy.sleep(1.0)
        joint_tester.run_allegro_test()
    if joint_tester.hand_name == "reflex":
        rospy.sleep(1.0)
        joint_tester.run_reflex_test()
