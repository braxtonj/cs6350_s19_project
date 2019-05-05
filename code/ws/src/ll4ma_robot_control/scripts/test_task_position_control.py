#!/usr/bin/env python
import rospy
import numpy as np
# from geometry_msgs.msg import Pose
from ll4ma_movement_primitives.util import quaternion
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ll4ma_logger_msgs.msg import RobotState
from ll4ma_robot_control_msgs.msg import CartesianCommand
from trajectory_smoothing.srv import GetSmoothTraj, GetSmoothTrajRequest
from matplotlib import pyplot as plt


# TODO write this to just run through a sequence of poses from the current pose, need to wait for
# current robot state, take that as the init, and create a CartesianCommand msg that sends them to
# the /lbr4/task_cmd topic. Will also need the service call that puts the controller in command mode.



class TaskPositionTester:

    def __init__(self, execute=False):
        self.execute = execute
        self.robot_state = None
        self.rate = rospy.Rate(100)
        self.cmd_pub = rospy.Publisher("/lbr4/task_cmd", CartesianCommand, queue_size=1)
        rospy.Subscriber("/lbr4/robot_state", RobotState, self._robot_state_cb)
        
    def run(self):
        rospy.loginfo("Waiting for robot state...")
        while not rospy.is_shutdown() and not self.robot_state:
            self.rate.sleep()
        rospy.loginfo("Robot state received!")
        traj = self._get_trajectory()
        if self.execute:
            self._execute_trajectory(traj)
        else:
            self._plot_trajectory(traj)
        
    def stop(self):
        rospy.loginfo("Exiting.")

    def _get_trajectory(self):
        traj = JointTrajectory()
        x_val = self.robot_state.end_effector_pose.position.x
        for i in range(300):
            point = JointTrajectoryPoint()
            point.positions.append(x_val)
            traj.points.append(point)
            x_val += 0.001
        # for i in range(600):
        #     point = JointTrajectoryPoint()
        #     point.positions.append(x_val)
        #     traj.points.append(point)
        #     x_val -= 0.001
        # for i in range(300):
        #     point = JointTrajectoryPoint()
        #     point.positions.append(x_val)
        #     traj.points.append(point)
        #     x_val += 0.001
        return self._get_smooth_traj(traj)

    def _plot_trajectory(self, traj):
        pos = []
        vel = []
        acc = []
        for point in traj.points:
            pos.append(point.positions[0])
            vel.append(point.velocities[0])
            acc.append(point.accelerations[0])

        plt.plot(pos, lw=4.0, label='pos')
        plt.plot(vel, lw=4.0, label='vel')
        plt.plot(acc, lw=4.0, label='acc')
        plt.legend()
        plt.show()
        
    def _execute_trajectory(self, traj):
        rospy.loginfo("Commanding trajectory...")
        cmd = CartesianCommand()
        # Making explicit because I think it wasn't copying before
        cmd.desired_pose.position.x    = self.robot_state.end_effector_pose.position.x
        cmd.desired_pose.position.y    = self.robot_state.end_effector_pose.position.y
        cmd.desired_pose.position.z    = self.robot_state.end_effector_pose.position.z
        cmd.desired_pose.orientation.x = self.robot_state.end_effector_pose.orientation.x
        cmd.desired_pose.orientation.y = self.robot_state.end_effector_pose.orientation.y
        cmd.desired_pose.orientation.z = self.robot_state.end_effector_pose.orientation.z
        cmd.desired_pose.orientation.w = self.robot_state.end_effector_pose.orientation.w
        cmd.desired_twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        cmd.desired_acceleration = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self._set_controller_mode(is_command=True)
        for point in traj.points:
            cmd.desired_pose.position.x = point.positions[0] # Should only be one value in there
            cmd.desired_twist.linear.x = point.velocities[0]
            cmd.desired_acceleration.linear.x = point.accelerations[0]
            self.cmd_pub.publish(cmd)
            self.rate.sleep()
            
        # Zero out acceleration and velocity
        cmd.desired_twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        cmd.desired_acceleration = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.cmd_pub.publish(cmd)
        rospy.sleep(3.0)
        self._set_controller_mode(is_command=False)
        print "CARTESIAN ERROR\n", self._cartesian_error(self.robot_state.end_effector_pose,
                                                         cmd.desired_pose)
        rospy.loginfo("Motion complete!")

    def _robot_state_cb(self, robot_state):
        self.robot_state = robot_state

    def _set_controller_mode(self, is_command=False):
        if is_command:
            rospy.loginfo("Setting controller to COMMAND mode...")
        else:
            rospy.loginfo("Disabling COMMAND mode...")
        success = False
        try:
            set_cmd_mode = rospy.ServiceProxy("lwr_task_control/set_command_mode", SetBool)
            resp = set_cmd_mode(is_command)
            success = resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service request to set controller mode failed: %s" %e)
            return False
        if not success:
            rospy.logerr("Could not set controller mode.")
            return False
        else:
            rospy.loginfo("Control mode successfully set.")
            return True

    def _get_smooth_traj(self, joint_trajectory):
        smooth_traj = None

        # Populate request
        req = GetSmoothTrajRequest()
        req.max_acc = np.ones(7) * 0.07
        req.max_vel = np.ones(7) * 0.4
        req.max_dev = 0.1
        req.dt = 0.01
        req.init_traj = joint_trajectory
                    
        success = False
        try:
            get_smooth = rospy.ServiceProxy("/get_smooth_trajectory", GetSmoothTraj)
            resp = get_smooth(req)
            success = resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service request to get smooth trajectory failed: %s" % e)
            return None
        if not success:
            rospy.logerr("Could not get smooth trajectory.")
            return None
        else:
            rospy.loginfo("Successfully got smooth trajectory.")
            return resp.smooth_traj

    def _cartesian_error(self, actual, desired, orientation=True, norm=True):
        p1 = actual.position
        p2 = desired.position
        p_err = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
        if orientation:
            oa = actual.orientation
            od = desired.orientation
            q1 = np.array([oa.x, oa.y, oa.z, oa.w])
            q2 = np.array([od.x, od.y, od.z, od.w])
            q_err = np.squeeze(quaternion.err(q1, q2))
            pose_error = np.hstack((p_err, q_err))
        else:
            pose_error = np.hstack((p_err, np.zeros(3)))
        if norm:
            return np.linalg.norm(pose_error)
        else:
            return pose_error

        
if __name__ == '__main__':
    rospy.init_node("task_position_tester")

    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', dest='execute', action='store_true')
    parser.set_defaults(execute=False)
    args = parser.parse_args(sys.argv[1:])
    
    tester = TaskPositionTester(args.execute)
    rospy.on_shutdown(tester.stop)
    tester.run()
