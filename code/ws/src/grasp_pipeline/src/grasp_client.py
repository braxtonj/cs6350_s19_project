#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from grasp_type_inference.srv import *
from grasp_pipeline.srv import *
from point_cloud_segmentation.srv import *
from grasp_cnn_inference.srv import *
from prob_grasp_planner.srv import *
import os
import tf
from sensor_msgs.msg import JointState
# from grasp_control.srv import *
import copy
import numpy as np
import time
import roslib.packages as rp
import sys
sys.path.append(rp.get_pkg_dir('ll4ma_planner') 
                + '/scripts')
from palm_planner import PalmPosePlanner
import h5py
import align_object_frame as align_obj
import pickle
import roslib.packages as rp
import sys
sys.path.append(rp.get_pkg_dir('hand_services') 
                + '/scripts')
from hand_client import handClient
import os.path


class ExpPreshape:
    '''
    Class for experiment grasp preshapes of one object pose.
    '''
    def __init__(self):
        self.palm_pose = None
        self.allegro_js = None
        self.is_top = None
        #grasp_type can be: 'prec', 'power', 'all', 'heuristic'
        self.grasp_type = None
        #grasp_control_type can be: 'prec', 'power'
        self.grasp_control_type = None
        self.inf_suc_prob = None
        self.inits_suc_prob_list = []
        self.max_init_idx = None

class GraspDataCollectionClient:
    def __init__(self):
        rospy.init_node('grasp_client')
        self.use_sim = rospy.get_param('~use_sim', False)
        self.use_hd = rospy.get_param('~use_hd', True)
        self.num_grasps_per_object = rospy.get_param('~num_grasps_per_object', 10)
        self.save_visual_data_pre_path = rospy.get_param('~save_visual_data_pre_path', '')
        self.smooth_plan_traj = rospy.get_param('~smooth_plan_traj', False)
        self.move_robot_home = rospy.get_param('~move_robot_home', False)
        self.mount_desired_world = None
        self.set_up_place_range()
        self.palm_desired_world = None
        self.object_world_pose = None
        #self.kinect2_sd_frame_id = 'kinect2_ir_optical_frame'
        self.kinect2_hd_frame_id = 'kinect2_rgb_optical_frame'
        self.listener = tf.TransformListener()
        self.kinect2_hd_pcd_topic = '/kinect2/hd/points'
        self.kinect2_qhd_pcd_topic = '/kinect2/qhd/points'
        self.palm_planner = None
        self.grasp_obj_frame_id = 'grasp_object'

        self.data_recording_path = rospy.get_param('~data_recording_path', '')
        self.empty_exp_folder_path = rospy.get_param('~empty_exp_folder_path', '')

        hand_joint_states_topic = '/allegro_hand_right/joint_states'
        rospy.Subscriber(hand_joint_states_topic, JointState, self.get_hand_joint_state)

        self.create_batch_grasp_id_file()
        # Number of grasps for a trial of a given object pose.
        self.grasps_num_per_object = 2 #6 #5

        #Create paths to save initialization heuristic grasp
        self.init_hand_js_path = self.data_recording_path + 'init_grasp/object_hand_js'
        self.init_palm_pose_in_obj_path = self.data_recording_path + 'init_grasp/object_palm_pose_in_obj'
        #self.init_palm_pose_in_cam_path = self.data_recording_path + 'init_grasp/object_palm_pose_in_cam'

        #Offset for robot modelling error.
        #Can't add the offset if it's too close to the object, since 
        #the planner can't find the plan if the inaccurate robot model
        #is too close to the object. This might not be the best way to solve 
        #the collision checking problem. I need to dig this deeper to figure out 
        #a better way.
        self.model_err_x_offset = 0. #0.04

        self.hand_client = handClient()
        self.get_visual_data_response = None

        self.queries_num_per_batch = 4


    def create_batch_grasp_id_file(self):
        '''
        Create the grasp queries batch id and grasp id for active learning.
        '''
        self.batch_grasp_id_file_name = self.data_recording_path + 'batch_grasp_id.h5'
        #a: Read/write if exists, create otherwise (default)
        grasp_id_file = h5py.File(self.batch_grasp_id_file_name, 'a')
        cur_batch_id_key = 'cur_batch_id'
        if cur_batch_id_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_batch_id_key, data=-1)
        cur_object_id_key = 'cur_object_id'
        if cur_object_id_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_object_id_key, data=-1)
        cur_object_name_key = 'cur_object_name'
        if cur_object_name_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_object_name_key, data='empty')
        cur_grasp_id_key = 'cur_grasp_id'
        if cur_grasp_id_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_grasp_id_key, data=-1)
        cur_grasp_type_key = 'cur_grasp_type'
        if cur_grasp_type_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_grasp_type_key, data='empty')
        grasp_id_file.close()


    def update_batch_grasp_id_db(self, object_name, grasp_type):
        # Update the object and grasp ID in h5 database
        #'r+': Read/write, file must exist
        grasp_id_file = h5py.File(self.batch_grasp_id_file_name, 'r+')
        grasp_id_file['cur_grasp_id'][()] +=1
        grasp_id_file['cur_object_name'][()] = object_name 
        grasp_id_file['cur_grasp_type'][()] = grasp_type  
        #cur_grasp_id = grasp_id_file['cur_grasp_id'][()]
        self.get_cur_grasp_id_name()
        if self.cur_grasp_id % self.grasps_num_per_object == 0:
            grasp_id_file['cur_object_id'][()] +=1
            grasp_id_file['cur_grasp_id'][()] = 0
        self.get_cur_grasp_id_name()
        self.get_cur_object_id()
        #cur_object_id = grasp_id_file['cur_object_id'][()]
        cur_queries_num = self.cur_object_id * self.grasps_num_per_object \
                            + self.cur_grasp_id + 1
        if (cur_queries_num - 1) % self.queries_num_per_batch == 0:
            grasp_id_file['cur_batch_id'][()] +=1
            grasp_id_file['cur_object_id'][()] = 0
            grasp_id_file['cur_grasp_id'][()] = 0
            self.get_cur_grasp_id_name()
            self.get_cur_object_id()
        self.get_cur_batch_id()
        print '###############self.cur_object_id:', self.cur_object_id
        print 'self.cur_grasp_id:', self.cur_grasp_id
        print 'self.cur_batch_id', self.cur_batch_id
        grasp_id_file.close()


    def update_batch_grasp_id(self): 
        # Keep the object id and grasp id consistent with grasp data recording 
        # Update the object id and grasp id before grasp data recording,
        # for 1. tactile data recording and 2. grasp inference. The tactile data
        # and grasp inference files would be over-written by the next grasp if 
        # the grasp is not recorded at data recording stage.
        self.get_cur_batch_id()
        self.get_cur_object_id()
        self.get_cur_grasp_id_name()
        cur_batch_id = self.cur_batch_id
        cur_object_id = self.cur_object_id
        cur_grasp_id = self.cur_grasp_id + 1
        if cur_grasp_id % self.grasps_num_per_object == 0:
            cur_object_id +=1
            cur_grasp_id = 0
        print '***************cur_object_id:', cur_object_id
        print 'cur_grasp_id:', cur_grasp_id
        cur_queries_num = cur_object_id * self.grasps_num_per_object + cur_grasp_id + 1
        print 'cur_queries_num:', cur_queries_num
        if (cur_queries_num - 1) % self.queries_num_per_batch == 0:
            cur_batch_id +=1
            cur_object_id = 0
            cur_grasp_id = 0
            # Create batch folder for the new batch
            print '################Create folder'
            cp_dst_path = self.data_recording_path + 'queries_batch_' + str(cur_batch_id)
            if not os.path.exists(cp_dst_path):
                os.system('cp -r ' + self.empty_exp_folder_path + ' ' + cp_dst_path)

        return cur_batch_id, cur_object_id, cur_grasp_id


    def set_object_name(self, object_name):
        self.object_name = object_name


    def get_cur_batch_id(self):
        grasp_id_file = h5py.File(self.batch_grasp_id_file_name, 'r')
        self.cur_batch_id = grasp_id_file['cur_batch_id'][()]
        grasp_id_file.close()


    def get_cur_grasp_id_name(self):
        grasp_id_file = h5py.File(self.batch_grasp_id_file_name, 'r')
        self.cur_grasp_id = grasp_id_file['cur_grasp_id'][()]
        grasp_id_file.close()


    def get_cur_object_id(self):
        grasp_id_file = h5py.File(self.batch_grasp_id_file_name, 'r')
        self.cur_object_id = grasp_id_file['cur_object_id'][()]
        grasp_id_file.close()


    def get_hand_joint_state(self, hand_js):
        self.true_hand_joint_state = hand_js 

    def set_up_place_range(self):
        #Box table position: x: -0.171 y: 0.596 z: 0.25
        self.table_x = -0.171
        self.table_y = 0.596
        self.table_len_x = .5
        self.table_len_y = .5
        place_x_half_range = (self.table_len_x - 0.1) * .5                
        place_y_half_range = (self.table_len_y - 0.1) * .5
        self.place_x_min = self.table_x - place_x_half_range
        self.place_x_max = self.table_x + place_x_half_range  
        self.place_y_min = self.table_y - place_y_half_range
        self.place_y_max = self.table_y + place_y_half_range  

    def create_moveit_scene_client(self):
        rospy.loginfo('Waiting for service create_moveit_scene.')
        rospy.wait_for_service('create_moveit_scene')
        rospy.loginfo('Calling service create_moveit_scene.')
        try:
            create_scene_proxy = rospy.ServiceProxy('create_moveit_scene', ManageMoveitScene)
            create_scene_request = ManageMoveitSceneRequest()
            create_scene_request.create_scene = True
            create_scene_request.obj_seg = self.object_segment_response.obj 
            create_scene_request.object_pose_world = self.object_world_pose
            self.create_scene_response = create_scene_proxy(create_scene_request) 
            #print self.create_scene_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service create_moveit_scene call failed: %s'%e)
        rospy.loginfo('Service create_moveit_scene is executed %s.' %str(self.create_scene_response))

    def segment_object_client(self, align_obj_frame=True):
        rospy.loginfo('Waiting for service object_segmenter.')
        rospy.wait_for_service('object_segmenter')
        rospy.loginfo('Calling service object_segmenter.')
        try:
            object_segment_proxy = rospy.ServiceProxy('object_segmenter', SegmentGraspObject)
            object_segment_request = SegmentGraspObjectRequest()
            self.object_segment_response = object_segment_proxy(object_segment_request) 
            if align_obj_frame:
                self.object_segment_response.obj = \
                        align_obj.align_object(self.object_segment_response.obj, self.listener)
        except rospy.ServiceException, e:
            rospy.loginfo('Service object_segmenter call failed: %s'%e)
        rospy.loginfo('Service object_segmenter is executed.')

    def get_visual_data_client(self):
        rospy.loginfo('Waiting for service get_visual_data.')
        rospy.wait_for_service('get_visual_data')
        rospy.loginfo('Calling service get_visual_data.')
        try:
            get_visual_data_proxy = rospy.ServiceProxy('get_visual_data', GetVisualData)
            get_visual_data_request = GetVisualDataRequest()
            self.get_visual_data_response = get_visual_data_proxy(get_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service get_visual_data call failed: %s'%e)
        rospy.loginfo('Service get_visual_data is executed.')

    def gen_grasp_preshape_client(self):
        rospy.loginfo('Waiting for service gen_grasp_preshape.')
        rospy.wait_for_service('gen_grasp_preshape')
        rospy.loginfo('Calling service gen_grasp_preshape.')
        try:
            preshape_proxy = rospy.ServiceProxy('gen_grasp_preshape', GraspPreshape)
            preshape_request = GraspPreshapeRequest()
            if not self.use_sim:
                preshape_request.obj = self.object_segment_response.obj
            else:
                preshape_request.obj = self.object_segment_blensor_response.obj
                preshape_request.sample = self.sampling_grasp

            self.preshape_response = preshape_proxy(preshape_request) 
            #print self.preshape_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service gen_grasp_preshape call failed: %s'%e)
        rospy.loginfo('Service gen_grasp_preshape is executed.')

    def update_detection_grasp_preshape_client(self, update_preshape):
        rospy.loginfo('Waiting for service update_detection_grasp_preshape.')
        rospy.wait_for_service('update_detection_grasp_preshape')
        rospy.loginfo('Calling service update_detection_grasp_preshape.')
        try:
            update_preshape_proxy = rospy.ServiceProxy('update_detection_grasp_preshape', 
                                                UpdateInfPreshape)
            update_preshape_request = UpdateInfPreshapeRequest()
            update_preshape_request.exp_palm_poses = update_preshape 
            update_preshape_response = update_preshape_proxy(update_preshape_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service update_detection_grasp_preshape call failed: %s'%e)
        rospy.loginfo('Service update_detection_grasp_preshape is executed.')

    def control_allegro_config_client(self, go_home=False, close_hand=False, exp_preshape=None):
        rospy.loginfo('Waiting for service control_allegro_config.')
        rospy.wait_for_service('control_allegro_config')
        rospy.loginfo('Calling service control_allegro_config.')
        try:
            control_proxy = rospy.ServiceProxy('control_allegro_config', AllegroConfig)
            control_request = AllegroConfigRequest()
            if go_home:
                control_request.go_home = True
            elif close_hand:
                control_request.close_hand = True
            else:
                control_request.allegro_target_joint_state = exp_preshape.allegro_js 
                #rospy.loginfo('###############')
                #rospy.loginfo(control_request.allegro_target_joint_state)
            self.control_response = control_proxy(control_request) 
            #print self.control_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service control_allegro_config call failed: %s'%e)
        rospy.loginfo('Service control_allegro_config is executed %s.'%str(self.control_response))

    def listen_mount_pose(self, palm_type, grasp_preshape_idx):
        self.mount_desired_world = None
        self.object_world_pose = None
        #palm_goal_tf_name = 'palm_goal_' + str(grasp_preshape_idx)
        palm_goal_tf_name = palm_type + '_' + str(grasp_preshape_idx)
        #print palm_goal_tf_name
        #raw_input('listen mount pose')
        rate = rospy.Rate(100.0)
        i = 0
        while (not rospy.is_shutdown()) and i < 10:
            try:
                if self.mount_desired_world is None:
                    (trans_m_to_p, rot_m_to_p) = self.listener.lookupTransform('palm_link', 'allegro_mount', rospy.Time(0))
                    mount_desired_pose = PoseStamped()
                    mount_desired_pose.header.frame_id = palm_goal_tf_name 
                    mount_desired_pose.pose.position.x = trans_m_to_p[0]
                    mount_desired_pose.pose.position.y = trans_m_to_p[1]
                    mount_desired_pose.pose.position.z = trans_m_to_p[2]
                    mount_desired_pose.pose.orientation.x = rot_m_to_p[0]
                    mount_desired_pose.pose.orientation.y = rot_m_to_p[1]
                    mount_desired_pose.pose.orientation.z = rot_m_to_p[2]
                    mount_desired_pose.pose.orientation.w = rot_m_to_p[3]
                    self.mount_desired_world = self.listener.transformPose('world', mount_desired_pose)
                    
                    #Make the pose hight above a threshold to avoid collision
                    #if self.mount_desired_world.pose.position.z < 0.05: 
                    #    rospy.loginfo('Increase height.')
                    #    #raw_input('Increase height')
                    #    self.mount_desired_world.pose.position.z = 0.05

                    #Decrease the palm pose height if too high
                    #if self.mount_desired_world.pose.position.z > 0.15: 
                    #    rospy.loginfo('Decrease height.')
                    #    #raw_input('Increase height')
                    #    self.mount_desired_world.pose.position.z = 0.15

                if self.object_world_pose is None:
                    (trans_o_to_w, rot_o_to_w) = self.listener.lookupTransform('world', 'grasp_object', rospy.Time(0))
                    object_world_pose = PoseStamped()
                    object_world_pose.header.frame_id = 'world'
                    object_world_pose.pose.position.x = trans_o_to_w[0]
                    object_world_pose.pose.position.y = trans_o_to_w[1]
                    object_world_pose.pose.position.z = trans_o_to_w[2]
                    object_world_pose.pose.orientation.x = rot_o_to_w[0]
                    object_world_pose.pose.orientation.y = rot_o_to_w[1]
                    object_world_pose.pose.orientation.z = rot_o_to_w[2]
                    object_world_pose.pose.orientation.w = rot_o_to_w[3]
                    #object_world_pose.pose.orientation = Quaternion(*rot_o_to_w)
                    self.object_world_pose = object_world_pose
 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            i += 1
            rate.sleep()

    def listen_palm_pose(self, palm_type, grasp_preshape_idx):
        self.palm_object_pose = None
        self.palm_desired_world = None
        #palm_goal_tf_name = 'palm_goal_' + str(grasp_preshape_idx)
        palm_goal_tf_name = palm_type + '_' + str(grasp_preshape_idx)

        rate = rospy.Rate(100.0)
        i = 0
        while (not rospy.is_shutdown()) and i < 10:
            try:
                if self.palm_object_pose is None:
                    (trans_o_to_w, rot_o_to_w) = self.listener.lookupTransform(self.grasp_obj_frame_id, palm_goal_tf_name, rospy.Time(0))
                    palm_object_pose = PoseStamped()
                    palm_object_pose.header.frame_id = self.grasp_obj_frame_id
                    palm_object_pose.pose.position.x = trans_o_to_w[0]
                    palm_object_pose.pose.position.y = trans_o_to_w[1]
                    palm_object_pose.pose.position.z = trans_o_to_w[2]
                    palm_object_pose.pose.orientation.x = rot_o_to_w[0]
                    palm_object_pose.pose.orientation.y = rot_o_to_w[1]
                    palm_object_pose.pose.orientation.z = rot_o_to_w[2]
                    palm_object_pose.pose.orientation.w = rot_o_to_w[3]
                
                    self.palm_object_pose = palm_object_pose

                if self.palm_desired_world is None:
                    (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform('world', palm_goal_tf_name, rospy.Time(0))
                    palm_desired_world = PoseStamped()
                    palm_desired_world.header.frame_id = 'world'
                    palm_desired_world.pose.position.x = trans_p_to_w[0]
                    palm_desired_world.pose.position.y = trans_p_to_w[1]
                    palm_desired_world.pose.position.z = trans_p_to_w[2]
                    palm_desired_world.pose.orientation.x = rot_p_to_w[0]
                    palm_desired_world.pose.orientation.y = rot_p_to_w[1]
                    palm_desired_world.pose.orientation.z = rot_p_to_w[2]
                    palm_desired_world.pose.orientation.w = rot_p_to_w[3]
    
                    self.palm_desired_world = palm_desired_world
         
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            i += 1
            rate.sleep()

    def listen_true_palm_pose(self):
        '''
        Listen to get the true palm pose in camera and world frame. 
        This is necesssary because 
        '''
        true_palm_pose_pcd = None
        true_palm_pose_world = None

        rate = rospy.Rate(100.0)
        i = 0
        while (not rospy.is_shutdown()) and i < 10:
            try:
                if true_palm_pose_world is None:
                    (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform('world', 'palm_link', rospy.Time(0))
                    true_palm_pose_world = PoseStamped()
                    true_palm_pose_world.header.frame_id = 'world'
                    true_palm_pose_world.pose.position.x = trans_p_to_w[0]
                    true_palm_pose_world.pose.position.y = trans_p_to_w[1]
                    true_palm_pose_world.pose.position.z = trans_p_to_w[2]
                    true_palm_pose_world.pose.orientation.x = rot_p_to_w[0]
                    true_palm_pose_world.pose.orientation.y = rot_p_to_w[1]
                    true_palm_pose_world.pose.orientation.z = rot_p_to_w[2]
                    true_palm_pose_world.pose.orientation.w = rot_p_to_w[3]
    
                if true_palm_pose_pcd is None:
                    (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform(self.kinect2_hd_frame_id, 'palm_link', rospy.Time(0))
                    true_palm_pose_pcd = PoseStamped()
                    true_palm_pose_pcd.header.frame_id = 'blensor_camera'
                    true_palm_pose_pcd.pose.position.x = trans_p_to_w[0]
                    true_palm_pose_pcd.pose.position.y = trans_p_to_w[1]
                    true_palm_pose_pcd.pose.position.z = trans_p_to_w[2]
                    true_palm_pose_pcd.pose.orientation.x = rot_p_to_w[0]
                    true_palm_pose_pcd.pose.orientation.y = rot_p_to_w[1]
                    true_palm_pose_pcd.pose.orientation.z = rot_p_to_w[2]
                    true_palm_pose_pcd.pose.orientation.w = rot_p_to_w[3]
    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            i += 1
            rate.sleep()

        print true_palm_pose_pcd, true_palm_pose_world 
        #raw_input('listen_palm_pose')
        return true_palm_pose_pcd, true_palm_pose_world 

    def arm_moveit_planner_client(self, go_home=False, place_goal_pose=None):
        rospy.loginfo('Waiting for service moveit_cartesian_pose_planner.')
        rospy.wait_for_service('moveit_cartesian_pose_planner')
        rospy.loginfo('Calling service moveit_cartesian_pose_planner.')
        try:
            planning_proxy = rospy.ServiceProxy('moveit_cartesian_pose_planner', PalmGoalPoseWorld)
            planning_request = PalmGoalPoseWorldRequest()
            if go_home:
                planning_request.go_home = True
            elif place_goal_pose is not None:
                planning_request.palm_goal_pose_world = place_goal_pose
            else:
                planning_request.palm_goal_pose_world = self.mount_desired_world.pose
            self.planning_response = planning_proxy(planning_request) 
            #print self.planning_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service moveit_cartesian_pose_planner call failed: %s'%e)
        rospy.loginfo('Service moveit_cartesian_pose_planner is executed %s.'%str(self.planning_response.success))
        return self.planning_response.success

    def arm_movement_client(self):
        rospy.loginfo('Waiting for service arm_movement.')
        rospy.wait_for_service('arm_movement')
        rospy.loginfo('Calling service arm_movement.')
        try:
            movement_proxy = rospy.ServiceProxy('arm_movement', MoveArm)
            movement_request = MoveArmRequest()
            self.movement_response = movement_proxy(movement_request) 
            print self.movement_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service arm_movement call failed: %s'%e)
        rospy.loginfo('Service arm_movement is executed %s.'%str(self.movement_response))

    def execute_arm_plan(self, send_cmd_manually=True):
        rospy.loginfo('Executing moveit arm plan...')
        if not self.planning_response.success:
            rospy.loginfo('Does not have a plan to execute!')
            return False
        plan_traj = self.planning_response.plan_traj

        if self.palm_planner is None:
            self.palm_planner = PalmPosePlanner(init_node=False)
        
        if self.smooth_plan_traj:
            #Smooth trajectory.
            plan_traj = self.palm_planner.robot.get_smooth_traj(plan_traj)

        if send_cmd_manually:
            #self.palm_planner.plot_traj(plan_traj)

            # send to robot:
            send_cmd = raw_input('send to robot? (y/n)')
            if(send_cmd == 'y'):
                self.palm_planner.robot.send_jtraj(plan_traj)
                #raw_input('Hit any key to keep going after the robot control is done!')
                #TO DO: make sure the robot finishing executing the trajectory before returning.
                return True
            return False
        else:
            # send to robot:
            self.palm_planner.robot.send_jtraj(plan_traj)
            return True

    def create_ll4ma_planner(self):
        #TO DO: make the ll4ma planner a ros service.
        self.palm_planner = None
        self.palm_planner = PalmPosePlanner(init_node=False, camera_topic=self.kinect2_qhd_pcd_topic)

        loop_r = rospy.Rate(100)
        #Get environment point cloud
        while(not self.palm_planner.got_cloud):
            loop_r.sleep()
        #Get tf 
        trans = self.palm_planner.get_camera_tf(self.palm_planner.cloud)
        # update collision environment:
        self.palm_planner.update_collision_env(self.palm_planner.cloud, trans)
 
    def run_ll4ma_planner(self, go_home=False):
        while(not self.palm_planner.robot.got_state or not self.palm_planner.robot.got_hand_state):
            loop_r.sleep()
        arm_init_js = self.palm_planner.robot.arm_joint_state
        hand_init_js = self.palm_planner.robot.hand_joint_state

        if go_home:
            arm_home_js = copy.deepcopy(arm_init_js)
            #home_joint_states = np.array([0.0001086467455024831, 0.17398914694786072, -0.00015721925592515618, 
            #                           -1.0467143058776855, 0.0006054198020137846, 
            #                           -0.00030679398332722485, 3.3859387258416973e-06])
            home_joint_states = np.array([0.0001086467455024831, -0.17398914694786072, 0.00015721925592515618, 
                                            1.0467143058776855, 0.0006054198020137846, 
                                            -0.00030679398332722485, 3.3859387258416973e-06])
            #arm_home_js.position = np.ones(7) * 0.2
            arm_home_js.position = home_joint_states
            robot_traj, arm_jtraj = self.palm_planner.plan_joint(arm_home_js, arm_init_js, hand_init_js, T=10)
        else:
            #Should the goal pose be palm pose or mount pose?
            #robot_traj,arm_jtraj = self.palm_planner.get_plan(self.mount_desired_world, arm_init_js, hand_init_js, T=10)
            robot_traj,arm_jtraj = self.palm_planner.get_plan(self.palm_desired_world.pose, arm_init_js, hand_init_js, T=10)

        # Get robot controller ready joint trajectory:
        smooth_traj = self.palm_planner.robot.get_smooth_traj(arm_jtraj)
        
        # visualize plan:
        self.palm_planner.robot.viz_traj(arm_jtraj)
        
        self.palm_planner.plot_traj(smooth_traj)

        # send to robot:
        send_cmd = raw_input('send to robot? (y/n)')
        if(send_cmd == 'y'):
            self.palm_planner.robot.send_jtraj(smooth_traj)

    #def grasp_client(self, grasp_type, non_thumb_speed=0.2, thumb_speed=0.25):
    ##def grasp_client(self, grasp_type, non_thumb_speed=0.1, thumb_speed=0.15):
    #    rospy.loginfo('Waiting for service grasp_control.')
    #    rospy.wait_for_service('grasp_control')
    #    rospy.loginfo('Calling service grasp_control.')
    #    try:
    #        grasp_proxy = rospy.ServiceProxy('grasp_control', PreshapeControl)
    #        grasp_request = PreshapeControlRequest()
    #        grasp_request.joint_vel_thresh = .01 #.1  
    #        grasp_request.grasp_type = grasp_type
    #        grasp_request.close_non_thumb_speed = non_thumb_speed
    #        grasp_request.close_thumb_speed = thumb_speed
    #        self.grasp_response = grasp_proxy(grasp_request) 
    #        #print self.grasp_response
    #    except rospy.ServiceException, e:
    #        rospy.loginfo('Service grasp call failed: %s'%e)
    #    rospy.loginfo('Service grasp is executed %s.'%self.grasp_response.success)

    def hand_grasp_control(self, grasp_type):
        if grasp_type == 'prec':
            joint_idx = [1, 2, 5, 6, 9, 10, 14, 15]
        elif grasp_type == 'power':
            joint_idx = [1, 2, 3, 5, 6, 7, 9, 10, 11, 14, 15]
        else:
            rospy.logerr('Wrong grasp type for grasp controller!')
            return
        self.hand_client.grasp_object(joint_idx)
        cur_js = self.hand_client.get_joint_state() 
        increase_stiffness_times = 3
        for i in xrange(increase_stiffness_times):
            _, cur_js = self.hand_client.increase_stiffness(joint_idx, cur_js)

    def clean_moveit_scene_client(self):
        rospy.loginfo('Waiting for service clean_moveit_scene.')
        rospy.wait_for_service('clean_moveit_scene')
        rospy.loginfo('Calling service clean_moveit_scene.')
        try:
            clean_scene_proxy = rospy.ServiceProxy('clean_moveit_scene', ManageMoveitScene)
            clean_scene_request = ManageMoveitSceneRequest()
            clean_scene_request.clean_scene = True
            self.clean_scene_response = clean_scene_proxy(clean_scene_request) 
            print self.clean_scene_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service clean_moveit_scene call failed: %s'%e)
        rospy.loginfo('Service clean_moveit_scene is executed %s.' %str(self.clean_scene_response))

    def lift_moveit_planner_client(self, height_to_lift=0.15):
        rospy.loginfo('Waiting for service moveit_cartesian_pose_planner to lift.')
        rospy.wait_for_service('moveit_cartesian_pose_planner')
        rospy.loginfo('Calling service moveit_cartesian_pose_planner to lift.')
        try:
            planning_proxy = rospy.ServiceProxy('moveit_cartesian_pose_planner', PalmGoalPoseWorld)
            planning_request = PalmGoalPoseWorldRequest()
            planning_request.palm_goal_pose_world = copy.deepcopy(self.mount_desired_world.pose)
            planning_request.palm_goal_pose_world.position.z += height_to_lift
            #planning_request.lift_way_points = False
            self.planning_response = planning_proxy(planning_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service moveit_cartesian_pose_planner call to lift failed: %s'%e)
        rospy.loginfo('Service moveit_cartesian_pose_planner to lift is executed %s.'%str(self.planning_response.success))

    def lift_task_vel_planner_client(self, height_to_lift=0.15):
        rospy.loginfo('Waiting for service straight_line_planner to lift.')
        rospy.wait_for_service('straight_line_planner')
        rospy.loginfo('Calling service straight_line_planner to lift.')
        try:
            planning_proxy = rospy.ServiceProxy('straight_line_planner', StraightLinePlan)
            planning_request = StraightLinePlanRequest()
            planning_request.lift_height = height_to_lift
            self.planning_response = planning_proxy(planning_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service straight_line_planner call to lift failed: %s'%e)
        rospy.loginfo('Service straight_line_planner to lift is executed %s.'
                      %str(self.planning_response.success))
        return self.planning_response.success

    def place_arm_movement_client(self):
        rospy.loginfo('Move the arm to palce the object back.')
        place_x_loc = np.random.uniform(self.place_x_min, self.place_x_max) 
        place_y_loc = np.random.uniform(self.place_y_min, self.place_y_max) 
        place_pose = copy.deepcopy(self.mount_desired_world.pose)
        place_pose.position.x = place_x_loc
        place_pose.position.y = place_y_loc
        print 'place_pose:', place_pose
        dc_client.arm_moveit_planner_client(place_goal_pose=place_pose)
        dc_client.arm_movement_client()

    def place_control_allegro_client(self):
        rospy.loginfo('Open the allegro hand to place the object.')
        self.control_allegro_config_client(go_home=True)

    def move_arm_home_client(self):
        rospy.loginfo('Move the arm to go home.')
        dc_client.arm_moveit_planner_client(go_home=True)
        dc_client.arm_movement_client()

    def move_arm_home(self):
        rospy.loginfo('Move the arm to go home.')
        dc_client.arm_moveit_planner_client(go_home=True)
        dc_client.execute_arm_plan()

    def record_grasp_visual_data_client(self, save_visual_data_request):
        rospy.loginfo('Waiting for service save_visual_data.')
        rospy.wait_for_service('save_visual_data')
        rospy.loginfo('Calling service save_visual_data.')
        try:
            save_visual_data_proxy = rospy.ServiceProxy('save_visual_data', SaveVisualData)
            self.save_visual_data_response = save_visual_data_proxy(save_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service save_visual_data call failed: %s'%e)
        rospy.loginfo('Service save_visual_data is executed %s.'
                       %self.save_visual_data_response.save_visual_data_success)

    def bag_tactile_visual_data(self, grasp_phase, operation):
        cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
        self.bag_grasp_tactile_data_client(grasp_phase, operation, cur_batch_id,
                                           cur_object_id, cur_grasp_id)
        self.bag_grasp_visual_data_client(grasp_phase, operation, cur_batch_id,
                                           cur_object_id, cur_grasp_id)

    def bag_grasp_tactile_data_client(self, grasp_phase, operation, cur_batch_id, 
                                      cur_object_id, cur_grasp_id):
        rospy.loginfo('Waiting for service bag_tactile_data.')
        rospy.wait_for_service('bag_tactile_data')
        rospy.loginfo('Calling service bag_tactile_data.')
        try:
            bag_tactile_data_request = GraspDataBaggingRequest()
            
            bag_tactile_data_request.batch_id = cur_batch_id
            bag_tactile_data_request.object_id = cur_object_id
            bag_tactile_data_request.object_name = self.object_name
            bag_tactile_data_request.grasp_id = cur_grasp_id
            bag_tactile_data_request.grasp_type = exp_preshape.grasp_type
            bag_tactile_data_request.grasp_control_type = exp_preshape.grasp_control_type

            bag_tactile_data_request.grasp_phase = grasp_phase
            bag_tactile_data_request.operation = operation

            bag_tactile_data_proxy = rospy.ServiceProxy('bag_tactile_data', GraspDataBagging)
            self.bag_tactile_data_response = bag_tactile_data_proxy(bag_tactile_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service bag_tactile_data call failed: %s'%e)
        rospy.loginfo('Service bag_tactile_data is executed %s.'
                       %self.bag_tactile_data_response.bag_data_success)

    def bag_grasp_visual_data_client(self, grasp_phase, operation, cur_batch_id,
                                     cur_object_id, cur_grasp_id):
        rospy.loginfo('Waiting for service bag_visual_data.')
        rospy.wait_for_service('bag_visual_data')
        rospy.loginfo('Calling service bag_visual_data.')
        try:
            bag_visual_data_request = GraspDataBaggingRequest()
            
            bag_tactile_data_request.batch_id = cur_batch_id
            bag_visual_data_request.object_id = cur_object_id
            bag_visual_data_request.object_name = self.object_name
            bag_visual_data_request.grasp_id = cur_grasp_id
            bag_visual_data_request.grasp_type = exp_preshape.grasp_type
            bag_visual_data_request.grasp_control_type = exp_preshape.grasp_control_type

            bag_visual_data_request.grasp_phase = grasp_phase
            bag_visual_data_request.operation = operation

            bag_visual_data_proxy = rospy.ServiceProxy('bag_visual_data', GraspDataBagging)
            self.bag_visual_data_response = bag_visual_data_proxy(bag_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service bag_visual_data call failed: %s'%e)
        rospy.loginfo('Service bag_visual_data is executed %s.'
                       %self.bag_visual_data_response.bag_data_success)


    def record_grasp_data_client(self, exp_preshape):
        keep_current_grasp = True
        rospy.loginfo('Waiting for service record_grasp_data.')
        rospy.wait_for_service('record_grasp_data')
        rospy.loginfo('Calling service record_grasp_data.')
        try:
            record_grasp_data_proxy = rospy.ServiceProxy('record_grasp_data', GraspDataRecording)
            record_grasp_data_request = GraspDataRecordingRequest()

            grasp_label = None
            while grasp_label is None:
                grasp_label_str = raw_input('### Is this grasp successful? Please input y (success), n (failure),' + \
                        ' w (success with wrong type), p (no plan) or i (ignore). \n')
                if grasp_label_str == 'i':
                    keep_current_grasp = False
                    rospy.loginfo('Ignore the current grasp.')
                    return keep_current_grasp
                #print 'grasp_label_str:', grasp_label_str
                if grasp_label_str == 'y':
                    grasp_label = 1
                elif grasp_label_str == 'n':
                    grasp_label = 0
                elif grasp_label_str == 'w':
                    grasp_label = -1
                elif grasp_label_str == 'p':
                    grasp_label = -2
                else:
                    rospy.logerr('Wrong grasp label input!')
            record_grasp_data_request.grasp_success_label = grasp_label 

            #Update the grasp id and object id if the current grasp is not ignored.
            self.update_batch_grasp_id_db(self.object_name, exp_preshape.grasp_type)

            record_grasp_data_request.batch_id = self.cur_batch_id
            record_grasp_data_request.object_id = self.cur_object_id
            record_grasp_data_request.object_name = self.object_name
            record_grasp_data_request.grasp_id = self.cur_grasp_id
            record_grasp_data_request.time_stamp = time.time()

            #Minus offset of robot modelling error for data recording.
            #exp_preshape.palm_pose.pose.position.x -= self.model_err_x_offset

            record_grasp_data_request.preshape_palm_pose = exp_preshape.palm_pose
            record_grasp_data_request.preshape_allegro_joint_state = exp_preshape.allegro_js 
            record_grasp_data_request.top_grasp = exp_preshape.is_top
            record_grasp_data_request.grasp_type = exp_preshape.grasp_type
            record_grasp_data_request.grasp_control_type = exp_preshape.grasp_control_type
            record_grasp_data_request.inf_suc_prob = exp_preshape.inf_suc_prob
            record_grasp_data_request.inits_suc_prob_list = exp_preshape.inits_suc_prob_list
            record_grasp_data_request.max_init_idx = exp_preshape.max_init_idx

            record_grasp_data_request.object_pose = self.preshape_response.object_pose
            record_grasp_data_request.preshape_palm_world_pose = self.palm_desired_world

            if grasp_label_str != 'p':
                #true palm poses and preshape hand joint states
                record_grasp_data_request.true_preshape_joint_state = self.true_preshape_hand_js
                record_grasp_data_request.true_preshape_palm_world_pose = self.true_palm_pose_world
                record_grasp_data_request.true_preshape_palm_pcd_pose = self.true_palm_pose_pcd
                
                #Closed hand joint states and palm_pose
                record_grasp_data_request.close_shape_allegro_joint_state = self.close_hand_js  
                record_grasp_data_request.close_shape_palm_pcd_pose = self.close_palm_pose_pcd 
                record_grasp_data_request.close_shape_palm_world_pose = self.close_palm_pose_world 

            self.record_grasp_data_response = record_grasp_data_proxy(record_grasp_data_request) 
            rospy.loginfo('****' + str(self.record_grasp_data_response))
        except rospy.ServiceException, e:
            rospy.loginfo('Service record_grasp_data call failed: %s'%e)
        rospy.loginfo('Service record_grasp_data is executed %s.'%self.record_grasp_data_response.save_h5_success)
    
        #self.save_grasp_visual_data()
        self.save_lift_visual_data()
        return keep_current_grasp

       
    def save_grasp_visual_data(self):
        cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
        self.save_grasp_visual_data_request = SaveVisualDataRequest()
        self.save_grasp_visual_data_request.scene_cloud = self.object_segment_response.scene_cloud 

        self.save_grasp_visual_data_request.scene_depth_img = self.object_segment_response.scene_depth_img
        self.save_grasp_visual_data_request.scene_rgb_img = self.object_segment_response.scene_rgb_img

        self.save_grasp_visual_data_request.scene_sd_depth_img = self.object_segment_response.scene_sd_depth_img
        self.save_grasp_visual_data_request.scene_sd_rgb_img = self.object_segment_response.scene_sd_rgb_img

        self.save_grasp_visual_data_request.scene_sd_cloud = self.object_segment_response.scene_sd_cloud 
        visual_data_batch_path = self.save_visual_data_pre_path + 'queries_batch_' + str(cur_batch_id) + '/'
        self.save_grasp_visual_data_request.scene_cloud_save_path = visual_data_batch_path + \
                'pcd/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.pcd' 

        self.save_grasp_visual_data_request.rgb_image_save_path = visual_data_batch_path + \
                'rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 
        self.save_grasp_visual_data_request.depth_image_save_path = visual_data_batch_path + \
                'depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 
        self.save_grasp_visual_data_request.sd_rgb_image_save_path = visual_data_batch_path + \
                'sd_rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 
        self.save_grasp_visual_data_request.sd_depth_image_save_path = visual_data_batch_path + \
                'sd_depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 

        self.save_grasp_visual_data_request.scene_sd_cloud_save_path = visual_data_batch_path + \
                'sd_pcd/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.pcd' 
        self.save_grasp_visual_data_request.sd_cloud_depth_image_save_path = visual_data_batch_path + \
                'sd_cloud_depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 
        self.save_grasp_visual_data_request.sd_cloud_rgb_image_save_path = visual_data_batch_path + \
                'sd_cloud_rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.ppm' 
        self.save_grasp_visual_data_request.sd_cloud_normal_save_path = visual_data_batch_path + \
                'sd_cloud_normal/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.pcd' 
        self.save_grasp_visual_data_request.cloud_normal_save_path = visual_data_batch_path + \
                'cloud_normal/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                '_grasp_' + str(cur_grasp_id) + '.pcd' 

        self.record_grasp_visual_data_client(self.save_grasp_visual_data_request)

    def save_lift_visual_data(self):
        if self.get_visual_data_response is not None:
            save_visual_data_request = SaveVisualDataRequest()
            save_visual_data_request.scene_cloud = self.get_visual_data_response.scene_cloud 

            save_visual_data_request.scene_depth_img = self.get_visual_data_response.scene_depth_img
            save_visual_data_request.scene_rgb_img = self.get_visual_data_response.scene_rgb_img

            save_visual_data_request.scene_sd_depth_img = self.get_visual_data_response.scene_sd_depth_img
            save_visual_data_request.scene_sd_rgb_img = self.get_visual_data_response.scene_sd_rgb_img

            save_visual_data_request.scene_sd_cloud = self.get_visual_data_response.scene_sd_cloud 
            save_visual_data_request.scene_cloud_save_path = self.save_visual_data_pre_path + \
                    'pcd/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 

            save_visual_data_request.rgb_image_save_path = self.save_visual_data_pre_path + \
                    'rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 
            save_visual_data_request.depth_image_save_path = self.save_visual_data_pre_path + \
                    'depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 
            save_visual_data_request.sd_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 
            save_visual_data_request.sd_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 

            save_visual_data_request.scene_sd_cloud_save_path = self.save_visual_data_pre_path + \
                    'sd_pcd/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 
            save_visual_data_request.sd_cloud_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 
            save_visual_data_request.sd_cloud_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.ppm' 
            save_visual_data_request.sd_cloud_normal_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_normal/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 
            save_visual_data_request.cloud_normal_save_path = self.save_visual_data_pre_path + \
                    'cloud_normal/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 

            self.record_grasp_visual_data_client(save_visual_data_request)
        else:
            self.get_visual_data_response = None


    def grasp_pgm_inf_client(self, grasp_type, grasp_preshape_idx=-1):
        self.grasp_inf_response = None
        rospy.loginfo('Waiting for service grasp_pgm_inference.')
        rospy.wait_for_service('grasp_pgm_inference')
        rospy.loginfo('Calling service grasp_pgm_inference.')
        try:
            grasp_inf_proxy = rospy.ServiceProxy('grasp_pgm_inference', GraspPgmInf)
            grasp_inf_request = GraspPgmInfRequest()
            #Use hd cloud.
            grasp_inf_request.scene_cloud = self.object_segment_response.scene_cloud 
            grasp_inf_request.object_frame_id = self.grasp_obj_frame_id 
            #Save the initialization heuristic grasp for the current object id when inferring the first grasp of the grasp trial.
            #if (self.cur_grasp_id == -1 or self.cur_grasp_id == self.grasps_num_per_object - 1) and \
            if (self.cur_grasp_id % self.grasps_num_per_object == self.grasps_num_per_object - 1) and \
                    grasp_type == 'prec':
                grasp_inf_request.init_hand_object_config.hand_joint_state = self.preshape_response.allegro_joint_state[grasp_preshape_idx] 
                grasp_inf_request.init_hand_object_config.palm_pose = self.palm_object_pose 
                pickle.dump(self.preshape_response.allegro_joint_state[grasp_preshape_idx], open(self.init_hand_js_path, 'wb'))
                pickle.dump(self.palm_object_pose, open(self.init_palm_pose_in_obj_path, 'wb'))
            #Load the initialization heuristics grasp for the current object id for 2nd and following grasps.
            else:
                grasp_inf_request.init_hand_object_config.hand_joint_state = pickle.load(open(self.init_hand_js_path, 'rb'))
                grasp_inf_request.init_hand_object_config.palm_pose = pickle.load(open(self.init_palm_pose_in_obj_path, 'rb'))

            grasp_inf_request.object_name = self.object_name
            grasp_types = ['prec', 'power', 'all']
            grasp_type_id = grasp_types.index(grasp_type)
            grasp_inf_request.grasp_id =  grasp_type_id * 10 + grasp_preshape_idx 
            #NOTICE: the object id and grasp id for inference are not consistent with the object id for data recording.
            #The object id and grasp id are used for inference before it's updated. I don't have a good idea to fix it 
            #right now, since we don't know the object id will be updated or not when doing inference. But 
            #the object id and grasp id can be easily mapped one to one with the data recording object and grasp id. 
            #grasp_inf_request.object_id = self.cur_object_id
            #grasp_inf_request.grasp_type = grasp_type
            
            cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
            grasp_inf_request.object_id = self.cur_object_id
            grasp_inf_request.grasp_type = grasp_type

            grasp_inf_response = grasp_inf_proxy(grasp_inf_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_pgm_inference failed: %s'%e)
        rospy.loginfo('Service grasp_pgm_inference is executed %s.'%str(grasp_inf_response.success))

        return grasp_inf_response


    def grasp_pgm_al_client(self, grasp_type, grasp_preshape_idx=-1):
        self.grasp_inf_response = None
        rospy.loginfo('Waiting for service grasp_active_learn.')
        rospy.wait_for_service('grasp_active_learn')
        rospy.loginfo('Calling service grasp_active_learn.')
        try:
            grasp_inf_proxy = rospy.ServiceProxy('grasp_active_learn', GraspActiveLearn)
            grasp_inf_request = GraspActiveLearnRequest()
            grasp_inf_request.grasp_planner_name = 'grasp_type_pgm'
            #Use hd cloud.
            grasp_inf_request.scene_cloud = self.object_segment_response.scene_cloud 
            grasp_inf_request.object_frame_id = self.grasp_obj_frame_id 
            ##Save the initialization heuristic grasp for the current object id when inferring the first grasp of the grasp trial.
            #if (self.cur_grasp_id % self.grasps_num_per_object == self.grasps_num_per_object - 1) and \
            #        grasp_type == 'prec':
            #    grasp_inf_request.init_hand_object_config.hand_joint_state = self.preshape_response.allegro_joint_state[grasp_preshape_idx] 
            #    grasp_inf_request.init_hand_object_config.palm_pose = self.palm_object_pose 
            #    pickle.dump(self.preshape_response.allegro_joint_state[grasp_preshape_idx], open(self.init_hand_js_path, 'wb'))
            #    pickle.dump(self.palm_object_pose, open(self.init_palm_pose_in_obj_path, 'wb'))
            ##Load the initialization heuristics grasp for the current object id for 2nd and following grasps.
            #else:
            #    grasp_inf_request.init_hand_object_config.hand_joint_state = pickle.load(open(self.init_hand_js_path, 'rb'))
            #    grasp_inf_request.init_hand_object_config.palm_pose = pickle.load(open(self.init_palm_pose_in_obj_path, 'rb'))

            grasp_inf_request.init_hand_object_config.hand_joint_state = self.preshape_response.allegro_joint_state[grasp_preshape_idx] 
            grasp_inf_request.init_hand_object_config.palm_pose = self.palm_object_pose 

            grasp_inf_request.object_name = self.object_name
            grasp_types = ['prec', 'power', 'all']
            grasp_type_id = grasp_types.index(grasp_type)
            grasp_inf_request.grasp_id = grasp_type_id * 10 + grasp_preshape_idx 
            
            cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
            grasp_inf_request.object_id = self.cur_object_id
            grasp_inf_request.grasp_type = grasp_type

            grasp_inf_response = grasp_inf_proxy(grasp_inf_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_active_learn failed: %s'%e)
        rospy.loginfo('Service grasp_active_learn is executed %s.'%str(grasp_inf_response.success))

        return grasp_inf_response


    def grasp_cnn_al_client(self, grasp_preshape_idx=-1):
        self.grasp_inf_response = None
        rospy.loginfo('Waiting for service grasp_active_learn.')
        rospy.wait_for_service('grasp_active_learn')
        rospy.loginfo('Calling service grasp_active_learn.')
        try:
            grasp_inf_proxy = rospy.ServiceProxy('grasp_active_learn', GraspActiveLearn)
            grasp_inf_request = GraspActiveLearnRequest()
            grasp_inf_request.grasp_planner_name = 'grasp_config_net'

            grasp_inf_request.rgbd_info.scene_cloud_normal = self.save_visual_data_response.scene_cloud_normal 
            grasp_inf_request.init_hand_pcd_goal_config.hand_joint_state = \
                                self.preshape_response.allegro_joint_state[grasp_preshape_idx] 

            if self.use_hd:
                grasp_inf_request.init_hand_pcd_goal_config.palm_pose = \
                                    self.preshape_response.palm_goal_pose_in_pcd[grasp_preshape_idx]
                grasp_inf_request.rgb_image_path = self.save_grasp_visual_data_request.rgb_image_save_path 
                grasp_inf_request.depth_image_path = self.save_grasp_visual_data_request.depth_image_save_path 
            else:
                grasp_inf_request.init_hand_pcd_goal_config.palm_pose = self.preshape_palm_goal_pose_sd_pcd
                # Downsample hd rgb to get sd rgb
                grasp_inf_request.rgb_image_path = self.save_grasp_visual_data_request.rgb_image_save_path 
                grasp_inf_request.depth_image_path = self.save_grasp_visual_data_request.sd_depth_image_save_path 
            grasp_inf_request.object_name = self.object_name
            cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
            grasp_inf_request.grasp_id = cur_grasp_id
            grasp_inf_request.object_id = cur_object_id
            #grasp_inf_request.grasp_success_label = self.save_grasp_visual_data_request.grasp_success_label
            grasp_inf_response = grasp_inf_proxy(grasp_inf_request) 
            raw_input('Wait')
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_active_learn failed: %s'%e)
        rospy.loginfo('Service grasp_active_learn is executed %s.'%str(grasp_inf_response.success))

        return grasp_inf_response


    def grasp_cnn_inf_client(self, grasp_preshape_idx=-1):
        rospy.loginfo('Waiting for service grasp_cnn_inference.')
        rospy.wait_for_service('grasp_cnn_inference')
        rospy.loginfo('Calling service grasp_cnn_inference.')
        try:
            grasp_inf_proxy = rospy.ServiceProxy('grasp_cnn_inference', GraspCnnInfer)
            grasp_inf_request = GraspCnnInferRequest()
            #TO DO: seperate the srv for visual data server and grasp data server! 
            #grasp_inf_request.rgbd_info.scene_sd_cloud_normal = self.save_visual_data_response.scene_sd_cloud_normal 
            grasp_inf_request.rgbd_info.scene_cloud_normal = self.save_visual_data_response.scene_cloud_normal 
            grasp_inf_request.init_hand_pcd_goal_config.hand_joint_state = \
                                self.preshape_response.allegro_joint_state[grasp_preshape_idx] 
            #grasp_inf_request.init_hand_pcd_true_config.hand_joint_state = self.true_hand_joint_state
            #grasp_inf_request.init_hand_pcd_true_config.palm_pose = self.true_palm_pose_pcd  

            #grasp_inf_request.close_hand_pcd_config.hand_joint_state = self.close_hand_js 
            #grasp_inf_request.close_hand_pcd_config.palm_pose = self.close_palm_pose_pcd
            if self.use_hd:
                grasp_inf_request.init_hand_pcd_goal_config.palm_pose = \
                                    self.preshape_response.palm_goal_pose_in_pcd[grasp_preshape_idx]
                grasp_inf_request.rgb_image_path = self.save_grasp_visual_data_request.rgb_image_save_path 
                grasp_inf_request.depth_image_path = self.save_grasp_visual_data_request.depth_image_save_path 
            else:
                grasp_inf_request.init_hand_pcd_goal_config.palm_pose = self.preshape_palm_goal_pose_sd_pcd
                # Downsample hd rgb to get sd rgb
                grasp_inf_request.rgb_image_path = self.save_grasp_visual_data_request.rgb_image_save_path 
                grasp_inf_request.depth_image_path = self.save_grasp_visual_data_request.sd_depth_image_save_path 
            grasp_inf_request.object_name = self.object_name
            cur_batch_id, cur_object_id, cur_grasp_id = self.update_batch_grasp_id()
            grasp_inf_request.grasp_id = cur_grasp_id
            grasp_inf_request.object_id = cur_object_id
            #grasp_inf_request.grasp_success_label = self.save_grasp_visual_data_request.grasp_success_label
            grasp_inf_response = grasp_inf_proxy(grasp_inf_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_cnn_inference failed: %s'%e)
        rospy.loginfo('Service grasp_cnn_inference is executed %s.'%str(grasp_inf_response))

        return grasp_inf_response

    def pgm_inf_all_heuristics(self, grasp_type):
        '''
        Perform inference for all heuristics and select the one 
        with highest success probability as the final inferred preshape.

        Arg: 
            grasp_type: 'prec', 'power', 'all'
        '''
        grasp_inf_response_list = []
        max_suc_prob = -1000000.
        max_suc_prob_idx = -1
        inits_suc_prob_list = []
        heuristics_num = len(self.preshape_response.allegro_joint_state)

        for i in xrange(heuristics_num):
        #for i in xrange(1):
            #Use each heuristic preshape for inference
            #Listen the heuristic palm pose in object frame for inference.
            self.listen_palm_pose('heu', i)
            #raw_input('Start inference.')
            grasp_inf_response = self.grasp_pgm_inf_client(grasp_type, i) 
            print i
            print 'inf_suc_prob:', grasp_inf_response.inf_suc_prob
            if grasp_inf_response.inf_suc_prob >= max_suc_prob:
                max_suc_prob = grasp_inf_response.inf_suc_prob
                max_suc_prob_idx = i
            grasp_inf_response_list.append(grasp_inf_response)
            inits_suc_prob_list.append(grasp_inf_response.init_suc_prob)
              
        max_grasp_inf = grasp_inf_response_list[max_suc_prob_idx]
        preshape = ExpPreshape()
        preshape.palm_pose = copy.deepcopy(max_grasp_inf.inf_hand_object_config.palm_pose)
        preshape.allegro_js = copy.deepcopy(max_grasp_inf.inf_hand_object_config.hand_joint_state)
        preshape.is_top = self.preshape_response.is_top_grasp[max_suc_prob_idx]
        preshape.grasp_type = grasp_type
        #preshape.grasp_control_type = None
        preshape.inf_suc_prob = max_suc_prob
        preshape.inits_suc_prob_list = inits_suc_prob_list
        preshape.max_init_idx = max_suc_prob_idx

        return preshape

    def cnn_inf_all_heuristics(self):
        '''
        Perform CNN inference of isrr paper for all heuristics and select the one 
        with highest success probability as the final inferred preshape.

        '''
        grasp_inf_response_list = []
        max_suc_prob = -1000000.
        max_suc_prob_idx = -1
        inits_suc_prob_list = []
        heuristics_num = len(self.preshape_response.allegro_joint_state)

        for i in xrange(heuristics_num):
        #for i in xrange(1):
            #Use each heuristic preshape for inference
            #Listen the heuristic palm pose in object frame for inference.
            self.listen_palm_pose('heu', i)
            #raw_input('Start inference.')
            #grasp_inf_response = self.grasp_pgm_inf_client(grasp_type, i) 
            grasp_inf_response = self.grasp_cnn_inf_client(i)
            print i
            print 'inf_suc_prob:', grasp_inf_response.inf_suc_prob
            if grasp_inf_response.inf_suc_prob >= max_suc_prob:
                max_suc_prob = grasp_inf_response.inf_suc_prob
                max_suc_prob_idx = i
            grasp_inf_response_list.append(grasp_inf_response)
            inits_suc_prob_list.append(grasp_inf_response.init_suc_prob)
              
        max_grasp_inf = grasp_inf_response_list[max_suc_prob_idx]
        preshape = ExpPreshape()
        preshape.palm_pose = copy.deepcopy(max_grasp_inf.inf_hand_object_config.palm_pose)
        preshape.allegro_js = copy.deepcopy(max_grasp_inf.inf_hand_object_config.hand_joint_state)
        preshape.is_top = self.preshape_response.is_top_grasp[max_suc_prob_idx]
        preshape.grasp_type = 'power'
        #preshape.grasp_control_type = None
        preshape.inf_suc_prob = max_suc_prob
        preshape.inits_suc_prob_list = inits_suc_prob_list
        preshape.max_init_idx = max_suc_prob_idx

        return preshape

    def find_heu_closest_to_cam(self):
        #find the closest non-top heuristic grasp to the camera.
        #Actually, the 1st heuristic grasp generated by the heuristic planner
        #should be the closest to camera. However, it sometimes returns the top 
        #grasp as the 1st one. I don't know why yet. 
        min_z_to_cam = 10.
        closest_heu_id = -1
        heuristics_num = len(self.preshape_response.allegro_joint_state)

        for i in xrange(heuristics_num):
            if self.preshape_response.is_top_grasp[i]:
                continue
            heu_z_cam = self.preshape_response.palm_goal_pose_in_pcd[i].pose.position.z 
            if heu_z_cam < min_z_to_cam:
                min_z_to_cam = heu_z_cam
                closest_heu_id = i

        if closest_heu_id == -1:
            rospy.logerr('Could not find heuristic preshape closest to camera!')
        return closest_heu_id

    def find_top_heu(self):
        # Find the top heuristic grasp
        heuristics_num = len(self.preshape_response.allegro_joint_state)

        for i in xrange(heuristics_num):
            if self.preshape_response.is_top_grasp[i]:
                return i

    def pgm_inf_one_heuristic(self, grasp_type, closest_heu_id=-1):
        '''
        Perform inference using one heuristic grasp. 
        Arg: 
            grasp_type: 'prec', 'power', 'all'
        '''
        grasp_inf_response_list = []
        
        if closest_heu_id != -1:
            self.listen_palm_pose('heu', closest_heu_id)
        #raw_input('Start inference.')
        grasp_inf_response = self.grasp_pgm_inf_client(grasp_type, closest_heu_id) 

        preshape = ExpPreshape()
        preshape.palm_pose = copy.deepcopy(grasp_inf_response.inf_hand_object_config.palm_pose)

        #Add offset for robot modelling error.
        #preshape.palm_pose.pose.position.x += self.model_err_x_offset

        preshape.allegro_js = copy.deepcopy(grasp_inf_response.inf_hand_object_config.hand_joint_state)
        preshape.grasp_type = grasp_type
        #preshape.grasp_control_type = None
        preshape.inf_suc_prob = grasp_inf_response.inf_suc_prob
        #preshape.init_suc_prob = grasp_inf_response.init_suc_prob
        preshape.inits_suc_prob_list = [grasp_inf_response.init_suc_prob]
        #Notice None value message will mess up the ros service to have inconnect error!
        preshape.is_top = False
        #preshape.inits_suc_prob_list = []
        preshape.max_init_idx = -1

        return preshape

    def al_one_heuristic(self, grasp_type='', closest_heu_id=-1):
        '''
        Perform max entropy active learning using one heuristic grasp. 
        Arg: 
            grasp_type: 'prec', 'power', 'all'
        '''

        grasp_inf_response_list = []
        
        if closest_heu_id != -1:
            self.listen_palm_pose('heu', closest_heu_id)
        #raw_input('Start inference.')
        grasp_inf_response = self.grasp_pgm_al_client(grasp_type, closest_heu_id) 
        #grasp_inf_response = self.grasp_cnn_al_client(closest_heu_id)

        preshape = ExpPreshape()
        preshape.palm_pose = copy.deepcopy(grasp_inf_response.inf_hand_object_config.palm_pose)

        #Add offset for robot modelling error.
        #preshape.palm_pose.pose.position.x += self.model_err_x_offset

        preshape.allegro_js = copy.deepcopy(grasp_inf_response.inf_hand_object_config.hand_joint_state)
        preshape.grasp_type = grasp_type
        #preshape.grasp_control_type = None
        preshape.inf_suc_prob = grasp_inf_response.inf_entropy
        #preshape.init_suc_prob = grasp_inf_response.init_suc_prob
        preshape.inits_suc_prob_list = [grasp_inf_response.inf_entropy]
        #Notice None value message will mess up the ros service to have inconnect error!
        preshape.is_top = False
        #preshape.inits_suc_prob_list = []
        preshape.max_init_idx = -1

        return preshape


    def grasp_pgm_inf_cst_init(self, grasp_type):
        '''
        Inference using BFGS with constant initialization.
        '''
        self.grasp_inf_response = None
        rospy.loginfo('Waiting for service grasp_pgm_inference.')
        rospy.wait_for_service('grasp_pgm_inference')
        rospy.loginfo('Calling service grasp_pgm_inference.')
        try:
            grasp_inf_proxy = rospy.ServiceProxy('grasp_pgm_inference', GraspPgmInf)
            grasp_inf_request = GraspPgmInfRequest()
            #Use hd cloud.
            grasp_inf_request.scene_cloud = self.object_segment_response.scene_cloud 
            grasp_inf_request.object_frame_id = self.grasp_obj_frame_id 
            grasp_inf_request.init_config_array = np.zeros(14)

            grasp_inf_request.object_name = self.object_name
            grasp_types = ['prec', 'power', 'all']
            grasp_type_id = grasp_types.index(grasp_type)
            grasp_inf_request.grasp_id =  grasp_type_id 
            grasp_inf_request.object_id = self.cur_object_id
            grasp_inf_request.grasp_type = grasp_type

            grasp_inf_response = grasp_inf_proxy(grasp_inf_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_pgm_inference failed: %s'%e)
        rospy.loginfo('Service grasp_pgm_inference is executed %s.'%str(grasp_inf_response.success))
        rospy.loginfo('Inference success probability: %s'%str(grasp_inf_response.inf_suc_prob))

        preshape = ExpPreshape()
        preshape.palm_pose = copy.deepcopy(grasp_inf_response.inf_hand_object_config.palm_pose)
        preshape.allegro_js = copy.deepcopy(grasp_inf_response.inf_hand_object_config.hand_joint_state)
        preshape.grasp_type = grasp_type
        #preshape.grasp_control_type = None
        preshape.inf_suc_prob = grasp_inf_response.inf_suc_prob
        #preshape.init_suc_prob = grasp_inf_response.init_suc_prob
        preshape.inits_suc_prob_list = [grasp_inf_response.init_suc_prob]
        #Notice None value message will mess up the ros service to have inconnect error!
        preshape.is_top = False
        preshape.max_init_idx = -1

        return preshape

    def create_heu_exp_preshape(self, heu_preshape_idx, grasp_control_type):
        '''
        Create experiment preshape for heuritics preshapes.
        '''
        preshape = ExpPreshape()

        #preshape.palm_pose = copy.deepcopy(self.preshape_response.palm_goal_pose_in_pcd[heu_preshape_idx])
        #preshape.allegro_js = copy.deepcopy(self.preshape_response.allegro_joint_state[heu_preshape_idx])

        ##Save the initialization heuristic grasp for the current object id when inferring the first grasp of the grasp trial.
        #if self.cur_grasp_id % self.grasps_num_per_object == self.grasps_num_per_object - 1:
        #    #preshape.palm_pose = copy.deepcopy(self.preshape_response.palm_goal_pose_in_pcd[heu_preshape_idx])
        #    #Trasnform plam pose to object frame.
        #    self.listen_palm_pose('heu', heu_preshape_idx)
        #    preshape.palm_pose = self.palm_object_pose
        #    preshape.allegro_js = copy.deepcopy(self.preshape_response.allegro_joint_state[heu_preshape_idx])
        #    #pickle.dump(self.palm_object_pose, open(self.init_palm_pose_in_cam_path, 'wb'))
        ##Load the initialization heuristics grasp for the current object id for 2nd and following grasps.
        #else:
        #    preshape.allegro_js = pickle.load(open(self.init_hand_js_path, 'rb'))
        #    #preshape.palm_pose = pickle.load(open(self.init_palm_pose_in_cam_path, 'rb'))
        #    preshape.palm_pose = pickle.load(open(self.init_palm_pose_in_obj_path, 'rb'))
        #    print '###***', preshape.palm_pose

        self.listen_palm_pose('heu', heu_preshape_idx)
        preshape.palm_pose = self.palm_object_pose
        preshape.allegro_js = copy.deepcopy(self.preshape_response.allegro_joint_state[heu_preshape_idx])

        preshape.is_top = self.preshape_response.is_top_grasp[heu_preshape_idx]
        preshape.grasp_type = 'heuristic'
        preshape.grasp_control_type = grasp_control_type
        #preshape.inf_suc_prob = max_suc_prob
        #preshape.inits_suc_prob_list = inits_inf_suc_prob
        #preshape.max_init_idx = max_suc_prob_idx
        #Notice None value message will mess up the ros service to have inconnect error!
        preshape.inits_suc_prob_list = []
        preshape.inf_suc_prob = 0
        preshape.max_init_idx = -1

        return preshape

    def segment_and_generate_preshape(self):
        #segment -> generate heruistic preshapes -> 
        #for each heristics, listen palm and run inference ->
        #pick the max suc grasp -> update and broadcast grasp pose
        self.segment_object_client(align_obj_frame=True)
        self.save_grasp_visual_data()
        #Still need to call preshape client even if the initialization heuristic grasp is already saved, since
        #we need to broadcast grasp_object pose.
        #TODO: seperate the broadcasting of grasp_object pose and heuristics poses.
        self.gen_grasp_preshape_client()

        #Generate the initialization heuristic grasp for the current object id at the begining of the grasp trial.
        if self.cur_grasp_id % self.grasps_num_per_object == self.grasps_num_per_object - 1:
            #self.gen_grasp_preshape_client()
            closest_cam_heu_id = self.find_heu_closest_to_cam()
        else:
            closest_cam_heu_id = -1
        
        exp_preshape_list = []
        #grasp_inf_method = ''
        grasp_inf_method = 'pgm_inf'
        #grasp_inf_method = 'cnn_inf'
        if grasp_inf_method == 'pgm_inf':
            #grasp_types = ['prec', 'power', 'all']
            #grasp_types = ['prec', 'power']
            grasp_types = ['prec']
            for g_type in grasp_types:
                #raw_input('Infer.')
                #preshape = self.pgm_inf_all_heuristics(g_type)
                #preshape = self.inf_closest_heuristic(closest_heu_id)
                #preshape = self.pgm_inf_one_heuristic(g_type, closest_cam_heu_id)
                preshape = self.al_one_heuristic(grasp_type=g_type, closest_heu_id=closest_cam_heu_id)
                #preshape = self.grasp_pgm_inf_cst_init(g_type)

                if g_type == 'all':
                    prec_preshape = copy.deepcopy(preshape)
                    prec_preshape.grasp_control_type = 'prec'
                    exp_preshape_list.append(prec_preshape)
                    power_preshape = copy.deepcopy(preshape)
                    power_preshape.grasp_control_type = 'power'
                    exp_preshape_list.append(power_preshape)
                else:
                    preshape.grasp_control_type = g_type
                    exp_preshape_list.append(preshape)
        elif grasp_inf_method == 'cnn_inf':
            #preshape = self.cnn_inf_all_heuristics()
            preshape = self.al_one_heuristic(closest_heu_id=closest_cam_heu_id)
            power_preshape = copy.deepcopy(preshape)
            power_preshape.grasp_control_type = 'power'
            exp_preshape_list.append(power_preshape)

        preshape = self.create_heu_exp_preshape(closest_cam_heu_id, 'prec')
        exp_preshape_list.append(preshape)
        #preshape = self.create_heu_exp_preshape(closest_cam_heu_id, 'power')
        #exp_preshape_list.append(preshape)

        #preshape = self.create_heu_exp_preshape(0, 'prec')
        #exp_preshape_list.append(preshape)

        #heuristics_num = len(self.preshape_response.allegro_joint_state)
        #for i in xrange(heuristics_num):
        #    if not self.preshape_response.is_top_grasp[i]:
        #        preshape = self.create_heu_exp_preshape(i, 'prec')
        #        exp_preshape_list.append(preshape)
        
        #top_heu_id = self.find_top_heu()
        #preshape = self.create_heu_exp_preshape(top_heu_id, 'prec')
        #exp_preshape_list.append(preshape)

        #Update and broadcast all preshapes' tf
        exp_palm_poses = [copy.deepcopy(p.palm_pose) for p in exp_preshape_list]

        ##Add offset for robot modelling error.
        #for i in xrange(len(exp_palm_poses)):
        #    exp_palm_poses[i].pose.position.x += self.model_err_x_offset

        ## Increase hight to avoid collision with tables
        #for i in xrange(len(exp_palm_poses)):
        #    exp_palm_poses[i].pose.position.z += 0.05

        self.update_detection_grasp_preshape_client(exp_palm_poses)

        return exp_preshape_list

    def grasp_and_lift_object_steps(self, exp_preshape, exp_preshape_idx):
        #create moveit scene -> move hand to preshape (To do) -> move arm to preshape
        #close hand to grasp -> clean object from moveit -> lift object 

        #Listen mount poses of experiment grasp preshapes to 
        #find arm plans.
        self.listen_mount_pose('exp', exp_preshape_idx)
        #Listen experiment grasp preshape palm poses
        #in object and world frame for data recording.
        self.listen_palm_pose('exp', exp_preshape_idx)

        self.create_moveit_scene_client()

        self.control_allegro_config_client(exp_preshape=exp_preshape)

        moveit_found_plan = self.arm_moveit_planner_client()
        if not moveit_found_plan:
            return False
        if not self.execute_arm_plan():
            return False

        self.true_palm_pose_pcd, self.true_palm_pose_world = self.listen_true_palm_pose()
        self.true_preshape_hand_js = self.true_hand_joint_state

        #Lego
        #if exp_preshape.grasp_control_type == 'power':
        #    non_thumb_speed = 0.35
        #    thumb_speed = 0.3
        #elif exp_preshape.grasp_control_type == 'prec':
        #    non_thumb_speed = 0.15
        #    thumb_speed = 0.1 #0.05
        #Pringle, soft scrub, mustard
        #non_thumb_speed = 0.35
        #thumb_speed = 0.3
        #self.grasp_client(exp_preshape.grasp_control_type, non_thumb_speed, thumb_speed)

        self.bag_tactile_visual_data(grasp_phase='preshape', operation='start')
        rospy.sleep(1.)
        self.bag_tactile_visual_data(grasp_phase='preshape', operation='stop')
        raw_input('Grasp?')
        self.bag_tactile_visual_data(grasp_phase='grasp', operation='start')
        self.hand_grasp_control(exp_preshape.grasp_control_type)
        rospy.sleep(1.)
        self.bag_tactile_visual_data(grasp_phase='grasp', operation='stop')

        self.close_palm_pose_pcd, self.close_palm_pose_world = self.listen_true_palm_pose()
        self.close_hand_js = self.true_hand_joint_state 

        #Remove the object from the hand for collision checking.
        self.clean_moveit_scene_client()
        #lift = raw_input('Lift or not (y/n)?')
        lift = 'y'
        if lift == 'y':
            task_vel_lift_succes = self.lift_task_vel_planner_client()
            if not task_vel_lift_succes:
                rospy.loginfo('Task velocity straight line planner fails to find a valid plan' \
                               ' for lifting. Switch to the moveit planner.')
                self.lift_moveit_planner_client()
            self.execute_arm_plan(send_cmd_manually=True)
            self.bag_tactile_visual_data(grasp_phase='lift', operation='start')
            rospy.loginfo('Wait for a few seconds before getting the lifting visual data.')
            rospy.sleep(1.)
            self.bag_tactile_visual_data(grasp_phase='lift', operation='stop')
            self.get_visual_data_client()
        return True 

    def place_object_steps(self):
        #self.place_arm_movement_client()
        self.place_control_allegro_client()
        if self.use_sim:
            self.create_moveit_scene_client()
        self.move_arm_home_client()

if __name__ == '__main__':
    #Segment -> generate preshape -> listen pose ->
    #create moveit scene -> move hand to preshape (To do) -> move arm to preshape
    #close hand to grasp (To do) -> clean object from moveit -> lift object 
    #-> record data -> hand go home -> arm go home

    dc_client = GraspDataCollectionClient()
    
    move_home_cmd = raw_input('Move robot to the home position esier for planning or not? (y or n).')
    dc_client.move_robot_home = move_home_cmd == 'y'
    #dc_client.move_robot_home = False #True
    if dc_client.move_robot_home:
       dc_client.move_arm_home()
       rospy.loginfo('Hand go home.')
       dc_client.control_allegro_config_client(go_home=True)

    while True:
        #dc_client.create_ll4ma_planner()
        object_name = raw_input('### Input the new object name or input stop to quit: \n')
        if object_name == 'stop':
            rospy.loginfo('Quit grasp data collection.')
            break
        #while True:
        #    object_name_confirmation = raw_input('### Are you sure ' + object_name + ' is the correct object name? ' + \
        #            'Please input y to continue or n to re-input object name. \n')
        #    if object_name_confirmation == 'y':
        #        break
        #    object_name = raw_input('### Input the new object name or input stop to quit: \n')

        dc_client.set_object_name(object_name)
        dc_client.get_cur_object_id()
        dc_client.get_cur_grasp_id_name()
        exp_preshape_list = dc_client.segment_and_generate_preshape()

        for preshape_idx, exp_preshape in enumerate(exp_preshape_list): 
            rospy.loginfo('object id: %s, object name: %s, grasp id: %s \n', 
                    dc_client.cur_object_id, object_name, preshape_idx)
            exe_grasp = raw_input('Do you want to try this grasp? Please input: y (yes) or n (no).')
            if exe_grasp == 'n':
                continue

            #grasp_arm_plan = dc_client.grasp_and_lift_object_steps(exp_preshape, preshape_idx)
            #if not grasp_arm_plan:
            #    rospy.logerr('Can not find moveit plan to grasp. \n')
            #    raw_input('Press to continue to record the grasp without arm plan.')

            keep_current_grasp = dc_client.record_grasp_data_client(exp_preshape)

            #Move arm and hand home after grasping and lifting.
            #raw_input('Ready to move arm home. Please release the object and take grasp photos.')
            #dc_client.move_arm_home()
            #rospy.loginfo('Hand go home.')
            #dc_client.control_allegro_config_client(go_home=True)

        rospy.loginfo('All grasps are finished for this object.') 


