#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline.srv import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState, CameraInfo
import tf
import numpy as np
import h5py
#from compute_finger_tip_location import ComputeFingerTipPose

class RecordGraspData():
    def __init__(self):
        rospy.init_node('record_grasp_data_server')
        self.num_grasps_per_object = rospy.get_param('~num_grasps_per_object', 10)
        self.data_recording_path = rospy.get_param('~data_recording_path', '')
        self.use_hd = rospy.get_param('~use_hd', True)
    
    def handle_record_grasp_data(self, req):
        file_name = 'object_' + str(req.object_id) + '_' + req.object_name + \
                '_grasp_' + str(req.grasp_id) + '_' + req.grasp_type + '_' + req.grasp_control_type + '.h5' 
        self.grasp_file_name = self.data_recording_path +  'queries_batch_' + \
                                str(req.batch_id) + '/grasp_data/' + file_name
        grasp_file = h5py.File(self.grasp_file_name, 'w')

        #hand_joint_state_name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
        #           'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
        #           'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
        #           'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']
        #hand_js_name_key = 'hand_joint_state_name' 
        #if hand_js_name_key not in grasp_file:
        #    grasp_file.create_dataset(hand_js_name_key, data=hand_joint_state_name)


        # [()] is the way to get the scalar value from h5 file.
        #object_id = grasp_file['max_object_id'][()] 
        object_grasp_id = 'object_' + str(req.object_id) + '_grasp_' + str(req.grasp_id)

        grasp_object_name_key = 'object_' + str(req.object_id) + '_name'
        #print 'object_name:', req.object_name
        #print 'req.grasp_id:', req.grasp_id
        if grasp_object_name_key not in grasp_file:
            grasp_file.create_dataset(grasp_object_name_key, data=req.object_name)
    
        grasp_time_stamp_key = object_grasp_id + '_time_stamp'
        if grasp_time_stamp_key not in grasp_file:
            grasp_file.create_dataset(grasp_time_stamp_key, data=req.time_stamp)

        preshape_palm_world_pose_list = [req.preshape_palm_world_pose.pose.position.x, req.preshape_palm_world_pose.pose.position.y,
               req.preshape_palm_world_pose.pose.position.z, req.preshape_palm_world_pose.pose.orientation.x, 
               req.preshape_palm_world_pose.pose.orientation.y, req.preshape_palm_world_pose.pose.orientation.z, 
               req.preshape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_preshape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=preshape_palm_world_pose_list)

        preshape_palm_pose_list = [req.preshape_palm_pose.pose.position.x, req.preshape_palm_pose.pose.position.y,
               req.preshape_palm_pose.pose.position.z, req.preshape_palm_pose.pose.orientation.x, 
               req.preshape_palm_pose.pose.orientation.y, req.preshape_palm_pose.pose.orientation.z, 
               req.preshape_palm_pose.pose.orientation.w]
        palm_pose_key = object_grasp_id + '_preshape_palm_pose'
        if palm_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_pose_key, 
                    data=preshape_palm_pose_list)

        palm_pose_frame_key = object_grasp_id + '_preshape_pose_frame'
        if palm_pose_frame_key not in grasp_file:
            grasp_file.create_dataset(palm_pose_frame_key, data=req.preshape_palm_pose.header.frame_id)

        preshape_js_position_key = object_grasp_id + '_preshape_joint_state_position'
        if preshape_js_position_key not in grasp_file:
            grasp_file.create_dataset(preshape_js_position_key, 
                    data=req.preshape_allegro_joint_state.position)

        top_grasp_key = object_grasp_id + '_top_grasp'
        if top_grasp_key not in grasp_file:
            grasp_file.create_dataset(top_grasp_key,
                    data=req.top_grasp)

        grasp_type_key = object_grasp_id + '_grasp_type'
        if grasp_type_key not in grasp_file:
            grasp_file.create_dataset(grasp_type_key,
                    data=req.grasp_type)

        grasp_control_type_key = object_grasp_id + '_grasp_control_type'
        if grasp_control_type_key not in grasp_file:
            grasp_file.create_dataset(grasp_control_type_key,
                    data=req.grasp_control_type)

        inf_suc_prob_key = object_grasp_id + '_inf_suc_prob'
        if inf_suc_prob_key not in grasp_file:
            grasp_file.create_dataset(inf_suc_prob_key,
                    data=req.inf_suc_prob)

        #init_suc_prob_key = object_grasp_id + '_init_suc_prob'
        #if init_suc_prob_key not in grasp_file:
        #    grasp_file.create_dataset(init_suc_prob_key,
        #            data=req.init_suc_prob)

        max_init_idx_key = object_grasp_id + '_max_init_idx'
        if max_init_idx_key not in grasp_file:
            grasp_file.create_dataset(max_init_idx_key,
                    data=req.max_init_idx)

        inits_suc_prob_list_key = object_grasp_id + '_inits_suc_prob_list'
        if inits_suc_prob_list_key not in grasp_file:
            grasp_file.create_dataset(inits_suc_prob_list_key,
                    data=req.inits_suc_prob_list)

        object_pose_list = [req.object_pose.pose.position.x, req.object_pose.pose.position.y,
               req.object_pose.pose.position.z, req.object_pose.pose.orientation.x,
               req.object_pose.pose.orientation.y, req.object_pose.pose.orientation.z, 
               req.object_pose.pose.orientation.w]
        object_pose_key = object_grasp_id + '_object_pose'
        if object_pose_key not in grasp_file:
            grasp_file.create_dataset(object_pose_key, 
                    data=object_pose_list)

        true_preshape_palm_pcd_pose_list = [req.true_preshape_palm_pcd_pose.pose.position.x, req.true_preshape_palm_pcd_pose.pose.position.y,
               req.true_preshape_palm_pcd_pose.pose.position.z, req.true_preshape_palm_pcd_pose.pose.orientation.x,
               req.true_preshape_palm_pcd_pose.pose.orientation.y, req.true_preshape_palm_pcd_pose.pose.orientation.z, 
               req.true_preshape_palm_pcd_pose.pose.orientation.w]
        palm_pcd_pose_key = object_grasp_id + '_true_preshape_palm_pcd_pose'
        if palm_pcd_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_pcd_pose_key, 
                    data=true_preshape_palm_pcd_pose_list)

        true_preshape_palm_world_pose_list = [req.true_preshape_palm_world_pose.pose.position.x, req.true_preshape_palm_world_pose.pose.position.y,
               req.true_preshape_palm_world_pose.pose.position.z, req.true_preshape_palm_world_pose.pose.orientation.x, 
               req.true_preshape_palm_world_pose.pose.orientation.y, req.true_preshape_palm_world_pose.pose.orientation.z, 
               req.true_preshape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_true_preshape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=true_preshape_palm_world_pose_list)

        true_preshape_js_position_key = object_grasp_id + '_true_preshape_js_position'
        if true_preshape_js_position_key not in grasp_file:
            grasp_file.create_dataset(true_preshape_js_position_key, 
                    data=req.true_preshape_joint_state.position)

        close_shape_palm_pcd_pose_list = [req.close_shape_palm_pcd_pose.pose.position.x, req.close_shape_palm_pcd_pose.pose.position.y,
               req.close_shape_palm_pcd_pose.pose.position.z, req.close_shape_palm_pcd_pose.pose.orientation.x,
               req.close_shape_palm_pcd_pose.pose.orientation.y, req.close_shape_palm_pcd_pose.pose.orientation.z, 
               req.close_shape_palm_pcd_pose.pose.orientation.w]
        palm_pcd_pose_key = object_grasp_id + '_close_shape_palm_pcd_pose'
        if palm_pcd_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_pcd_pose_key, 
                    data=close_shape_palm_pcd_pose_list)

        close_shape_palm_world_pose_list = [req.close_shape_palm_world_pose.pose.position.x, req.close_shape_palm_world_pose.pose.position.y,
               req.close_shape_palm_world_pose.pose.position.z, req.close_shape_palm_world_pose.pose.orientation.x, 
               req.close_shape_palm_world_pose.pose.orientation.y, req.close_shape_palm_world_pose.pose.orientation.z, 
               req.close_shape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_close_shape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=close_shape_palm_world_pose_list)

        close_js_position_key = object_grasp_id + '_close_shape_joint_state_position'
        if close_js_position_key not in grasp_file:
            grasp_file.create_dataset(close_js_position_key, 
                    data=req.close_shape_allegro_joint_state.position)

        grasp_label_key = object_grasp_id + '_grasp_label' 
        if grasp_label_key not in grasp_file:
            grasp_file.create_dataset(grasp_label_key, 
                    data=req.grasp_success_label)

        response = GraspDataRecordingResponse()
        #response.object_id = object_id
        response.save_h5_success = True
        grasp_file.close()
        return response

    def create_record_data_server(self):
        rospy.Service('record_grasp_data', GraspDataRecording, self.handle_record_grasp_data)
        rospy.loginfo('Service record_grasp_data:')
        rospy.loginfo('Ready to record grasp data.')

if __name__ == '__main__':
    record_grasp_data = RecordGraspData()
    record_grasp_data.create_record_data_server()
    rospy.spin()

