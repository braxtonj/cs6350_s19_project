#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from grasp_pipeline.srv import *
import copy

#To do: change this file to a ros server to load different objects.
class ManageSceneInMoveit:
    def __init__(self, gazebo_model_state_topic='/gazebo/model_states'):
        #roscpp_initialize(sys.argv)
        self.use_sim = rospy.get_param('~use_sim', False)
        #Shrink the object bounding box to decrease the collision chance.
        self.shrink_obj_bb_dist = rospy.get_param('~shrink_obj_bb_dist', 0.)
        rospy.init_node('manage_moveit_scene_node')
        if self.use_sim:
            rospy.Subscriber(gazebo_model_state_topic, ModelStates, self.get_world_states)
            self.box_len_x = .5
            self.box_len_y = .5
            self.box_len_z = .5
        else:
            self.box_pose = PoseStamped()
            self.box_pose.header.frame_id = 'world'
            #self.box_pose.pose.position.x = 0.
            #self.box_pose.pose.position.y = -0.88
            # Change table collision box model position for the new robot arm pose
            self.box_pose.pose.position.x = 0.
            self.box_pose.pose.position.y = -0.88
            self.box_pose.pose.position.z = 0.59 + 0.015
            self.box_pose.pose.orientation.x = 0. 
            self.box_pose.pose.orientation.y = 0. 
            self.box_pose.pose.orientation.z = 0. 
            self.box_pose.pose.orientation.w = 1. 
            self.box_len_x = 1.4
            self.box_len_y = 1.4
            #self.box_len_x = 1.4
            #self.box_len_y = 0.9
            self.box_len_z = 0.015#4

            self.wall_pose = PoseStamped()
            self.wall_pose.header.frame_id = 'world'
            # Change table collision wall model position for the new robot arm pose
            self.wall_pose.pose.position.x = 0.
            self.wall_pose.pose.position.y = -1.
            self.wall_pose.pose.position.z = 0.
            self.wall_pose.pose.orientation.x = 0. 
            self.wall_pose.pose.orientation.y = 0. 
            self.wall_pose.pose.orientation.z = 0. 
            self.wall_pose.pose.orientation.w = 1. 
            self.wall_len_x = 5.
            self.wall_len_y = 0.01#5.
            self.wall_len_z = 5.#5.

    def get_world_states(self, gz_model_msg):
        #rostopic echo /gazebo/model_states
        #name: ['ground_plane', 'kinect', 'grasping_object', 'unit_box', 'lbr4_allegro']
        self.obj_pos = gz_model_msg.pose[2].position
        self.obj_ort = gz_model_msg.pose[2].orientation
        self.box_pos = gz_model_msg.pose[3].position
        self.box_ort = gz_model_msg.pose[3].orientation
    
    def handle_create_moveit_scene(self, req):        
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        
        if self.use_sim:
            box_pose = PoseStamped()
            box_pose.header.frame_id = 'world'
            box_pose.pose.position = self.box_pos
            box_pose.pose.orientation = self.box_ort
            scene.add_box('box', box_pose, (self.box_len_x, self.box_len_y, self.box_len_z))

            obj_pose = PoseStamped()
            obj_pose.header.frame_id = 'world'
            obj_pose.pose.position = self.obj_pos
            obj_pose.pose.orientation = self.obj_ort

        else:
            #obj_pose = req.object_pose_in_world
            #rospy.loginfo('***obj_pose: %s'%str(obj_pose))

            #Add a box for the table.
            scene.add_box('box', self.box_pose, (self.box_len_x, self.box_len_y, self.box_len_z))
            #scene.add_box('wall', self.wall_pose, (self.wall_len_x, self.wall_len_y, self.wall_len_z))

            ##Add a boundig box for the object.
            #width = req.obj_seg.width
            #if width > self.shrink_obj_bb_dist:
            #    width -= self.shrink_obj_bb_dist
            #height = req.obj_seg.height
            #if height > self.shrink_obj_bb_dist:
            #    height -= self.shrink_obj_bb_dist
            #depth = req.obj_seg.depth
            #if depth > self.shrink_obj_bb_dist:
            #    depth -= self.shrink_obj_bb_dist
            #scene.add_box('grasp_object', req.object_pose_world, (width, height, depth))

            #scene.add_box('grasp_object', req.object_pose_world, (req.obj_seg.width, 0.01, 0.01))
            #scene.add_box('grasp_object', req.object_pose_world, (0.02, 0.02, req.obj_seg.width))
            #scene.add_box('grasp_object', req.object_pose_world, (req.obj_seg.depth, req.obj_seg.height, req.obj_seg.width))

            #scene.add_box('grasp_object', req.object_pose_world, (req.obj_seg.depth, req.obj_seg.height, req.obj_seg.width + 0.05))
            #scene.add_box('grasp_object', req.object_pose_world, (0.01, 0.02, req.obj_seg.width + 0.35))

            ##robot model error offset
            #robot_model_offset = 0.04
            object_pose = copy.deepcopy(req.object_pose_world)
            #object_pose.pose.position.x += robot_model_offset

            #scene.add_box('grasp_object', object_pose, (req.obj_seg.depth, req.obj_seg.height, req.obj_seg.width))
            #scene.add_box('grasp_object', object_pose, (0.01, req.obj_seg.height, req.obj_seg.width))
            #scene.add_box('grasp_object', object_pose, (0.01, 0.02, req.obj_seg.width + 0.1))
            #scene.add_box('grasp_object', object_pose, (0.02, 0.02, req.obj_seg.width + 0.2))
            #scene.add_box('grasp_object', object_pose, (0.02, 0.02, req.obj_seg.width))
            #scene.add_box('grasp_object', object_pose, (0.02, 0.02, req.obj_seg.depth))
            scene.add_box('grasp_object', object_pose, (req.obj_seg.width, req.obj_seg.height, req.obj_seg.depth))
            

        #scene.add_mesh('grasp_object', obj_pose, 
        #        '/home/kai/Workspace/grasp_data_collection_ws/src/urlg_robots_gazebo/worlds/objects/pringle/optimized_poisson_texture_mapped_mesh.dae')

        rospy.sleep(1)
        
        response = ManageMoveitSceneResponse()
        response.success = True
        return response

    def create_moveit_scene_server(self):
        rospy.Service('create_moveit_scene', ManageMoveitScene, self.handle_create_moveit_scene)
        rospy.loginfo('Service create_scene:')
        rospy.loginfo('Ready to create the moveit_scene.')

    def handle_clean_moveit_scene(self, req):        
        scene = PlanningSceneInterface()
        rospy.sleep(1)
    
        # clean the scene
        #scene.remove_world_object('box')
        scene.remove_world_object('grasp_object')
         
        response = ManageMoveitSceneResponse()
        response.success = True
        return response
   
    def clean_moveit_scene_server(self):
        rospy.Service('clean_moveit_scene', ManageMoveitScene, self.handle_clean_moveit_scene)
        rospy.loginfo('Service clean_scene:')
        rospy.loginfo('Ready to clean the moveit_scene.')

if __name__=='__main__':
    ms = ManageSceneInMoveit()
    ms.create_moveit_scene_server()
    ms.clean_moveit_scene_server()
    rospy.spin()
