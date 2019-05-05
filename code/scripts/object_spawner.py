#!/usr/bin/env python

# Script to spawn in objects

import os
import rospy, tf, rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np
import random


def read_models(directory):
    # Generate list of model urdf files from a directory containing only urdf files
    model_names = os.listdir(directory + '/boxes/')
    print(model_names)
    models = []
    for model in model_names:
        with open(models_dir + '/boxes/' + model, "r") as f:
            model_xml = f.read()
            models.append(model_xml)

    return models


def generate_poses(n_models):

    poses = []

    # workspace boundaries
    min_x, max_x = [0.5, 2.5]
    min_y, max_y = [-1.5, 1.5]
    x_range = max_x - min_x
    y_range = max_y - min_y

    for i in range(n_models):
        # calculate random model orientation
        euler_angles = 2*np.pi*np.random.rand(3)
        # q = tf.transformations.quaternion_from_euler(*euler_angles)
        q = tf.transformations.quaternion_from_euler(0,0, euler_angles[2])
        orient = Quaternion(*q)

        # calculate random model position
        bin_x = x_range*np.random.rand(1) + min_x
        bin_y = y_range*np.random.rand(1) + min_y

        print('Position: {}, {}'.format(bin_x, bin_y))

        model_pose = Pose(Point(x=bin_x, y=bin_y, z=4), orient)
        poses.append(model_pose)

    return poses

def spawn_models(models,n_models):

    model_names = []

    # generate random model poses
    poses = generate_poses(n_models)

    for i in range(n_models):
        model = random.choice(models)
        model_name   =   "model_{0}_0".format(i)
        model_names.append(model_name)
        print("Spawning model:%s", model_name)
        model_pose   =  poses[i] 
        spawn_model(model_name, model, "", model_pose, "world")
        rospy.sleep(0.5)

    return model_names



if __name__ == '__main__':
    # initialize rospy 
    rospy.init_node("object_spawner")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    # Get models directory from ll4ma_robots_gazebo
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path('ll4ma_robots_gazebo')
    models_dir = package_dir + '/models/'

    models = read_models(models_dir)

    model_names = spawn_models(models,12)
    