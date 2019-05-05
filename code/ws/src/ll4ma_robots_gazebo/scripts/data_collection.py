#!/usr/bin/env python

# Script to spawn in objects

import os
import rospy, tf, rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

import random


def run_data_collection():

    # init environment with robot and no other objects

    for env in environments:
        pass

        # spawn objects

        # generate robot_configs

        for conf in robot_configs:
            pass

            # move robot to config

            # get point cloud data, save to file
            
            # get joint angles, save to file?

            # check for collision, output collision status

        # clear objects


def read_models(directory):
    # Generate list of model urdf files from a directory containing only urdf files
    model_names = os.listdir(directory)
    print(model_names)
    models = []
    for model in model_names:
        with open(models_dir + model, "r") as f:
            model_xml = f.read()
            models.append(model_xml)


    return models

def spawn_models(models,max_models):

    model_names = []

    # initialize model orientation
    q = tf.transformations.quaternion_from_euler(0,0,0)
    orient = Quaternion(q[0], q[1], q[2], q[3])

    for i in xrange(max_models):
        model = random.choice(models)
        bin_y   =   3 *   (i    /   6)  -   4
        bin_x   =   3 *   (i    %   6)  -   4
        model_name   =   "model_{0}_0".format(i)
        model_names.append(model_name)
        print("Spawning model: {}".format(model_name))
        model_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=4),   orient)
        spawn_model(model_name, model, "", model_pose, "world")

    return model_names


def delete_models(model_names):

    for model in model_names:
        print("Deleting model: {}".format(model))
        delete_model(model)




if __name__ == "__main__":

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

    rospy.sleep(10)

    delete_models(model_names)


    

