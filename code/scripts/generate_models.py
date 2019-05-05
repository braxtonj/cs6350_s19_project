#!/usr/bin/env python

# Script to generate model .urdf files
# Author: Cade Parkison

import os
import sys
import xml.etree.ElementTree as ET

from scipy.stats import truncnorm
import numpy as np
import random


def generate_boxes(n_shapes, output_dir):

    os.mkdir(output_dir)

    for i in range(n_shapes):

        tree = ET.parse('box_template.urdf')
        root = tree.getroot()

        # Generate XYZ box dimensions from truncated normal distribution
        min_x, max_x = [0.25, .75]
        min_y, max_y = [0.25, .75]
        min_z, max_z = [0.25, .75]
        mean_x = np.mean([min_x, max_x])
        mean_y = np.mean([min_y, max_y])
        mean_z = np.mean([min_z, max_z])
        sigma = 1   # standard deviation
        val_x = np.round(truncnorm((min_x-mean_x)/sigma, (max_x-mean_x)/sigma, loc=mean_x, scale=sigma).rvs(1), decimals=3)
        val_y = np.round(truncnorm((min_y-mean_y)/sigma, (max_y-mean_y)/sigma, loc=mean_y, scale=sigma).rvs(1), decimals=3)
        val_z = np.round(truncnorm((min_z-mean_z)/sigma, (max_z-mean_z)/sigma, loc=mean_z, scale=sigma).rvs(1), decimals=3)

        xyz = str(val_x[0]) + ' ' + str(val_y[0]) + ' ' + str(val_z[0])
        # print('XYZ: {}'.format(xyz))

        # Set box size XYZ
        for box in root.iter('box'):
            size = box.get("size")
            box.set('size', xyz)
            # print(box.attrib, box.tag)

        # Set color
        colors = ['Red', 'Blue', 'Green', 'Yellow', 'Purple', 'Orange']

        for mat in root.iter('material'):
            color = random.choice(colors)
            mat.text = 'Gazebo/' + color


        tree.write(output_dir + '/box_{0}.urdf'.format(i))


if __name__ == "__main__":

    N_SHAPES = 10
    generate_boxes(N_SHAPES,'boxes')