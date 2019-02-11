#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import tf.transformations as tft

def main():
    object_in_base_link = tft.quaternion_matrix([0, 0, 0.38268343, 0.92387953])
    object_in_base_link[0, 3] = 0.6
    object_in_base_link[1, 3] = -0.1
    object_in_base_link[2, 3] = 0.7
    grasp_in_object = np.identity(4)
    grasp_in_object[0, 3] = -0.1
    grasp_in_base_link = np.dot(object_in_base_link, grasp_in_object)
    quaternion = tft.quaternion_from_matrix(grasp_in_base_link)
    result = Pose(
        position = Point(grasp_in_base_link[0, 3], grasp_in_base_link[1, 3], grasp_in_base_link[2, 3]),
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    )
    print(result)

if __name__ == '__main__':
    main()