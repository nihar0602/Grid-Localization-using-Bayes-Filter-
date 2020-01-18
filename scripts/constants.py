#!/usr/bin/env python
import numpy as np
import rospkg

rospack = rospkg.RosPack()
path = rospack.get_path('ros_pa3')
bag_read = path + '/grid.bag'

initial_pos = [12, 28, 3]

cube_points = [
    (0, 0),
    (125, 525),
    (125, 325),
    (125, 125),
    (425, 125),
    (425, 325),
    (425, 525),
]
max_iter = 35


def rotation_adjust(rot_z):
    if rot_z < -180:
        rot_z += 360
    elif rot_z > 180:
        rot_z -= 360
    return rot_z


def gaussian(rotational_measure, rot_z, translational_measure, trans):
    rotation_pos = 0.0088653 * np.power(np.e, -1.0 * (((rotational_measure - rot_z) ** 2) / (2.0 * 45 ** 2)))
    translational_pos = 0.0398942 * np.power(np.e, -1.0 * (((translational_measure - trans) ** 2) / (2.0 * 10 ** 2)))
    return rotation_pos, translational_pos


def get_change(trans1, trans2, trans3, translational_pos, rotation_pos, pos_array):
    change = pos_array[trans1, trans2, trans3]
    change *= rotation_pos
    change *= translational_pos
    return change


def get_next_change(rt1, rt2, rt3, rt4, trans1, trans2, pos_array, threshold):
    if pos_array[rt2, rt3, rt4] > threshold:
        rt1 += 1
        if rt1 == 4:
            rt1 = 0
            trans2 += 1
        if trans2 == max_iter:
            rt1 = 0
            trans2 = 0
            trans1 += 1
        if trans1 == max_iter:
            rt1 = 0
            trans1 = 0
            trans2 = 0
            rt4 += 1
        if rt4 == 4:
            rt1 = 0
            rt4 = 0
            trans1 = 0
            trans2 = 0
            rt3 += 1
        if rt3 == max_iter:
            rt1 = 0
            rt3 = 0
            rt4 = 0
            trans1 = 0
            trans2 = 0
            rt2 += 1
    else:
        rt4 += 1
        if rt4 == 4:
            rt1 = 0
            rt4 = 0
            trans1 = 0
            trans2 = 0
            rt3 += 1
        if rt3 == max_iter:
            rt1 = 0
            rt3 = 0
            rt4 = 0
            trans1 = 0
            trans2 = 0
            rt2 += 1

    return rt1, rt2, rt3, rt4, trans1, trans2