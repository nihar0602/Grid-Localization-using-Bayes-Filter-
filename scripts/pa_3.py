#!/usr/bin/env python

import roslib
import rospy
import rosbag
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from constants import *
from markers_example import display_cube_list, display_line_list


class Main:
    def __init__(self):
        self.pub_line_list = rospy.Publisher('line_list', Marker, queue_size=1)
        self.pub_cube_list = rospy.Publisher('cube_list', Marker, queue_size=1)

        self.pos_array = np.zeros((35, 35, 4))
        self.pos_array[initial_pos[0] - 1, initial_pos[1] - 1, initial_pos[2] - 1] = 1

        self.bag = rosbag.Bag(bag_read)
        self.threshold = 0.1
        self.line_points = []

        self.write_file = open(path + "/estimation.txt", "w")

    def read_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['Movements', 'Observations']):
            if topic == 'Movements':
                rot_1 = msg.rotation1
                rot_2 = msg.rotation2
                rot_1 = np.degrees((euler_from_quaternion([rot_1.x, rot_1.y, rot_1.z, rot_1.w]))[2])
                rot_2 = np.degrees((euler_from_quaternion([rot_2.x, rot_2.y, rot_2.z, rot_2.w]))[2])
                self.motion_model(rot_1, msg.translation * 100, rot_2)

            if topic == 'Observations':
                range_val = msg.range * 100
                bearingval = msg.bearing
                rot_z = np.degrees(
                    (euler_from_quaternion([bearingval.x, bearingval.y, bearingval.z, bearingval.w]))[2])
                self.sensor_model(msg.tagNum, range_val, rot_z)

        print 'read done'
        self.bag.close()

    def motion_model(self, rot_1, trans, rot_2):
        trans1 = 0
        trans2 = 0
        total_probability = 0
        temp1 = 0
        temp2 = 0
        temp3 = 0
        temp4 = 0
        while temp2 < max_iter:
            if self.pos_array[temp2, temp3, temp4] >= self.threshold:
                trans1x = 20 * trans1 + 10
                trans1y = 20 * trans2 + 10
                rot_11 = (-180) + (temp1 * 90) + 45
                trans2x = 20 * temp2 + 10
                trans2y = 20 * temp3 + 10
                rot_21 = (-180) + (temp4 * 90) + 45
                transt = np.sqrt((trans1x - trans2x) ** 2 + (trans1y - trans2y) ** 2)
                rot_1t = np.degrees(np.arctan2(trans1y - trans2y, trans1x - trans2x)) - rot_21
                rot_2t = rot_11 - np.degrees(np.arctan2(trans1y - trans2y, trans1x - trans2x))
                a = np.degrees(np.arctan2(trans1y - trans2y, trans1x - trans2x))
                rot_1t = rotation_adjust(rot_1t)
                rot_2t = rotation_adjust(rot_2t)
                rot_1p, transp = gaussian(rot_1t, rot_1, transt, trans)
                rot_2p, transp2 = gaussian(rot_2t, rot_2, transt, trans)
                incr = get_change(temp2, temp3, temp4, transp, rot_1p, self.pos_array)
                self.pos_array[trans1, trans2, temp1] += incr * rot_2p
                total_probability += incr * rot_2p

            temp1, temp2, temp3, temp4, trans1, trans2 = get_next_change(temp1, temp2, temp3, temp4, trans1, trans2, self.pos_array, self.threshold)

        self.pos_array /= total_probability
        mx = np.argmax(self.pos_array)
        
        angle = mx % self.pos_array.shape[2]
        angle = (-180) + (angle * 90) - 45.0
        mx /= self.pos_array.shape[2]

        y = mx % self.pos_array.shape[1]
        mx /= self.pos_array.shape[1]

        x = mx % self.pos_array.shape[0]
        
        self.line_points.append((x, y))
        self.write_file.write('P: ({}, {}, {})\n'.format((20 * x + 10) / 100.0, (20 * y + 10) / 100.0, angle))

    def sensor_model(self, tagnum, trans, rot_z):
        trans1 = 0
        trans2 = 0
        trans3 = 0
        temp = 0
        while trans1 < max_iter:
            translational_measure = np.sqrt(((10 + 20 * trans1) - cube_points[tagnum + 1][0]) ** 2 + (
                    (10 + 20 * trans2) - cube_points[tagnum + 1][1]) ** 2)
            rotational_measure = rotation_adjust(
                np.degrees(np.arctan2(cube_points[tagnum + 1][0] - (10 + 20 * trans2),
                                      cube_points[tagnum + 1][1] - (
                                              10 + 20 * trans1))) - (
                        -135 + trans3 * 90))
            a = np.degrees(np.arctan2(cube_points[tagnum + 1][0] - (10 + 20 * trans2),
                                      cube_points[tagnum + 1][1] - (
                                              10 + 20 * trans1)))
            rotation_pos, translational_pos = gaussian(rotational_measure, rot_z, translational_measure, trans)
            temp += get_change(trans1, trans2, trans3, translational_pos, rotation_pos, self.pos_array)
            change = translational_pos * rotation_pos
            self.pos_array[trans1, trans2, trans3] *= change
            trans3 += 1
            if trans3 == 4:
                trans3 = 0
                trans2 += 1
            if trans2 == max_iter:
                trans3, trans2 = 0, 0
                trans1 += 1

        self.pos_array /= temp

        mx = np.argmax(self.pos_array)
        angle = mx % self.pos_array.shape[2]
        angle = (-180) + (angle * 90) - 45.0
        mx /= self.pos_array.shape[2]

        y = mx % self.pos_array.shape[1]
        mx /= self.pos_array.shape[1]

        x = mx % self.pos_array.shape[0]

        self.line_points.append((x, y))
        self.write_file.write('U: ({}, {}, {})\n'.format((20 * x + 10) / 100.0, (20 * y + 10) / 100.0, angle))
        display_line_list(self.line_points, self.pub_line_list)
        display_cube_list(cube_points, self.pub_cube_list)

    def run(self):
        self.read_bag()

        while not rospy.is_shutdown():
            display_cube_list(cube_points, self.pub_cube_list)


if __name__ == '__main__':
    try:
        rospy.init_node('pa_3')
        Main().run()
    except rospy.ROSInterruptException:
        pass
