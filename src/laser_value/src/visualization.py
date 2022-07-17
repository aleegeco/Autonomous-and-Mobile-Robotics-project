#! /usr/bin/env python3

from cgi import test
from itertools import tee
from cv2 import sqrt
import numpy as np
import cv2
from matplotlib import pyplot as plt
from visualizer import visibility
import rospy
import tf
import geometry_msgs.msg
import roslib
from std_msgs.msg import *
from nav_msgs.msg import *


def map_visualization(res, map, map_offset):
    test = OccupancyGrid()

    test.header.seq = 0
    test.header.stamp.secs = 0
    test.header.frame_id = "map"

    test.info.resolution = res
    test.info.width = len(map)
    test.info.height = len(map)
    test.info.origin.position.x = -map_offset
    test.info.origin.position.y = -map_offset
    test.info.origin.position.z = 0

    test.info.origin.orientation.x = 0
    test.info.origin.orientation.y = 0
    test.info.origin.orientation.z = 0
    test.info.origin.orientation.w = 1

    img = [0 for j in range(map.size)]

    x = 0

    for i in range(len(map) - 1, -1, -1):
        for j in range(len(map)):
            img[x] = int((map[i, j] / 255) * 100)
            x += 1

    test.data = img

    return test


def function():
    rospy.init_node('Visualization', anonymous=True)
    rospy.loginfo('Starting node!!!')

    pub = rospy.Publisher('/map/test', OccupancyGrid, queue_size=10)

    listener = tf.TransformListener()
    while not rospy.is_shutdown():

        try:
            trans = listener.lookupTransform('/map', '/base_scan', rospy.Time(0))[0]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rel_pos_y = round(trans[0], 2)
        rel_pos_x = round(trans[1], 2)

        fn = '/home/marco/Documents/AMR/turtlebot_project_ws/map_02.pgm'

        img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
        image = img.copy()

        res = 0.2
        map_offset = 10

        adder_y = 9.2
        adder_x = 10

        centre = [0, 0]
        centre[0] = int(round((adder_x + rel_pos_y) / res))
        centre[1] = int(round((adder_y - rel_pos_x) / res))

        image[centre[1], centre[0]] = 1

        image[0, 0] = 80
        image[2, 0] = 70

        for angle in range(360):
            d = 1
            while True:
                x = np.rint(d * np.cos(angle))
                y = np.rint(d * np.sin(angle))
                if angle <= 90 and image[(centre[1] - y), (centre[0] - x)] > 0:
                    image[centre[1] - y, centre[0] - x] = 80
                elif 90 < angle <= 180 and image[centre[1] - y, centre[0] + x] > 0:
                    image[centre[1] - y, centre[0] + x] = 80
                elif 180 < angle <= 270 and image[centre[1] + y, centre[0] + x] > 0:
                    image[centre[1] + y, centre[0] + x] = 80
                elif 270 < angle <= 360 and image[centre[1] + y, centre[0] - x] > 0:
                    image[centre[1] + y, centre[0] - x] = 80
                else:
                    break
                d += 1

        test = map_visualization(res, image, map_offset)

        pub.publish(test)
        rospy.loginfo('Publishing map')


if __name__ == "__main__":
    try:
        function()
    except rospy.ROSInterruptException:
        pass
