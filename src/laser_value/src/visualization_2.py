#! /usr/bin/env python3

from cgi import test
from itertools import tee
from cv2 import sqrt
import numpy as np
import cv2
from visualizer import visibility
import rospy
import tf
import geometry_msgs.msg
import roslib
from std_msgs.msg import *
from nav_msgs.msg import *


def UV_energy(x, y, turtle_x, turtle_y, res):
    Pi = 1e-6 * 100  # light power [Wm^2]
    if x == turtle_x or y == turtle_y:
        UV_dt = 0
    else:
        UV_dt = (Pi / (np.square((x - turtle_x)*res) * np.square((y - turtle_y))*res))

    return UV_dt



def map_visualization(res, map, map_offset):
    occupancy = OccupancyGrid()

    occupancy.header.seq = 0
    occupancy.header.stamp.secs = 0
    occupancy.header.frame_id = "map"

    occupancy.info.resolution = res
    occupancy.info.width = len(map)
    occupancy.info.height = len(map)
    occupancy.info.origin.position.x = -map_offset
    occupancy.info.origin.position.y = -map_offset
    occupancy.info.origin.position.z = 0

    occupancy.info.origin.orientation.x = 0
    occupancy.info.origin.orientation.y = 0
    occupancy.info.origin.orientation.z = 0
    occupancy.info.origin.orientation.w = 1

    img = [0 for j in range(map.size)]

    x = 0

    for i in range(len(map) - 1, -1, -1):
        for j in range(len(map)):
            img[x] = int((map[i, j] / 255) * 100)
            x += 1

    occupancy.data = img

    return occupancy


def main():
    rospy.init_node('Visualization', anonymous=True)
    rospy.loginfo('Starting node!!!')

    pub_visibility = rospy.Publisher('/map/visibility', OccupancyGrid, queue_size=10)
    pub_sanification = rospy.Publisher('/map/sanification', OccupancyGrid, queue_size=10)

    listener = tf.TransformListener()

    fn = '/home/marco/Documents/AMR/turtlebot_project_ws/map_02.pgm'

    img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
    visibility_map = img.copy()
    sanitization_map = img.copy()

    while not rospy.is_shutdown():

        try:
            trans = listener.lookupTransform('/map', '/base_scan', rospy.Time(0))[0]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rel_pos_y = round(trans[0], 2)
        rel_pos_x = round(trans[1], 2)

        img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
        visibility_map = img.copy()

        res = 0.2
        map_offset = 10

        adder_y = 9.2
        adder_x = 10

        # Initialize the center for the turtlebot
        turtle_pixel = [0, 0]
        turtle_pixel[0] = int(round((adder_x + rel_pos_y) / res))
        turtle_pixel[1] = int(round((adder_y - rel_pos_x) / res))

        # Color the center on the map
        #image[center[1], center[0]] = 1

        for angle in range(360):
            d = 1
            pippo = 5
            while True:
                x = int(np.rint(d * np.cos(angle)))
                y = int(np.rint(d * np.sin(angle)))
                if angle <= 90 and visibility_map[(turtle_pixel[1] - y), (turtle_pixel[0] - x)] > 0:
                    #image[center[1] - y, center[0] - x] += UV_energy(x, y, center[0], center[1], res)
                    visibility_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 50
                    sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] += pippo
                elif 90 < angle <= 180 and visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] > 0:
                    #image[center[1] - y, center[0] + x] += UV_energy(x, y, center[0], center[1], res)
                    visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 50
                    sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] += pippo

                elif 180 < angle <= 270 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] > 0:
                    #image[center[1] + y, center[0] + x] += UV_energy(x, y, center[0], center[1], res)
                    visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 50
                    sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] += pippo
                elif 270 < angle <= 360 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] > 0:
                    #image[center[1] + y, center[0] - x] += UV_energy(x, y, center[0], center[1], res)
                    visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 50
                    if sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] < 80:
                        sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] += pippo

                else:
                    break
                d += 1

        visibility_occupancy = map_visualization(res, visibility_map, map_offset)
        pub_visibility.publish(visibility_occupancy)
        rospy.loginfo('Publishing map')

        sanitization_occupancy = map_visualization(res, sanitization_map, map_offset)
        pub_sanification.publish(sanitization_occupancy)
        rospy.loginfo('Publishing map')


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
