#!/usr/bin/env python3

from cgi import test
from itertools import tee
import os
import cv2
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import *
from nav_msgs.msg import *
import tf
import geometry_msgs.msg
import roslib


def map_visualization():
    occupancy_grid = OccupancyGrid()

    occupancy_grid.header.seq = 0
    occupancy_grid.header.stamp.secs = 0
    occupancy_grid.header.frame_id = "map"

    occupancy_grid.info.resolution = 0.2
    occupancy_grid.info.width = 384
    occupancy_grid.info.height = 384
    occupancy_grid.info.origin.position.x = -10
    test.info.origin.position.y = -10
    test.info.origin.position.z = 0

    test.info.origin.orientation.x = 0
    test.info.origin.orientation.y = 0
    test.info.origin.orientation.z = 0
    test.info.origin.orientation.w = 1

    map = cv.imread('/home/marco/Documents/AMR/turtlebot_project_ws/src/second_launch/big_house_map.pgm', 0)
    img = [0 for j in range(map.size)]

    x = 0

    for i in range(len(map) - 1, -1, -1):
        for j in range(len(map)):
            img[x] = int((map[i, j] / 255) * 100)
            x += 1

    test.data = img

    return test
def main():
    # create the node
    rospy.init_node('Visualization', anonymous=True)
    rospy.loginfo('Starting node !!!')
    # define the publisher
    pub = rospy.Publisher('map/visualization', OccupancyGrid, queue_size=10)
    # define the listener
    listener = tf.TransformListener()

    try:
        # read the robot position wrt the map frame positon
        turtle_position = listener.lookupTransform('/map', '/base_scam', rospy.Time(0))[0]
        turtle_position = round(turtle_position, 2)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo('Cannot read robot position')
        pass

    # define the directory of the image
    map_file = 'map_02.pgm'
    dir = os.path.dirname(os.path.realpath(map_file))
    dir = dir + '/' + map_file
    print(dir)
    # open the image
    img = cv2.imread(dir, cv2.IMREAD_GRAYSCALE)
    image = img.copy()

    res = 0.2
    map_offet = 10

    dder_y = 9.2
    adder_x = 10

    center = [0, 0]
    center[0] = int(round((adder_x + turtle_position[0]) / res))
    center[1] = int(round((adder_y - turtle_position[1]) / res))






if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

