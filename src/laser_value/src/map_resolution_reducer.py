#! /usr/bin/env python3

from cgi import test
from itertools import tee
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import *
from nav_msgs.msg import *
import tf
import geometry_msgs.msg
import roslib


def map_visualization():
    test = OccupancyGrid()

    test.header.seq = 0
    test.header.stamp.secs = 0
    test.header.frame_id = "map"

    test.info.resolution = 0.2
    test.info.width = 96
    test.info.height = 96
    test.info.origin.position.x = -10
    test.info.origin.position.y = -10
    test.info.origin.position.z = 0

    test.info.origin.orientation.x = 0
    test.info.origin.orientation.y = 0
    test.info.origin.orientation.z = 0
    test.info.origin.orientation.w = 1

    map = cv.imread('/home/marco/Documents/AMR/turtlebot_project_ws/map_02.pgm', 0)

    img = [0 for j in range(map.size)]

    x = 0

    for i in range(len(map) - 1, -1, -1):
        for j in range(len(map)):
            img[x] = int((map[i, j] / 255) * 100)
            x += 1

    test.data = img

    return test


def occupancy_test():
    # creating a node
    rospy.init_node('Occupancy_node', anonymous=True)
    # defining a publisher
    pub = rospy.Publisher('/map/test', OccupancyGrid, queue_size=10)
    # creating listener for tf
    listener = tf.TransformListener()
    rospy.loginfo('Starting node')

    rate = rospy.Rate(5)  # 10hz

    test = map_visualization()

    while not rospy.is_shutdown():
        rospy.loginfo('Publish map')
        pub.publish(test)

        rate.sleep()


if __name__ == "__main__":
    try:
        occupancy_test()
    except rospy.ROSInterruptException:
        pass
