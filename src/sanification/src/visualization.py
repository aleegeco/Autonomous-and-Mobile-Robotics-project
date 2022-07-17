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
    map = img.copy()

    cv2.imshow('image', img)
    cv2.waitKey(0)







if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

