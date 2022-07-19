#! /usr/bin/env python3

import numpy as np
import cv2
import os
import rospy
import tf
import roslib
from std_msgs.msg import *
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid

map = 'map_02.pgm'
map_dir = os.path.dirname(os.path.realpath(map))
map_path = map_dir + '/' + map

class sanitizer_node():
    def __init__(self):
        self.occupancy = OccupancyGrid(header=rospy.Header()) # initialize map object

        map_dir = os.path.dirname(os.path.realpath(map))
        map = 'map_02.pgm'
        map_path = map_dir + '/' + map
        self.map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        self.res = 0.2 # map resolution
        self.map_offset = 10 # difference between real map and discretized one (used to match the two better)

    def map_occupancy(self, data):
        self.occupancy.header.seq = data.header.seq
        self.occupancy.header.stamp = rospy.Time.now()
        self.occupancy.header.frame_id = data.header.frame_id

        self.occupancy.info.origin.position.x = -self.map_offset - self.res
        self.occupancy.info.origin.position.y = -self.map_offset - self.res

    def UV_energy(self, data):
        Pl = 1e-4 # light power [Wm^2]







