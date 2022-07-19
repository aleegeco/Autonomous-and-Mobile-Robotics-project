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
def UV_energy(x, y, turtle_x, turtle_y, res):
    Pi = 1e-4 # light power [Wm^2]
    if x == turtle_x or y == turtle_y:
         UV_dt = 0
    else:
        UV_dt = (Pi / (np.square((x - turtle_x)*res) * np.square((y - turtle_y))*res))
    return UV_dt

def transform_map_meters_to_grid_cells(coordinates, map_info):
    x_cord = (coordinates[0] - float(map_info.origin.position.x)) / map_info.resolution
    y_cord = (coordinates[1] - float(map_info.origin.position.y)) / map_info.resolution
    cell = (int(x_cord), int(y_cord), 0)
    return cell


def transform_grid_cells_to_map_meters(coordinates, map_info):
    x_cord = float(coordinates[0] + 0.5) * map_info.resolution + float(map_info.origin.position.x)
    #y_cord = float(cordinate[1] + .5) * map_info.resolution + float(map_info.origin.position.y)
    y_cord = float(coordinates[1] + 0.5) * map_info.resolution + float(map_info.origin.position.y)
    x_cord = round(x_cord, 3)
    y_cord = round(-y_cord, 3)
    cell = (x_cord, y_cord)
    return cell


def map_occupancy(res, map, map_offset, robot_pixel = 0):
    occupancy = OccupancyGrid(header=rospy.Header())
    occupancy_cell = GridCells(header=rospy.Header())

    occupancy.header.seq = 0
    occupancy.header.stamp.secs = 0
    occupancy.header.frame_id = "map"

    occupancy_cell.header.frame_id = 'map'
    occupancy_cell.cell_width = 0.2
    occupancy_cell.cell_height = 0.2
    point = Point()
    if robot_pixel != 0:
        temp = transform_grid_cells_to_map_meters(robot_pixel, occupancy.info)
        point.x = temp[0]
        point.y = temp[1]
        occupancy_cell.cells.append(point)

    occupancy.info.resolution = res
    occupancy.info.width = len(map)
    occupancy.info.height = len(map)

    occupancy.info.origin.position.x = -map_offset - 2*res
    occupancy.info.origin.position.y = -map_offset - res
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
    return occupancy, occupancy_cell


def main():
    rospy.init_node('Visualization', anonymous=True)
    rospy.loginfo('Starting node!!!')

    pub_visibility = rospy.Publisher('/map/visibility', OccupancyGrid, queue_size=25)
    pub_sanification = rospy.Publisher('/map/sanification', OccupancyGrid, queue_size=25)
    pub_cells = rospy.Publisher('/my_grid_cells', GridCells, queue_size=25)

    listener_tf = tf.TransformListener()

    # directory
    dir = os.path.dirname(os.path.realpath(map))
    fn = dir + '/' + map

    img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
    sanitization_map = img.copy()

    UV_store = np.zeros((len(img), len(img)))
    while not rospy.is_shutdown():
        try:
            trans_pos = listener_tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))[0]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rel_pos_y = round(trans_pos[0], 2)
        rel_pos_x = round(trans_pos[1], 2)

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


        for angle in range(360+1):
            d = 1  # distanza dal robot
            while True:
                x = int(np.rint(d * np.cos(angle)))  # x distance from a pixel
                y = int(np.rint(d * np.sin(angle)))  # y distance from a pixel

                if angle <= 90 and visibility_map[(turtle_pixel[1] - y), (turtle_pixel[0] - x)] > 0:
                    visibility_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 50
                    if  sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] < 255:
                        UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] += UV_energy(turtle_pixel[0] - x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1], res)
                        #sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] += np.rint(k * UV_energy(turtle_pixel[0] - x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1], res))
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (1e-3/3):
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 100
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (2*1e-3/3):
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 200
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= 1e-3:
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 255

                elif 90 < angle <= 180 and visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] > 0:
                    visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 50
                    if sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] < 255:
                        UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] += UV_energy(turtle_pixel[0] + x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1], res)
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (1e-3/3):
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 100
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (2*1e-3/3):
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 200
                        if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= 1e-3:
                            sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 255
                        #sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] += np.rint(k * UV_energy(turtle_pixel[0] + x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1], res))

                elif 180 < angle <= 270 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] > 0:
                    visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 50
                    if sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] < 255:
                        UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] += UV_energy(turtle_pixel[0] + x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1], res)
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (1e-3/3):
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 100
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (2*1e-3 / 3):
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 200
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= 1e-3:
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 255
                        #sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] += np.rint(k * UV_energy(turtle_pixel[0] + x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1], res))

                elif 270 < angle <= 360 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] > 0:
                    visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 50
                    if  sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] < 255:
                        UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] += UV_energy(turtle_pixel[0] - x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1], res)
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (1e-3/3):
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 100
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (2*1e-3/3):
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 200
                        if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= 1e-3:
                            sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 255
                        #sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] += np.rint(k * UV_energy(turtle_pixel[0] - x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1], res))
                else:
                    break
                d += 1

        visibility_occupancy, _ = map_occupancy(res, visibility_map, map_offset)
        pub_visibility.publish(visibility_occupancy)
        #rospy.loginfo('Publishing visibility map')

        sanitization_occupancy, _ = map_occupancy(res, sanitization_map, map_offset)
        pub_sanification.publish(sanitization_occupancy)
        #rospy.loginfo('Publishing sanitization map')


        _, cell_occupancy = map_occupancy(res, sanitization_map, map_offset, turtle_pixel )
        pub_cells.publish(cell_occupancy)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
