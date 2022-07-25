#! /usr/bin/env python3
import random

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
import matplotlib.pyplot as plt

map = 'map_02.pgm'  # name of the map file
rooms_txt = 'rooms_to_sanitize.txt'  # txt file with the predefined rooms

# definition of the rooms intervala
rooms_dict = {'kitchen': [[26, 50], [20, 45]],
              'living room': [[26, 74], [46, 63]],
              'living room 2': [[63, 87], [20, 45]],
              'corridor': [[15, 75], [65, 76]],
              'bathroom': [[76, 87], [46, 70]],
              'closet': [[51, 61], [20, 40]],
              'closet 2': [[15, 25], [42, 64]]}


def UV_energy(x, y, turtle_x, turtle_y, res):
    """
        input:
        - x : x pixel coordinate of the irradiated pixel
        - y : y pixel coordinate of the irradiated pixel
        - turtle_x : x pixel coordinate of the robot
        - turtle_y : y pixel coordinate of the robot
        - res : map resolution

        output:
        - UV_dt : Irradiation energy
    """

    Pi = 1e-4  # light power [Wm^2]
    if x == turtle_x and y == turtle_y:
        UV_dt = 0
    else:
        UV_dt = Pi * 2e-3 / (np.square((x - turtle_x) * res) + np.square((y - turtle_y) * res))
    return UV_dt


def transform_map_meters_to_grid_cells(coordinates, res, offset):
    """
       input:
       - coordinates : meters coordinates to be transformed
       - offset : map offset
       - res : map resolution

       output:
       - cell : grid cell coordinates
    """
    x_cord = (coordinates[0] - float(offset)) / res
    y_cord = (coordinates[1] - float(offset)) / res
    cell = (int(x_cord), int(y_cord), 0)
    return cell


def transform_pixel_to_map_meters(coordinates, res, offset):
    """
       input:
       - coordinates : pixel coordinates to be transformed
       - offset : map offset
       - res : map resolution

       output:
       - cell : meters coordinates
    """
    x_cord = float(coordinates[0]) * res + float(offset[0])
    y_cord = float(coordinates[1]) * res + float(offset[1])
    x_cord = round(x_cord, 3)
    y_cord = round(-y_cord, 3)
    point = (x_cord, y_cord)
    return point


def map_occupancy(res, map, map_offset):
    """
       input:
       - map_offset : map offset
       - map : image map
       - res : map resolution

       output:
       - occupancy : occupancy grid message
    """
    occupancy = OccupancyGrid(header=rospy.Header())

    occupancy.header.seq = 0
    occupancy.header.stamp.secs = 0
    occupancy.header.frame_id = "map"

    occupancy.info.resolution = res
    occupancy.info.width = len(map)
    occupancy.info.height = len(map)

    occupancy.info.origin.position.x = -map_offset - 2 * res
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
    return occupancy


def main():
    res = 0.2  # map resolution
    map_offset = 10  # map offset

    adder_y = 9.0
    adder_x = 10.4

    tresh = 0.1  # threshold value

    # map directory
    dir_map = os.path.dirname(os.path.realpath(map))
    path_map = dir_map + '/' + map

    # txt room list directory
    dir_rooms = os.path.dirname(os.path.realpath(rooms_txt))
    path_room = dir_rooms + '/' + rooms_txt
    rooms_file = open(path_room, 'r')

    rooms_to_sanitize = [rooms_dict[line.strip('\n')] for line in rooms_file]  # save the room to sanitize from the txt
    rooms_file.close()

    # defining the publishers
    pub_visibility = rospy.Publisher('/map/visibility', OccupancyGrid, queue_size=25)
    pub_sanification = rospy.Publisher('/map/sanitization', OccupancyGrid, queue_size=25)
    pub_goals = rospy.Publisher('san_goal', Point, queue_size=30)

    # subscribing to the tf topic
    listener_tf = tf.TransformListener()

    img = cv2.imread(path_map, cv2.IMREAD_GRAYSCALE)
    sanitization_map = img.copy()

    # initialization of the matrices
    UV_store = np.zeros((len(img), len(img)))
    Map_matrix = np.zeros_like(UV_store)

    # set to -1 the part of the matrix with walls and outer areas
    for i in range(Map_matrix.shape[0]):
        for j in range(Map_matrix.shape[1]):
            if img[i, j] == 0 or img[i, j] == 205:
                Map_matrix[i, j] = -1

    wall_offset = 3  # offset from the wall for the control

    for room in range(len(rooms_to_sanitize)):
        # compute the center pixel of the room and post it as a goal
        rospy.sleep(0.2)
        room_center_pixel = [0, 0]
        room_center_pixel[0] = rooms_to_sanitize[room][0][0] + (
                rooms_to_sanitize[room][0][1] - rooms_to_sanitize[room][0][0]) / 2
        room_center_pixel[1] = rooms_to_sanitize[room][1][0] + (
                rooms_to_sanitize[room][1][1] - rooms_to_sanitize[room][1][0]) / 2

        temp = transform_pixel_to_map_meters(room_center_pixel, res, [-adder_x, -adder_y])
        point = Point()
        point.x = temp[0]
        point.y = temp[1]
        pub_goals.publish(point)
        rospy.sleep(0.2)

        # compute the corner of the room and post it in series as goals
        room_corners = [[0, 0], [0, 0], [0, 0], [0, 0]]
        room_corners[0][0] = rooms_to_sanitize[room][0][0] + wall_offset
        room_corners[0][1] = rooms_to_sanitize[room][1][0] + wall_offset
        room_corners[1][0] = rooms_to_sanitize[room][0][1] - wall_offset
        room_corners[1][1] = rooms_to_sanitize[room][1][0] + wall_offset
        room_corners[2][0] = rooms_to_sanitize[room][0][1] - wall_offset
        room_corners[2][1] = rooms_to_sanitize[room][1][1] - wall_offset
        room_corners[3][0] = rooms_to_sanitize[room][0][0] + wall_offset
        room_corners[3][1] = rooms_to_sanitize[room][1][1] - wall_offset

        for i in range(len(room_corners)):
            temp = transform_pixel_to_map_meters(room_corners[i], res, [-adder_x, -adder_y])
            point = Point()
            point.x = temp[0]
            point.y = temp[1]
            pub_goals.publish(point)
            rospy.sleep(0.2)

        while not rospy.is_shutdown():
            try:
                trans_pos = listener_tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))[0]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # position of the robot
            rel_pos_x = round(trans_pos[0], 2)
            rel_pos_y = round(trans_pos[1], 2)

            img = cv2.imread(path_map, cv2.IMREAD_GRAYSCALE)
            visibility_map = img.copy()

            # define the pixel of the robot
            turtle_pixel = [0, 0]
            turtle_pixel[0] = int(round((adder_x + rel_pos_x) / res))
            turtle_pixel[1] = int(round((adder_y - rel_pos_y) / res))

            for angle in range(360 + 1):
                d = 1  # distanza dal robot
                while True:
                    x = int(np.rint(d * np.cos(angle)))  # x distance from a pixel
                    y = int(np.rint(d * np.sin(angle)))  # y distance from a pixel

                    if angle <= 90 and visibility_map[(turtle_pixel[1] - y), (turtle_pixel[0] - x)] > 0:
                        visibility_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 50
                        if sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] < 255:
                            UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] += \
                                UV_energy(turtle_pixel[0] - x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1],
                                          res)
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 100
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (2 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 125
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (3 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 150
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= (4 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 200
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] - x] >= 1e-3:
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] - x] = 255

                    elif 90 < angle <= 180 and visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] > 0:
                        visibility_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 50
                        if sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] < 255:
                            UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] += \
                                UV_energy(turtle_pixel[0] + x, turtle_pixel[1] - y, turtle_pixel[0], turtle_pixel[1],
                                          res)
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 100
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (2 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 125
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (3 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 150
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= (4 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 200
                            if UV_store[turtle_pixel[1] - y, turtle_pixel[0] + x] >= 1e-3:
                                sanitization_map[turtle_pixel[1] - y, turtle_pixel[0] + x] = 255

                    elif 180 < angle <= 270 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] > 0:
                        visibility_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 50
                        if sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] < 255:
                            UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] += \
                                UV_energy(turtle_pixel[0] + x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1],
                                          res)
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 100
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (2 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 125
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (3 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 150
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= (3 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 200
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] + x] >= 1e-3:
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] + x] = 255

                    elif 270 < angle <= 360 and visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] > 0:
                        visibility_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 50
                        if sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] < 255:
                            UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] += \
                                UV_energy(turtle_pixel[0] - x, turtle_pixel[1] + y, turtle_pixel[0], turtle_pixel[1],
                                          res)
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 100
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (2 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 125
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (3 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 150
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= (4 * 1e-3 / 5):
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 200
                            if UV_store[turtle_pixel[1] + y, turtle_pixel[0] - x] >= 1e-3:
                                sanitization_map[turtle_pixel[1] + y, turtle_pixel[0] - x] = 255
                    else:
                        break
                    d += 1

            # defining the sanitized parts of the map on the Map_matrix based on the value of the UV_store
            for i in range(Map_matrix.shape[0]):
                for j in range(Map_matrix.shape[1]):
                    if UV_store[i, j] >= 1e-3:
                        Map_matrix[i, j] = 1
            # publishing the visibility occupancy grid
            visibility_occupancy = map_occupancy(res, visibility_map, map_offset)
            pub_visibility.publish(visibility_occupancy)
            # publishing the sanitization occupancy grid
            sanitization_occupancy = map_occupancy(res, sanitization_map, map_offset)
            pub_sanification.publish(sanitization_occupancy)

            if abs(rel_pos_y - point.y) < tresh and abs(rel_pos_x - point.x) < tresh: # control if the last goal has been reached
                # ranges of the actual room
                range_x_min = rooms_to_sanitize[room][0][0]
                range_x_max = rooms_to_sanitize[room][0][1]
                range_y_min = rooms_to_sanitize[room][1][0]
                range_y_max = rooms_to_sanitize[room][1][1]

                pixel_to_sanitize = []
                # fll a list with the necessary pixel where sanitization is needed
                for i in range(range_x_min, range_x_max):
                    for j in range(range_y_min, range_y_max):
                        if Map_matrix[j, i] == -1:
                            pass
                        elif Map_matrix[j, i] != 1:
                            pixel_to_sanitize.append([i, j])

                if not pixel_to_sanitize:  # empty list when there are no more point to sanitize in the room
                    print('Switching room')
                    break
                else:
                    print('Need more sanitization')
                    # pick a random point to sanitize
                    objective_pixel = random.choice(pixel_to_sanitize)
                    if abs(objective_pixel[0] - range_x_min) <= wall_offset:
                        objective_pixel[0] += wall_offset
                    elif abs(objective_pixel[0] - range_x_max) <= wall_offset:
                        objective_pixel[0] -= wall_offset
                    if abs(objective_pixel[1] - range_y_min) <= wall_offset:
                        objective_pixel[1] += wall_offset
                    elif abs(objective_pixel[1] - range_y_max) <= wall_offset:
                        objective_pixel[1] -= wall_offset

                    objective_point = transform_pixel_to_map_meters(objective_pixel, res, [-adder_x, -adder_y])
                    # publish the new goal to sanitize
                    point = Point()
                    point.x = objective_point[0]
                    point.y = objective_point[1]
                    pub_goals.publish(point)
                    rospy.sleep(0.2)


if __name__ == "__main__":
    try:
        rospy.init_node('Visualization', anonymous=True)
        rospy.loginfo('Starting node!!!')
        main()
    except rospy.ROSInterruptException:
        pass
