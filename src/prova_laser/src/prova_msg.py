#! /usr/bin/env python3

import rospy
import std_msgs.msg
from sensor_msgs.msg import LaserScan

def callback(msg):
    rospy.loginfo(msg)

def prova(msg):
    rospy.init_node("prova_laser")
    sub = rospy.Subscriber("/scan", LaserScan,callback)
    pub = rospy.Publisher("/rviz", LaserScan)

    laser_msg = LaserScan()
    messaggio = []

    for i in range(360):
        messaggio.append(msg.ranges[i])
        if messaggio[i] == float("inf"):
            messaggio[i] = 3.5

    laser_msg.header = msg.header
    laser_msg.range_max = msg.range_max
    laser_msg.range_min = msg.range_min
    laser_msg.angle_min = msg.angle_min
    laser_msg.angle_max = msg.angle_max
    laser_msg.angle_increment = msg.angle_increment
    laser_msg.time_increment = msg.time_increment
    laser_msg.scan_time = msg.scan_time
    laser_msg.entities = msg.entities
    laser_msg.ranges = tuple(messaggio)

    pub.publish(laser_msg)
    return None

if __name__ == "__main__":
    try:
        prova(msg)

