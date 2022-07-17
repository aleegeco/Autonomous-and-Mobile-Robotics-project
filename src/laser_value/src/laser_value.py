#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    # print(len(msg.ranges))

    print(len(msg.ranges))
    messaggio = []

    for i in range(360):
        messaggio.append(msg.ranges[i])
        if messaggio[i] == float('inf'):
            messaggio[i] = 3.5

    print(messaggio)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)

rospy.spin()
