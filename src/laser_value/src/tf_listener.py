#!/usr/bin/env python3

import roslib

import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans)
        print(rot)

        rate.sleep()
