#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Illuminance

def illuminance_test():

    pub = rospy.Publisher('illuminance_topic', Illuminance, queue_size=10)

    rospy.loginfo('Starting node')

    rospy.init_node('illumincance_pub', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz	

    while not rospy.is_shutdown():
        test = Illuminance()

        test.header.frame_id = 'base_scan'
        #test.header.stamp  

        test.illuminance =   
        test.variance = 0

        rospy.loginfo('Sending illluminance')

        pub.publish(test)
        rate.sleep()






if __name__ == "__main__":
    try:
        illuminance_test()
    except rospy.ROSInterruptException:
        pass