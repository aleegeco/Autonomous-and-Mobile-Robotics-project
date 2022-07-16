import rospy
from geometry_msgs.msg import Twist
import numpy as np

def init_localization():

    rospy.init_node("localization_start")
    vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()

    angle = (15)*360
    speed = 20

    angular_speed = speed * 2 * np.pi / 360
    relative_angle = angle * 2 * np.pi /360

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    vel_msg.angular.z = abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1 - t0)

    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)
    done = 1
    rospy.loginfo("Initialization done")

    return done


if __name__ == "__main__":
    try:
        while True:
            done = init_localization()
            if done == 1:
                break

    except rospy.ROSInterruptException:
        pass





