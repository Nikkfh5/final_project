#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("cmd_vel_circle")
    radius = rospy.get_param("~radius", 0.6)
    angular_speed = rospy.get_param("~angular_speed", 0.35)  # rad/s
    rate_hz = rospy.get_param("~rate", 20.0)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(rate_hz)

    lin_speed = radius * angular_speed  # v = r * omega
    rospy.loginfo("Publishing circular cmd_vel: radius=%.2f m, ang=%.2f rad/s, lin=%.2f m/s", radius, angular_speed, lin_speed)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = lin_speed
        msg.angular.z = angular_speed
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
