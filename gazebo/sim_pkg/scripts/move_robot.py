#!/usr/bin/env python3

import math
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler


class RobotMover:
    def __init__(self):
        rospy.init_node("robot_motion_controller")
        self.model_name = rospy.get_param("~model_name", "robot_with_markers")
        self.radius = rospy.get_param("~radius", 0.6)
        self.angular_speed = rospy.get_param("~angular_speed", 0.35)  # rad/s
        self.z = rospy.get_param("~z", 0.1)
        self.rate_hz = rospy.get_param("~rate", 20.0)

        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("Waiting for /gazebo/set_model_state service...")
        self._set_state.wait_for_service(timeout=10.0)
        rospy.loginfo("Connected to /gazebo/set_model_state")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()
            x = self.radius * math.cos(self.angular_speed * t)
            y = self.radius * math.sin(self.angular_speed * t)

            # Face along the tangent of the circle
            yaw = self.angular_speed * t + math.pi / 2.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

            state = ModelState()
            state.model_name = self.model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = self.z
            state.pose.orientation.x = qx
            state.pose.orientation.y = qy
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw

            # Optional twist to make RViz arrows look smoother
            vx = -self.radius * self.angular_speed * math.sin(self.angular_speed * t)
            vy = self.radius * self.angular_speed * math.cos(self.angular_speed * t)
            state.twist.linear.x = vx
            state.twist.linear.y = vy
            state.twist.angular.z = self.angular_speed

            try:
                self._set_state(state)
            except rospy.ServiceException as e:
                rospy.logwarn_throttle(5.0, "Failed to set model state: %s", str(e))

            rate.sleep()


if __name__ == "__main__":
    mover = RobotMover()
    mover.run()
