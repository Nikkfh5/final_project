#!/usr/bin/env python3

import math
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler


class RobotMover:
    def __init__(self):
        rospy.init_node("robot_motion_controller")

        self.model_name = rospy.get_param("~model_name", "robot_with_markers")
        self.radius = rospy.get_param("~radius", 0.6)
        self.angular_speed = rospy.get_param("~angular_speed", 0.35)
        self.z = rospy.get_param("~z", 0.1)
        self.rate_hz = rospy.get_param("~rate", 20.0)

        self.trajectory = rospy.get_param("~trajectory", "circle")

        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.loginfo("Waiting for Gazebo services...")
        self._set_state.wait_for_service(timeout=10.0)
        self._get_state.wait_for_service(timeout=10.0)

        rospy.loginfo("Trajectory type: %s", self.trajectory)

    # ---------- Trajectories ----------

    def circle_motion(self, t):
        x = self.radius * math.cos(self.angular_speed * t)
        y = self.radius * math.sin(self.angular_speed * t)
        yaw = self.angular_speed * t + math.pi / 2.0

        vx = -self.radius * self.angular_speed * math.sin(self.angular_speed * t)
        vy = self.radius * self.angular_speed * math.cos(self.angular_speed * t)
        wz = self.angular_speed

        return x, y, yaw, vx, vy, wz

    def zigzag_motion(self, t):
        x = 0.4 * math.sin(0.5 * t)
        y = 0.2 * ((t % 4.0) - 2.0)

        vx = 0.4 * 0.5 * math.cos(0.5 * t)
        vy = 0.2 * (1.0 if (t % 4.0) < 2.0 else -1.0)
        yaw = math.atan2(vy, vx)
        wz = 0.0

        return x, y, yaw, vx, vy, wz

    def figure8_motion(self, t):
        a = 0.5
        w = self.angular_speed

        x = a * math.sin(w * t)
        y = a * math.sin(w * t) * math.cos(w * t)

        vx = a * w * math.cos(w * t)
        vy = a * w * math.cos(2.0 * w * t)
        yaw = math.atan2(vy, vx)
        wz = 0.0

        return x, y, yaw, vx, vy, wz

    # ---------- Main loop ----------

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()

            if self.trajectory == "circle":
                x, y, yaw, vx, vy, wz = self.circle_motion(t)
            elif self.trajectory == "zigzag":
                x, y, yaw, vx, vy, wz = self.zigzag_motion(t)
            elif self.trajectory == "figure8":
                x, y, yaw, vx, vy, wz = self.figure8_motion(t)
            else:
                rospy.logwarn_throttle(
                    5.0, "Unknown trajectory '%s'", self.trajectory
                )
                x = y = yaw = vx = vy = wz = 0.0

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

            state.twist.linear.x = vx
            state.twist.linear.y = vy
            state.twist.angular.z = wz

            self._set_state(state)
            rate.sleep()


if __name__ == "__main__":
    RobotMover().run()

