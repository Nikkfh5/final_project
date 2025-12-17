#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    print("Got frame:", img.shape)
    cv2.imshow("camera", img)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("camera_debug")
    rospy.Subscriber("/camera/image_raw", Image, cb, queue_size=1)
    rospy.spin()
