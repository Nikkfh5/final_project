#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Tracker node for robot position estimation from external camera.

This node:
1. Subscribes to /camera/image_raw and /camera/camera_info
2. Detects red and blue markers using OpenCV
3. Computes 3D positions using camera intrinsics + TF transform + ray-plane intersection
4. Publishes robot pose and path
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped


class TrackerNode:
    def __init__(self):
        rospy.init_node('tracker_node', anonymous=True)
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Load parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.ground_z = rospy.get_param('~ground_z', 0.0)
        
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.pose_topic = rospy.get_param('~pose_topic', '/robot_pose_external')
        self.path_topic = rospy.get_param('~path_topic', '/robot_path_external')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/debug/image')
        
        # HSV thresholds
        self.red_lower = np.array(rospy.get_param('~red_hsv/lower', [0, 100, 100]))
        self.red_upper = np.array(rospy.get_param('~red_hsv/upper', [10, 255, 255]))
        self.blue_lower = np.array(rospy.get_param('~blue_hsv/lower', [100, 100, 100]))
        self.blue_upper = np.array(rospy.get_param('~blue_hsv/upper', [130, 255, 255]))
        
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        self.max_contour_area = rospy.get_param('~max_contour_area', 10000)
        
        # Camera intrinsics (will be set from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.debug_image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = self.world_frame
        
        rospy.loginfo("Tracker node initialized")
        
    def camera_info_callback(self, msg):
        """Store camera intrinsics from CameraInfo message."""
        # Camera matrix K (3x3)
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        # Distortion coefficients
        self.dist_coeffs = np.array(msg.D)
        rospy.loginfo_once("Camera intrinsics received")
    
    def detect_marker(self, cv_image, lower_hsv, upper_hsv):
        """
        Detect colored marker in image using HSV thresholding.
        
        Args:
            cv_image: OpenCV image (BGR)
            lower_hsv: Lower HSV bound (numpy array)
            upper_hsv: Upper HSV bound (numpy array)
            
        Returns:
            (u, v) centroid of largest contour, or None if not found
        """
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Threshold
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter by area
        if area < self.min_contour_area or area > self.max_contour_area:
            return None
        
        # Compute centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        
        return (u, v)
    
    def pixel_to_world_point(self, u, v):
        """
        Convert pixel coordinates to world 3D point by ray-plane intersection.
        
        Algorithm:
        1. Build ray in camera frame using intrinsics (unproject pixel)
        2. Transform ray to world frame using TF
        3. Intersect ray with ground plane z = ground_z
        
        Args:
            u, v: Pixel coordinates
            
        Returns:
            (x, y, z) in world frame, or None if transform fails
        """
        if self.camera_matrix is None:
            return None
        
        # Get camera to world transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return None
        
        # Unproject pixel to ray in camera frame
        # Ray direction in camera frame (normalized)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Normalized image coordinates
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        
        # Ray direction in camera frame (z = 1 for normalized coordinates)
        ray_dir_camera = np.array([x_norm, y_norm, 1.0])
        ray_dir_camera = ray_dir_camera / np.linalg.norm(ray_dir_camera)
        
        # Transform ray direction to world frame
        # Extract rotation matrix from transform quaternion
        quat = transform.transform.rotation
        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
        rotation_matrix = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])
        ray_dir_world = rotation_matrix.dot(ray_dir_camera)
        
        # Camera position in world frame
        cam_pos_world = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        # Intersect ray with ground plane z = ground_z
        # Ray: P = cam_pos_world + t * ray_dir_world
        # Plane: z = ground_z
        # Solve: cam_pos_world.z + t * ray_dir_world.z = ground_z
        if abs(ray_dir_world[2]) < 1e-6:
            return None  # Ray parallel to ground
        
        t = (self.ground_z - cam_pos_world[2]) / ray_dir_world[2]
        
        if t < 0:
            return None  # Ray pointing away from ground
        
        point_world = cam_pos_world + t * ray_dir_world
        
        return (point_world[0], point_world[1], point_world[2])
    
    def image_callback(self, msg):
        """Process incoming image and publish robot pose."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return
        
        # Detect markers
        red_center = self.detect_marker(cv_image, self.red_lower, self.red_upper)
        blue_center = self.detect_marker(cv_image, self.blue_lower, self.blue_upper)
        
        # Debug image
        debug_image = cv_image.copy()
        
        if red_center is None or blue_center is None:
            rospy.logwarn_throttle(2.0, "Markers not detected (red: %s, blue: %s)", 
                                  red_center, blue_center)
            # Publish debug image anyway
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError:
                pass
            return
        
        # Draw detected centers
        cv2.circle(debug_image, red_center, 5, (0, 0, 255), -1)
        cv2.circle(debug_image, blue_center, 5, (255, 0, 0), -1)
        cv2.line(debug_image, red_center, blue_center, (0, 255, 0), 2)
        
        # Convert to world coordinates
        red_world = self.pixel_to_world_point(red_center[0], red_center[1])
        blue_world = self.pixel_to_world_point(blue_center[0], blue_center[1])
        
        if red_world is None or blue_world is None:
            rospy.logwarn_throttle(1.0, "Failed to convert pixel to world coordinates")
            return
        
        # Compute robot pose
        # Center = midpoint between markers
        center_x = (red_world[0] + blue_world[0]) / 2.0
        center_y = (red_world[1] + blue_world[1]) / 2.0
        
        # Yaw = atan2(tail - head) = atan2(blue - red)
        dx = blue_world[0] - red_world[0]
        dy = blue_world[1] - red_world[1]
        yaw = np.arctan2(dy, dx)
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = center_x
        pose_msg.pose.position.y = center_y
        pose_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion (rotation around z-axis)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = sy
        pose_msg.pose.orientation.w = cy
        
        self.pose_pub.publish(pose_msg)
        
        # Update path
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_msg)
        self.path_pub.publish(self.path)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr("Failed to publish debug image: %s", str(e))


def main():
    try:
        node = TrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

