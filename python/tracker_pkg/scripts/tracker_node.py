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

import os
import json
import csv
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros


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
        self.min_marker_distance = rospy.get_param('~min_marker_distance', 0.05)
        self.max_marker_distance = rospy.get_param('~max_marker_distance', 2.0)
        self.enable_pose_smoothing = rospy.get_param('~enable_pose_smoothing', False)
        self.pose_smoothing_alpha = rospy.get_param('~pose_smoothing_alpha', 0.2)
        
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.pose_topic = rospy.get_param('~pose_topic', '/robot_pose_external')
        self.path_topic = rospy.get_param('~path_topic', '/robot_path_external')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/debug/image')
        self.log_trajectory = rospy.get_param('~log_trajectory', False)
        self.log_format = rospy.get_param('~log_format', 'csv').lower()
        self.log_file = os.path.expanduser(rospy.get_param('~log_file', '~/tracker_logs/trajectory.csv'))
        
        # HSV thresholds
        self.red_lower = np.array(rospy.get_param('~red_hsv/lower', [0, 100, 100]))
        self.red_upper = np.array(rospy.get_param('~red_hsv/upper', [10, 255, 255]))
        self.blue_lower = np.array(rospy.get_param('~blue_hsv/lower', [100, 100, 100]))
        self.blue_upper = np.array(rospy.get_param('~blue_hsv/upper', [130, 255, 255]))
        
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        self.max_contour_area = rospy.get_param('~max_contour_area', 10000)
        
        # Frame skipping (process every N-th frame)
        self.frame_skip = rospy.get_param('~frame_skip', 1)  # 1 = process every frame
        self.frame_counter = 0
        
        # Interpolation when markers are lost
        self.enable_interpolation = rospy.get_param('~enable_interpolation', True)
        self.last_valid_pose = None
        self.markers_lost_count = 0
        self.max_lost_frames = rospy.get_param('~max_lost_frames', 10)  # Max frames to interpolate
        
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
        self.logged_points = []
        self.smoothed_pose = None
        self.frame_number = 0  # Counter for frame numbers in trajectory

        rospy.on_shutdown(self.save_trajectory)
        
        # Validate parameters
        if self.min_marker_distance >= self.max_marker_distance:
            rospy.logerr("min_marker_distance (%.3f) must be < max_marker_distance (%.3f)",
                         self.min_marker_distance, self.max_marker_distance)
            raise ValueError("Invalid marker distance parameters")
        
        if self.frame_skip < 1:
            rospy.logerr("frame_skip must be >= 1, got %d", self.frame_skip)
            raise ValueError("Invalid frame_skip parameter")
        
        if self.pose_smoothing_alpha < 0.0 or self.pose_smoothing_alpha > 1.0:
            rospy.logwarn("pose_smoothing_alpha should be in [0, 1], got %.2f", self.pose_smoothing_alpha)
        
        # Wait for camera_info before starting processing
        rospy.loginfo("Waiting for camera_info...")
        try:
            rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
            rospy.loginfo("Camera info received, tracker node ready")
        except rospy.ROSException:
            rospy.logwarn("Camera info not received within timeout, continuing anyway...")
        
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
            ((u, v) centroid, mask) tuple, where centroid is None if not found, 
            but mask is always returned for debugging
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
            return None, mask
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter by area
        if area < self.min_contour_area or area > self.max_contour_area:
            rospy.logdebug("Contour area %d out of range [%d, %d]", area, self.min_contour_area, self.max_contour_area)
            return None, mask
        
        # Compute centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None, mask
        
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        
        return (u, v), mask
    
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
            rospy.logwarn_throttle(5.0, "Camera matrix not yet initialized, waiting for camera_info...")
            return None
        
        rospy.logdebug_once("Camera matrix: %s", self.camera_matrix)
        
        # Get camera to world transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rospy.Time())
            rospy.logdebug_once("TF transform found: world -> camera_link")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed: %s", str(e))
            return None
        
        # Unproject pixel to ray in camera frame
        # Ray direction in camera frame (normalized)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Undistort pixel coordinates if distortion coefficients are available
        if self.dist_coeffs is not None and len(self.dist_coeffs) > 0:
            # Undistort the point
            pts = np.array([[[u, v]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(pts, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
            u_undist, v_undist = undistorted[0, 0]
        else:
            u_undist, v_undist = u, v
        
        # Normalized image coordinates
        x_norm = (u_undist - cx) / fx
        y_norm = (v_undist - cy) / fy
        
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
        # Frame skipping: process only every N-th frame
        self.frame_counter += 1
        if self.frame_counter < self.frame_skip:
            return
        self.frame_counter = 0
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return
        
        # Проверка, что изображение не пустое
        if cv_image is None or cv_image.size == 0:
            rospy.logwarn_throttle(2.0, "Received empty image")
            return
        
        # Detect markers
        # Красный цвет в HSV может быть около 0 или около 180 (циклический диапазон)
        # Пробуем оба диапазона для красного
        red_center, red_mask = self.detect_marker(cv_image, self.red_lower, self.red_upper)
        if red_center is None:
            # Пробуем второй диапазон для красного (170-180)
            red_lower2 = np.array([170, self.red_lower[1], self.red_lower[2]])
            red_upper2 = np.array([180, self.red_upper[1], self.red_upper[2]])
            red_center, red_mask = self.detect_marker(cv_image, red_lower2, red_upper2)
        
        blue_center, blue_mask = self.detect_marker(cv_image, self.blue_lower, self.blue_upper)
        
        # Debug image - показываем исходное изображение
        debug_image = cv_image.copy()
        
        # Добавляем отладочный текст
        status_text = f"Red: {'FOUND' if red_center else 'NOT FOUND'}, Blue: {'FOUND' if blue_center else 'NOT FOUND'}"
        cv2.putText(debug_image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Рисуем найденные центры меток
        if red_center is not None:
            cv2.circle(debug_image, red_center, 15, (0, 0, 255), 3)  # Красный круг
            cv2.putText(debug_image, "RED", (red_center[0] + 20, red_center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if blue_center is not None:
            cv2.circle(debug_image, blue_center, 15, (255, 0, 0), 3)  # Синий круг
            cv2.putText(debug_image, "BLUE", (blue_center[0] + 20, blue_center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Если обе метки найдены, рисуем линию между ними
        if red_center is not None and blue_center is not None:
            cv2.line(debug_image, red_center, blue_center, (0, 255, 0), 3)
        
        if red_center is None or blue_center is None:
            rospy.logwarn_throttle(2.0, "Markers not detected (red: %s, blue: %s)", 
                                  red_center, blue_center)
            
            # Try interpolation if enabled and we have a last valid pose
            if self.enable_interpolation and self.last_valid_pose is not None:
                if self.markers_lost_count < self.max_lost_frames:
                    self.markers_lost_count += 1
                    # Use last valid pose for interpolation
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
                    pose_msg.header.frame_id = self.world_frame
                    pose_msg.pose = self.last_valid_pose
                    
                    self.pose_pub.publish(pose_msg)
                    self.path.header.stamp = rospy.Time.now()
                    self.path.poses.append(pose_msg)
                    self.path_pub.publish(self.path)
                    
                    if self.log_trajectory:
                        # Extract pose data for logging
                        x = pose_msg.pose.position.x
                        y = pose_msg.pose.position.y
                        # Convert quaternion to yaw
                        q = pose_msg.pose.orientation
                        yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                        self.logged_points.append({
                            't': pose_msg.header.stamp.to_sec(),
                            'x': x,
                            'y': y,
                            'theta': yaw,
                            'frame': self.frame_number
                        })
                        self.frame_number += 1
                    rospy.logdebug("Using interpolated pose (lost frames: %d)", self.markers_lost_count)
                else:
                    rospy.logwarn_throttle(2.0, "Markers lost for too long, stopping interpolation")
            
            # Publish debug image anyway (даже если метки не найдены)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header  # Сохраняем header от исходного сообщения
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr("Failed to publish debug image: %s", str(e))
            return
        
        # Markers detected - reset lost counter
        self.markers_lost_count = 0
        
        # Draw detected centers
        cv2.circle(debug_image, red_center, 5, (0, 0, 255), -1)
        cv2.circle(debug_image, blue_center, 5, (255, 0, 0), -1)
        cv2.line(debug_image, red_center, blue_center, (0, 255, 0), 2)
        
        # Convert to world coordinates
        red_world = self.pixel_to_world_point(red_center[0], red_center[1])
        blue_world = self.pixel_to_world_point(blue_center[0], blue_center[1])
        
        if red_world is None or blue_world is None:
            rospy.logwarn_throttle(1.0, "Failed to convert pixel to world coordinates (red: %s, blue: %s)", 
                                  red_world, blue_world)
            # Публикуем debug изображение даже если конвертация не удалась
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr("Failed to publish debug image: %s", str(e))
            return

        # Reject detections if marker spacing is implausible
        dist = np.hypot(blue_world[0] - red_world[0], blue_world[1] - red_world[1])
        if dist < self.min_marker_distance or dist > self.max_marker_distance:
            rospy.logwarn_throttle(1.0, "Marker distance out of range: %.3f m (expected %.3f..%.3f)",
                                   dist, self.min_marker_distance, self.max_marker_distance)
            return
        
        # Compute robot pose
        # Center = midpoint between markers
        center_x = (red_world[0] + blue_world[0]) / 2.0
        center_y = (red_world[1] + blue_world[1]) / 2.0
        
        # Yaw = atan2(tail - head) = atan2(blue - red)
        dx = blue_world[0] - red_world[0]
        dy = blue_world[1] - red_world[1]
        yaw = np.arctan2(dy, dx)

        # Optional smoothing to reduce jitter
        if self.enable_pose_smoothing:
            if self.smoothed_pose is None:
                self.smoothed_pose = (center_x, center_y, yaw)
            else:
                prev_x, prev_y, prev_yaw = self.smoothed_pose
                alpha = self.pose_smoothing_alpha
                center_x = prev_x + alpha * (center_x - prev_x)
                center_y = prev_y + alpha * (center_y - prev_y)
                dyaw = np.arctan2(np.sin(yaw - prev_yaw), np.cos(yaw - prev_yaw))
                yaw = prev_yaw + alpha * dyaw
                self.smoothed_pose = (center_x, center_y, yaw)
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
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
        
        # Store as last valid pose for interpolation
        if self.enable_interpolation:
            self.last_valid_pose = pose_msg.pose
        
        self.pose_pub.publish(pose_msg)
        
        # Update path
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_msg)
        self.path_pub.publish(self.path)

        if self.log_trajectory:
            self.logged_points.append({
                't': pose_msg.header.stamp.to_sec(),
                'x': center_x,
                'y': center_y,
                'theta': yaw,
                'frame': self.frame_number
            })
            self.frame_number += 1
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header  # Сохраняем header от исходного сообщения
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr("Failed to publish debug image: %s", str(e))

    def save_trajectory(self):
        """Persist collected trajectory on shutdown."""
        if not self.log_trajectory or not self.logged_points:
            return

        # Expand user path (~) before getting directory
        log_file_expanded = os.path.expanduser(self.log_file)
        directory = os.path.dirname(log_file_expanded)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)

        # Use expanded path for file operations
        try:
            if self.log_format == 'json':
                with open(log_file_expanded, 'w') as f:
                    json.dump(self.logged_points, f, indent=2)
            else:
                # default: csv
                with open(log_file_expanded, 'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(['t', 'x', 'y', 'theta', 'frame'])
                    for p in self.logged_points:
                        writer.writerow([
                            p['t'], 
                            p['x'], 
                            p['y'], 
                            p['theta'],
                            p.get('frame', -1)  # -1 if frame number not available
                        ])
            rospy.loginfo("Saved trajectory with %d points to %s", len(self.logged_points), log_file_expanded)
        except Exception as e:
            rospy.logerr("Failed to save trajectory: %s", str(e))


def main():
    try:
        node = TrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
