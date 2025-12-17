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
import math
import xml.etree.ElementTree as ET
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros


class TrackerNode:
    def __init__(self):
        rospy.init_node('tracker_node', anonymous=False)
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Load parameters
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.camera_frame_is_optical = rospy.get_param('~camera_frame_is_optical', True)
        self.ground_z = rospy.get_param('~ground_z', 0.0)
        self.min_marker_distance = rospy.get_param('~min_marker_distance', 0.05)
        self.max_marker_distance = rospy.get_param('~max_marker_distance', 2.0)
        self.enable_pose_smoothing = rospy.get_param('~enable_pose_smoothing', False)
        self.pose_smoothing_alpha = rospy.get_param('~pose_smoothing_alpha', 0.2)
        self.fallback_camera_pose = {
            'x': rospy.get_param('~fallback_camera_pose/x', 0.0),
            'y': rospy.get_param('~fallback_camera_pose/y', 0.0),
            'z': rospy.get_param('~fallback_camera_pose/z', 3.0),
            'roll': rospy.get_param('~fallback_camera_pose/roll', 0.0),
            'pitch': rospy.get_param('~fallback_camera_pose/pitch', 1.57),
            'yaw': rospy.get_param('~fallback_camera_pose/yaw', 0.0),
        }
        self.fallback_intrinsics = {
            'width': rospy.get_param('~fallback_camera_intrinsics/width', 640),
            'height': rospy.get_param('~fallback_camera_intrinsics/height', 480),
            'fov': rospy.get_param('~fallback_camera_intrinsics/fov', 1.047),
        }
        self.force_fallback_pose = rospy.get_param('~force_fallback_pose', True)
        self.force_fallback_intrinsics = rospy.get_param('~force_fallback_intrinsics', True)
        self.disable_distance_check = rospy.get_param('~disable_distance_check', True)
        self.world_file = rospy.get_param('~world_file', None)
        self.optical_correction = self._rpy_to_matrix(-math.pi / 2.0, 0.0, -math.pi / 2.0) if self.camera_frame_is_optical else np.eye(3)

        # Optionally load defaults from an SDF world file (no Gazebo needed)
        self._load_world_camera_defaults()
        
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.pose_topic = rospy.get_param('~pose_topic', '/robot_pose_external')
        self.path_topic = rospy.get_param('~path_topic', '/robot_path_external')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/debug/image')
        self.log_trajectory = rospy.get_param('~log_trajectory', False)
        self.log_format = rospy.get_param('~log_format', 'csv').lower()
        self.log_file = os.path.expanduser(rospy.get_param('~log_file', '~/tracker_logs/trajectory.csv'))
        self.logging_active = rospy.get_param('~logging_initially_on', False)
        self.png_output = os.path.expanduser(rospy.get_param('~png_output', ''))
        self.png_dpi = rospy.get_param('~png_dpi', 200)
        
        # HSV thresholds and detection fallbacks
        self.red_hsv_ranges = self._load_hsv_ranges(
            'red_hsv',
            default_lower=[0, 25, 25],
            default_upper=[20, 255, 255],
            default_extra=[{'lower': [170, 25, 25], 'upper': [180, 255, 255]}]
        )
        self.blue_hsv_ranges = self._load_hsv_ranges(
            'blue_hsv',
            default_lower=[95, 25, 25],
            default_upper=[140, 255, 255]
        )
        self.enable_channel_fallback = rospy.get_param('~enable_channel_fallback', True)
        self.red_channel_min = rospy.get_param('~red_channel_min', 60)
        self.red_channel_margin = rospy.get_param('~red_channel_margin', 10)
        self.blue_channel_min = rospy.get_param('~blue_channel_min', 60)
        self.blue_channel_margin = rospy.get_param('~blue_channel_margin', 10)
        self.enable_circle_fallback = rospy.get_param('~enable_circle_fallback', True)
        self.circle_min_radius = rospy.get_param('~circle_min_radius', 5)
        self.circle_max_radius = rospy.get_param('~circle_max_radius', 80)
        self.circle_min_distance = rospy.get_param('~circle_min_distance', 25)
        self.circle_dp = rospy.get_param('~circle_dp', 1.2)
        self.circle_param1 = rospy.get_param('~circle_param1', 100)
        self.circle_param2 = rospy.get_param('~circle_param2', 15)
        self.circle_color_margin = rospy.get_param('~circle_color_margin', 5)
        self.circle_blur_kernel = rospy.get_param('~circle_blur_kernel', 7)
        
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        self.max_contour_area = rospy.get_param('~max_contour_area', 10000)
        self.morph_kernel_size = rospy.get_param('~morph_kernel_size', 5)
        
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
        
        if not self.camera_frame_is_optical:
            rospy.loginfo("camera_frame '%s' is treated as x-forward (Gazebo-style); applying optical correction",
                          self.camera_frame)

        # Subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.debug_image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.start_logging_srv = rospy.Service('~start_logging', Trigger, self.handle_start_logging)
        self.stop_logging_srv = rospy.Service('~stop_logging', Trigger, self.handle_stop_logging)
        self.save_png_srv = rospy.Service('~save_png', Trigger, self.handle_save_png)
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.logged_points = []
        self.smoothed_pose = None
        self.frame_number = 0  # Counter for frame numbers in trajectory
        rospy.on_shutdown(self._on_shutdown)
        
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

    def _rpy_to_matrix(self, roll, pitch, yaw):
        """Convert roll, pitch, yaw to rotation matrix (Rz * Ry * Rx)."""
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)

        Rz = np.array([[cy, -sy, 0],
                       [sy, cy, 0],
                       [0, 0, 1]])
        Ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])
        return Rz.dot(Ry).dot(Rx)

    def _load_world_camera_defaults(self):
        """Load camera pose/intrinsics from SDF world file if provided."""
        if not self.world_file:
            rospy.logwarn_once("world_file param not set; using configured fallbacks")
            return

        try:
            tree = ET.parse(self.world_file)
            root = tree.getroot()
        except Exception as e:
            rospy.logwarn_once("Failed to parse world_file '%s': %s", self.world_file, str(e))
            return

        try:
            camera_model = None
            for model in root.findall(".//model"):
                if model.get('name') == 'external_camera':
                    camera_model = model
                    break

            if camera_model is None:
                rospy.logwarn_once("No model named 'external_camera' in world_file '%s'", self.world_file)
                return

            pose_elem = camera_model.find('pose')
            if pose_elem is not None and pose_elem.text:
                parts = pose_elem.text.strip().split()
                if len(parts) == 6:
                    vals = list(map(float, parts))
                    self.fallback_camera_pose.update({
                        'x': vals[0], 'y': vals[1], 'z': vals[2],
                        'roll': vals[3], 'pitch': vals[4], 'yaw': vals[5],
                    })

            camera_sensor = camera_model.find(".//sensor[@type='camera']")
            if camera_sensor is not None:
                cam = camera_sensor.find('camera')
                if cam is not None:
                    fov_elem = cam.find('horizontal_fov')
                    if fov_elem is not None and fov_elem.text:
                        self.fallback_intrinsics['fov'] = float(fov_elem.text)
                    image_elem = cam.find('image')
                    if image_elem is not None:
                        w_elem = image_elem.find('width')
                        h_elem = image_elem.find('height')
                        if w_elem is not None and w_elem.text:
                            self.fallback_intrinsics['width'] = int(w_elem.text)
                        if h_elem is not None and h_elem.text:
                            self.fallback_intrinsics['height'] = int(h_elem.text)

            rospy.loginfo_once(
                "Loaded camera defaults from world_file=%s pose=%s intrinsics=%s",
                self.world_file, self.fallback_camera_pose, self.fallback_intrinsics
            )
        except Exception as e:
            rospy.logwarn_once("Error extracting camera defaults from '%s': %s", self.world_file, str(e))

    def _apply_optical_correction(self, rotation_matrix):
        """Apply static optical frame correction when using fallback transforms."""
        if rotation_matrix is None:
            return None
        if not self.camera_frame_is_optical:
            return rotation_matrix
        return rotation_matrix.dot(self.optical_correction)

    def _get_transform_world_to_camera(self):
        """
        Try to get world->camera transform from TF, otherwise fall back to params.
        Returns (translation ndarray shape (3,), rotation matrix 3x3) or (None, None).
        """
        if self.force_fallback_pose:
            t = self.fallback_camera_pose
            translation = np.array([t['x'], t['y'], t['z']])
            rotation_matrix = self._apply_optical_correction(
                self._rpy_to_matrix(t['roll'], t['pitch'], t['yaw'])
            )
            rospy.loginfo_once("Using forced fallback camera pose: %s", t)
            return translation, rotation_matrix

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame, rospy.Time())
            quat = transform.transform.rotation
            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
            rotation_matrix = np.array([
                [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
            ])
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            return translation, rotation_matrix
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "TF lookup failed, using fallback camera pose: %s", str(e))

        # Fallback to static parameters
        t = self.fallback_camera_pose
        translation = np.array([t['x'], t['y'], t['z']])
        rotation_matrix = self._apply_optical_correction(
            self._rpy_to_matrix(t['roll'], t['pitch'], t['yaw'])
        )
        rospy.loginfo_once("Using fallback camera pose (no TF): %s", t)
        return translation, rotation_matrix

    def _build_fallback_camera_matrix(self):
        """Build a simple pinhole camera matrix from fallback intrinsics."""
        width = float(self.fallback_intrinsics.get('width', 640.0))
        height = float(self.fallback_intrinsics.get('height', 480.0))
        fov = float(self.fallback_intrinsics.get('fov', 1.047))
        fx = fy = width / (2.0 * math.tan(fov / 2.0))
        cx = width / 2.0
        cy = height / 2.0
        K = np.array([[fx, 0.0, cx],
                      [0.0, fy, cy],
                      [0.0, 0.0, 1.0]])
        dist = np.zeros(5)
        return K, dist
        
    def camera_info_callback(self, msg):
        """Store camera intrinsics from CameraInfo message."""
        # Camera matrix K (3x3)
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        # Distortion coefficients
        self.dist_coeffs = np.array(msg.D)
        rospy.loginfo_once("Camera intrinsics received")

    def _load_hsv_ranges(self, param_prefix, default_lower, default_upper, default_extra=None):
        """Load HSV ranges from params, supporting multiple intervals."""
        ranges = []
        lower = np.array(rospy.get_param(f'~{param_prefix}/lower', default_lower))
        upper = np.array(rospy.get_param(f'~{param_prefix}/upper', default_upper))
        ranges.append((lower, upper))

        extra_ranges = rospy.get_param(f'~{param_prefix}/extra_ranges', default_extra or [])
        for rng in extra_ranges:
            extra_lower = np.array(rng.get('lower', default_lower))
            extra_upper = np.array(rng.get('upper', default_upper))
            ranges.append((extra_lower, extra_upper))
        return ranges

    def _apply_morphology(self, mask):
        """Clean up binary mask using configurable kernel."""
        if mask is None:
            return None
        kernel_size = max(1, int(self.morph_kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1  # prefer odd size kernels
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _channel_fallback_mask(self, cv_image, color_name):
        """Use raw BGR channel dominance when HSV detection fails."""
        if not self.enable_channel_fallback:
            return None

        b = cv_image[:, :, 0].astype(np.int16)
        g = cv_image[:, :, 1].astype(np.int16)
        r = cv_image[:, :, 2].astype(np.int16)

        if color_name == 'red':
            dominant = r - np.maximum(b, g)
            mask = np.where(
                (dominant >= self.red_channel_margin) & (r >= self.red_channel_min),
                255,
                0
            ).astype(np.uint8)
        elif color_name == 'blue':
            dominant = b - np.maximum(r, g)
            mask = np.where(
                (dominant >= self.blue_channel_margin) & (b >= self.blue_channel_min),
                255,
                0
            ).astype(np.uint8)
        else:
            return None

        return mask if np.count_nonzero(mask) else None

    def _extract_centroid_from_mask(self, mask):
        """Find marker centroid in mask, honoring contour size limits."""
        if mask is None:
            return None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area < self.min_contour_area or area > self.max_contour_area:
            rospy.logdebug("Contour area %d out of range [%d, %d]", area,
                           self.min_contour_area, self.max_contour_area)
            return None

        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        return (u, v)

    def _detect_markers_via_circles(self, cv_image):
        """Fallback detection using Hough circles on grayscale image."""
        if not self.enable_circle_fallback:
            return {}

        blur_kernel = max(3, int(self.circle_blur_kernel))
        if blur_kernel % 2 == 0:
            blur_kernel += 1

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=self.circle_dp,
            minDist=self.circle_min_distance,
            param1=self.circle_param1,
            param2=self.circle_param2,
            minRadius=self.circle_min_radius,
            maxRadius=self.circle_max_radius
        )

        results = {}
        if circles is None:
            return results
        
        h, w = cv_image.shape[:2]
        best_scores = {'red': (-np.inf, None), 'blue': (-np.inf, None)}
        best_means = {'red': None, 'blue': None}
        
        for circle in np.round(circles[0, :]).astype(int):
            x, y, r = circle
            if x < 0 or y < 0 or x >= w or y >= h:
                continue
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (x, y), r, 255, -1)
            mean_bgr = cv2.mean(cv_image, mask=mask)[:3]
            b, g, r_val = mean_bgr
            red_diff = float(r_val) - float(max(g, b))
            blue_diff = float(b) - float(max(r_val, g))

            if red_diff > best_scores['red'][0]:
                best_scores['red'] = (red_diff, (x, y))
                best_means['red'] = mean_bgr
            if blue_diff > best_scores['blue'][0]:
                best_scores['blue'] = (blue_diff, (x, y))
                best_means['blue'] = mean_bgr

        for color in ('red', 'blue'):
            diff, center = best_scores[color]
            if center is not None and diff >= self.circle_color_margin:
                results[color] = center
            else:
                if center is not None:
                    rospy.logdebug("Circle %s candidate rejected diff=%.2f < %.2f, mean=%s",
                                   color, diff, self.circle_color_margin,
                                   np.round(best_means[color], 2) if best_means[color] is not None else None)

        return results
    
    def detect_marker(self, cv_image, hsv_ranges, color_name):
        """
        Detect colored marker in image using HSV thresholding.
        
        Args:
            cv_image: OpenCV image (BGR)
            hsv_ranges: List of (lower, upper) HSV arrays
            color_name: 'red' or 'blue', used for fallbacks
            
        Returns:
            ((u, v) centroid, mask) tuple, where centroid is None if not found, 
            but mask is always returned for debugging
        """
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Combine all HSV ranges
        mask = None
        for lower, upper in hsv_ranges:
            current = cv2.inRange(hsv, lower, upper)
            mask = current if mask is None else cv2.bitwise_or(mask, current)
        
        mask = self._apply_morphology(mask)
        centroid = self._extract_centroid_from_mask(mask)
        
        if centroid is None:
            fallback_mask = self._channel_fallback_mask(cv_image, color_name)
            if fallback_mask is not None:
                mask = self._apply_morphology(fallback_mask)
                centroid = self._extract_centroid_from_mask(mask)
        
        return centroid, mask
    
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
        camera_matrix = self.camera_matrix
        dist_coeffs = self.dist_coeffs

        use_fallback_intrinsics = (
            self.force_fallback_intrinsics
            or camera_matrix is None
            or camera_matrix.shape != (3, 3)
        )
        if use_fallback_intrinsics:
            camera_matrix, dist_coeffs = self._build_fallback_camera_matrix()
            rospy.loginfo_once("Using fallback camera intrinsics")
        else:
            if dist_coeffs is None or len(dist_coeffs) == 0:
                dist_coeffs = np.zeros(5)
        
        rospy.logdebug_once("Camera matrix: %s", camera_matrix)
        
        # Get camera to world transform (TF or fallback)
        translation, rotation_matrix = self._get_transform_world_to_camera()
        if translation is None or rotation_matrix is None:
            rospy.logwarn_throttle(1.0, "Camera transform is unavailable, skipping frame")
            return None
        
        # Unproject pixel to ray in camera frame
        # Ray direction in camera frame (normalized)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Undistort pixel coordinates if distortion coefficients are available
        if dist_coeffs is not None and len(dist_coeffs) > 0:
            # Undistort the point
            pts = np.array([[[u, v]]], dtype=np.float32)
            undistorted = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=camera_matrix)
            u_undist, v_undist = undistorted[0, 0]
        else:
            u_undist, v_undist = u, v
        
        # Normalized image coordinates
        x_norm = (u_undist - cx) / fx
        y_norm = -(v_undist - cy) / fy
        
        # Ray direction in camera optical frame (z = 1 for normalized coordinates)
        ray_dir_camera = np.array([x_norm, y_norm, 1.0])
        ray_dir_camera = ray_dir_camera / np.linalg.norm(ray_dir_camera)
        
        # Transform ray direction to world frame
        ray_dir_world = rotation_matrix.dot(ray_dir_camera)

        # Ensure the ray points toward the ground; flip if z is upward
        if ray_dir_world[2] >= 0.0:
            ray_dir_world *= -1
            rospy.logwarn_once("Ray direction flipped downward because it was pointing up (using fallback pose)")
            rospy.loginfo_once("Ray direction flipped downward because it was pointing up (using fallback pose)")
        
        # Camera position in world frame
        cam_pos_world = translation

        # Log once to verify ray points downward
        rospy.loginfo_once("Ray dir world: %s, cam pos world: %s", ray_dir_world, cam_pos_world)
        
        # Intersect ray with ground plane z = ground_z
        # Ray: P = cam_pos_world + t * ray_dir_world
        # Plane: z = ground_z
        # Solve: cam_pos_world.z + t * ray_dir_world.z = ground_z
        if abs(ray_dir_world[2]) < 1e-6:
            return None  # Ray parallel to ground
        
        t = (self.ground_z - cam_pos_world[2]) / ray_dir_world[2]
        
        if t < 0:
            ray_dir_world *= -1
            if abs(ray_dir_world[2]) < 1e-6:
                return None
            t = (self.ground_z - cam_pos_world[2]) / ray_dir_world[2]
            if t < 0:
                return None  # Ray still pointing away from ground
        
        point_world = cam_pos_world + t * ray_dir_world
        
        return (point_world[0], point_world[1], point_world[2])
    
    def image_callback(self, msg):
        """Process incoming image and publish robot pose."""
        if rospy.is_shutdown():
            return
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
        red_center, _ = self.detect_marker(cv_image, self.red_hsv_ranges, 'red')
        blue_center, _ = self.detect_marker(cv_image, self.blue_hsv_ranges, 'blue')

        if (red_center is None or blue_center is None) and self.enable_circle_fallback:
            circle_candidates = self._detect_markers_via_circles(cv_image)
            if red_center is None and 'red' in circle_candidates:
                red_center = circle_candidates['red']
            if blue_center is None and 'blue' in circle_candidates:
                blue_center = circle_candidates['blue']
        
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
                    
                    try:
                        self.pose_pub.publish(pose_msg)
                    except rospy.ROSException:
                        if rospy.is_shutdown():
                            return
                    self.path.header.stamp = rospy.Time.now()
                    self.path.poses.append(pose_msg)
                    try:
                        self.path_pub.publish(self.path)
                    except rospy.ROSException:
                        if rospy.is_shutdown():
                            return
                    
                    if self.log_trajectory and self.logging_active:
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
                try:
                    self.debug_image_pub.publish(debug_msg)
                except rospy.ROSException:
                    if rospy.is_shutdown():
                        return
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
                try:
                    self.debug_image_pub.publish(debug_msg)
                except rospy.ROSException:
                    if rospy.is_shutdown():
                        return
            except CvBridgeError as e:
                rospy.logerr("Failed to publish debug image: %s", str(e))
            return

        # Reject detections if marker spacing is implausible
        dist = np.hypot(blue_world[0] - red_world[0], blue_world[1] - red_world[1])
        if (not self.disable_distance_check) and (dist < self.min_marker_distance or dist > self.max_marker_distance):
            rospy.logwarn_throttle(1.0, "Marker distance out of range: %.3f m (expected %.3f..%.3f)",
                                   dist, self.min_marker_distance, self.max_marker_distance)
            return
        
        # Compute robot pose
        # Center = midpoint between markers
        center_x = (red_world[0] + blue_world[0]) / 2.0
        center_y = (red_world[1] + blue_world[1]) / 2.0
        
        # Yaw = atan2(tail - head) = atan2(blue - red)
        dx = red_world[0] - blue_world[0]
        dy = red_world[1] - blue_world[1]
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
        
        try:
            self.pose_pub.publish(pose_msg)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
        
        # Update path
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_msg)
        try:
            self.path_pub.publish(self.path)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return

        if self.log_trajectory and self.logging_active:
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
            try:
                self.debug_image_pub.publish(debug_msg)
            except rospy.ROSException:
                if rospy.is_shutdown():
                    return
        except CvBridgeError as e:
            rospy.logerr("Failed to publish debug image: %s", str(e))

    def _on_shutdown(self):
        self._save_trajectory(generate_png=bool(self.png_output))

    def handle_start_logging(self, _req):
        if not self.log_trajectory:
            return TriggerResponse(success=False, message="log_trajectory is disabled")
        self.logged_points = []
        self.frame_number = 0
        self.logging_active = True
        return TriggerResponse(
            success=True,
            message="Logging started; writing to %s (%s)" % (self.log_file, self.log_format)
        )

    def handle_stop_logging(self, _req):
        if not self.log_trajectory:
            return TriggerResponse(success=False, message="log_trajectory is disabled")
        was_active = self.logging_active
        self.logging_active = False
        saved = self._save_trajectory(generate_png=True)
        msg = "Stopped logging" if was_active else "Logging already stopped"
        return TriggerResponse(success=saved, message="%s; saved=%s" % (msg, saved))

    def handle_save_png(self, _req):
        saved = self._render_png(self.log_file if self.log_file else "")
        return TriggerResponse(success=saved, message="PNG saved" if saved else "PNG not saved (no data)")

    def _save_trajectory(self, generate_png=False):
        """Persist collected trajectory to disk and optionally render PNG."""
        if not self.log_trajectory or not self.logged_points:
            return False

        log_file_expanded = os.path.expanduser(self.log_file)
        directory = os.path.dirname(log_file_expanded)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)

        try:
            if self.log_format == 'json':
                with open(log_file_expanded, 'w') as f:
                    json.dump(self.logged_points, f, indent=2)
            else:
                with open(log_file_expanded, 'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(['t', 'x', 'y', 'theta', 'frame'])
                    for p in self.logged_points:
                        writer.writerow([
                            p['t'],
                            p['x'],
                            p['y'],
                            p['theta'],
                            p.get('frame', -1)
                        ])
            rospy.loginfo("Saved trajectory with %d points to %s", len(self.logged_points), log_file_expanded)
            if generate_png:
                self._render_png(log_file_expanded)
            return True
        except Exception as e:
            rospy.logerr("Failed to save trajectory: %s", str(e))
            return False

    def _render_png(self, log_file_path):
        """Render a simple PNG of the trajectory."""
        if not self.logged_points:
            return False

        xs = [p['x'] for p in self.logged_points]
        ys = [p['y'] for p in self.logged_points]
        if not xs or not ys:
            return False

        out_path = self.png_output if self.png_output else ""
        if not out_path:
            base, _ = os.path.splitext(log_file_path)
            out_path = base + ".png"

        out_path = os.path.expanduser(out_path)
        out_dir = os.path.dirname(out_path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir)

        try:
            plt.figure(figsize=(6, 6))
            plt.plot(xs, ys, 'b-', alpha=0.8, label='trajectory')
            plt.scatter([xs[0]], [ys[0]], c='g', label='start')
            plt.scatter([xs[-1]], [ys[-1]], c='r', label='end')
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Robot trajectory')
            plt.tight_layout()
            plt.savefig(out_path, dpi=self.png_dpi)
            plt.close()
            rospy.loginfo("Saved trajectory PNG to %s", out_path)
            return True
        except Exception as e:
            rospy.logerr("Failed to render PNG: %s", str(e))
            return False


def main():
    try:
        node = TrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
