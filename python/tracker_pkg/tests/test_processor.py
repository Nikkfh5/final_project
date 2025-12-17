import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from tracker_pkg.camera import CameraIntrinsics, CameraPose, StaticRayProjector
from tracker_pkg.detectors import DetectionConfig, MarkerDetector
from tracker_pkg.pose import PoseEstimator
from tracker_pkg.processor import ImageSequenceProcessor


def _draw_frame(path: Path, red_px, blue_px):
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(img, red_px, 8, (0, 0, 255), -1)
    cv2.circle(img, blue_px, 8, (255, 0, 0), -1)
    cv2.imwrite(str(path), img)


def test_sequence_processor_builds_expected_trajectory(tmp_path: Path):
    intrinsics = CameraIntrinsics(fx=150.0, fy=150.0, cx=50.0, cy=50.0)
    pose = CameraPose.from_euler(position=(0.0, 0.0, 1.0), roll=0.0, pitch=math.pi, yaw=0.0)
    projector = StaticRayProjector(intrinsics=intrinsics, pose=pose, ground_z=0.0)
    detector = MarkerDetector(DetectionConfig(min_area=5, max_area=5000, morph_kernel=3))
    estimator = PoseEstimator(min_distance=0.01, max_distance=5.0)
    processor = ImageSequenceProcessor(detector, projector, estimator, frame_skip=1, time_step=0.1)

    expected_positions = []
    for i in range(3):
        red_px = (20 + i * 5, 30)
        blue_px = (40 + i * 5, 30)
        _draw_frame(tmp_path / f"frame_{i:03d}.png", red_px, blue_px)
        red_world = projector.pixel_to_world(*red_px)
        blue_world = projector.pixel_to_world(*blue_px)
        expected_positions.append(((red_world[0] + blue_world[0]) / 2.0, (red_world[1] + blue_world[1]) / 2.0))

    trajectory = processor.process_directory(str(tmp_path))
    assert len(trajectory) == 3

    for point, expected in zip(trajectory.points, expected_positions):
        assert point.pose.x == pytest.approx(expected[0], rel=1e-2)
        assert point.pose.y == pytest.approx(expected[1], rel=1e-2)
