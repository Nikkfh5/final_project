import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from tracker_pkg.adapters.detection import DetectionConfig, MarkerDetector
from tracker_pkg.domain.camera import CameraIntrinsics, CameraPose, StaticRayProjector
from tracker_pkg.domain.pose import PoseEstimator
from tracker_pkg.usecases.image_sequence import ImageSequenceProcessor


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


def test_sequence_processor_skips_invalid_frames(monkeypatch, tmp_path: Path):
    # Frame skip validation
    with pytest.raises(ValueError):
        ImageSequenceProcessor(detector=None, projector=None, estimator=None, frame_skip=0)

    # Stubs that trigger skip branches
    class DummyDetector:
        def __init__(self):
            self.calls = 0

        def detect(self, img):
            self.calls += 1
            if self.calls == 1:
                return {"red": None, "blue": (0, 0)}  # missing marker
            return {"red": (0, 0), "blue": (1, 1)}  # projector will nullify

    class DummyProjector:
        def pixel_to_world(self, u, v):
            return None  # forces skip at red_world/blue_world check

    class DummyEstimator:
        def estimate(self, r, b):
            return None

    # Prepare fake files
    paths = [tmp_path / f"frame_{i}.png" for i in range(3)]
    for p in paths:
        p.write_bytes(b"dummy")

    processor = ImageSequenceProcessor(
        detector=DummyDetector(),
        projector=DummyProjector(),
        estimator=DummyEstimator(),
        frame_skip=2,
        time_step=0.1,
    )

    # Patch file collection and imread to avoid real images
    monkeypatch.setattr(ImageSequenceProcessor, "_collect_images", lambda self, _: [str(p) for p in paths])
    monkeypatch.setattr(cv2, "imread", lambda path: None if path.endswith("1.png") else np.zeros((2, 2, 3), dtype=np.uint8))

    trajectory = processor.process_directory(str(tmp_path))
    # All frames should be skipped (missing marker, frame_skip on idx=1, projector None)
    assert len(trajectory) == 0
