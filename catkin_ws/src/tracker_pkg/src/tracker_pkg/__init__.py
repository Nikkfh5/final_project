"""
Offline tracker package built for processing image sequences and reconstructing
robot poses from colored markers.
"""

from tracker_pkg.detectors import MarkerDetector, DetectionConfig
from tracker_pkg.camera import CameraIntrinsics, CameraPose, StaticRayProjector
from tracker_pkg.pose import PoseEstimator, Pose
from tracker_pkg.trajectory import Trajectory, TrajectoryPoint
from tracker_pkg.processor import ImageSequenceProcessor

__all__ = [
    "MarkerDetector",
    "DetectionConfig",
    "CameraIntrinsics",
    "CameraPose",
    "StaticRayProjector",
    "PoseEstimator",
    "Pose",
    "Trajectory",
    "TrajectoryPoint",
    "ImageSequenceProcessor",
]
