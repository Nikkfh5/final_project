"""
Offline tracker package built for processing image sequences and reconstructing
robot poses from colored markers.
"""

from tracker_pkg.adapters.detection import DetectionConfig, MarkerCandidate, MarkerDetector
from tracker_pkg.domain.camera import (
    BaseRayProjector,
    CameraIntrinsics,
    CameraPose,
    StaticRayProjector,
    rotation_matrix_from_rpy,
)
from tracker_pkg.domain.pose import Pose, PoseEstimator
from tracker_pkg.domain.trajectory import Trajectory, TrajectoryPoint
from tracker_pkg.usecases.image_sequence import ImageSequenceProcessor

__all__ = [
    "BaseRayProjector",
    "rotation_matrix_from_rpy",
    "MarkerDetector",
    "DetectionConfig",
    "MarkerCandidate",
    "CameraIntrinsics",
    "CameraPose",
    "StaticRayProjector",
    "PoseEstimator",
    "Pose",
    "Trajectory",
    "TrajectoryPoint",
    "ImageSequenceProcessor",
]
