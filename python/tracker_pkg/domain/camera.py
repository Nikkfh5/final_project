from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create rotation matrix using ROS convention Rz * Ry * Rx."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "CameraIntrinsics":
        matrix = np.asarray(matrix).reshape(3, 3)
        return cls(fx=matrix[0, 0], fy=matrix[1, 1], cx=matrix[0, 2], cy=matrix[1, 2])

    @classmethod
    def from_fov(cls, width: int, height: int, horizontal_fov_rad: float) -> "CameraIntrinsics":
        fx = width / (2.0 * np.tan(horizontal_fov_rad / 2.0))
        fy = fx  # square pixels assumption
        cx = width / 2.0
        cy = height / 2.0
        return cls(fx=fx, fy=fy, cx=cx, cy=cy)

    def as_matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class CameraPose:
    position: np.ndarray
    rotation: np.ndarray

    @classmethod
    def from_euler(cls, position: Tuple[float, float, float], roll: float, pitch: float, yaw: float) -> "CameraPose":
        return cls(position=np.array(position, dtype=float), rotation=rotation_matrix_from_rpy(roll, pitch, yaw))


class BaseRayProjector(ABC):
    """Template-method style projector to keep intrinsics/pose providers swappable."""

    def __init__(self, ground_z: float = 0.0):
        self.ground_z = ground_z

    def pixel_to_world(self, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        intrinsics = self.get_intrinsics()
        pose = self.get_pose()
        return self._project(intrinsics, pose, float(u), float(v))

    @abstractmethod
    def get_intrinsics(self) -> CameraIntrinsics:
        raise NotImplementedError

    @abstractmethod
    def get_pose(self) -> CameraPose:
        raise NotImplementedError

    def _project(self, intrinsics: CameraIntrinsics, pose: CameraPose, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        ray_dir_camera = np.array([x_norm, y_norm, 1.0])
        ray_dir_camera = ray_dir_camera / np.linalg.norm(ray_dir_camera)

        ray_world = pose.rotation @ ray_dir_camera
        if abs(ray_world[2]) < 1e-9:
            return None
        t = (self.ground_z - pose.position[2]) / ray_world[2]
        if t < 0:
            return None
        point = pose.position + t * ray_world
        return float(point[0]), float(point[1]), float(point[2])


class StaticRayProjector(BaseRayProjector):
    """Simple projector that serves a fixed camera model."""

    def __init__(self, intrinsics: CameraIntrinsics, pose: CameraPose, ground_z: float = 0.0):
        super().__init__(ground_z=ground_z)
        self._intrinsics = intrinsics
        self._pose = pose

    def get_intrinsics(self) -> CameraIntrinsics:
        return self._intrinsics

    def get_pose(self) -> CameraPose:
        return self._pose
