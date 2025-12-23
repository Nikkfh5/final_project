from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


class PoseEstimator:
    """Computes robot pose from two marker positions."""

    def __init__(self, min_distance: float = 0.05, max_distance: float = 2.0, smoothing_alpha: Optional[float] = None):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.smoothing_alpha = smoothing_alpha
        self._last_pose: Optional[Pose] = None

    def estimate(self, red_world: Tuple[float, float, float], blue_world: Tuple[float, float, float]) -> Optional[Pose]:
        dx = blue_world[0] - red_world[0]
        dy = blue_world[1] - red_world[1]
        distance = math.hypot(dx, dy)
        if distance < self.min_distance or distance > self.max_distance:
            return None

        cx = (red_world[0] + blue_world[0]) / 2.0
        cy = (red_world[1] + blue_world[1]) / 2.0
        yaw = math.atan2(dy, dx)

        if self.smoothing_alpha is not None and self._last_pose is not None:
            alpha = self.smoothing_alpha
            cx = self._last_pose.x + alpha * (cx - self._last_pose.x)
            cy = self._last_pose.y + alpha * (cy - self._last_pose.y)
            yaw = self._blend_angle(self._last_pose.yaw, yaw, alpha)

        pose = Pose(x=cx, y=cy, yaw=yaw)
        self._last_pose = pose
        return pose

    def _blend_angle(self, a: float, b: float, alpha: float) -> float:
        delta = math.atan2(math.sin(b - a), math.cos(b - a))
        return a + alpha * delta
