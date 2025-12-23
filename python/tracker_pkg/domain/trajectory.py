from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass
from typing import List

from tracker_pkg.domain.pose import Pose


@dataclass
class TrajectoryPoint:
    t: float
    pose: Pose
    frame: int

    def as_dict(self):
        return {"t": self.t, "x": self.pose.x, "y": self.pose.y, "theta": self.pose.yaw, "frame": self.frame}


class Trajectory:
    def __init__(self):
        self.points: List[TrajectoryPoint] = []

    def append(self, point: TrajectoryPoint):
        self.points.append(point)

    def __len__(self):
        return len(self.points)

    def to_dicts(self) -> List[dict]:
        return [p.as_dict() for p in self.points]

    def save_csv(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "x", "y", "theta", "frame"])
            for p in self.points:
                writer.writerow([p.t, p.pose.x, p.pose.y, p.pose.yaw, p.frame])

    def save_json(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dicts(), f, indent=2)
