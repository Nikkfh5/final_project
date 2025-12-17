from __future__ import annotations

import glob
import os
from typing import Iterable, List, Optional

import cv2

from tracker_pkg.camera import BaseRayProjector
from tracker_pkg.detectors import MarkerDetector
from tracker_pkg.pose import PoseEstimator
from tracker_pkg.trajectory import Trajectory, TrajectoryPoint


class ImageSequenceProcessor:
    """Offline pipeline that converts image sequences into a trajectory."""

    def __init__(
        self,
        detector: MarkerDetector,
        projector: BaseRayProjector,
        estimator: PoseEstimator,
        frame_skip: int = 1,
        time_step: float = 1.0 / 30.0,
    ):
        if frame_skip < 1:
            raise ValueError("frame_skip must be >= 1")
        self.detector = detector
        self.projector = projector
        self.estimator = estimator
        self.frame_skip = frame_skip
        self.time_step = time_step

    def process_directory(self, image_dir: str) -> Trajectory:
        files = self._collect_images(image_dir)
        trajectory = Trajectory()
        for i, path in enumerate(files):
            if i % self.frame_skip != 0:
                continue
            img = cv2.imread(path)
            if img is None:
                continue
            markers = self.detector.detect(img)
            if markers["red"] is None or markers["blue"] is None:
                continue
            red_world = self.projector.pixel_to_world(*markers["red"])
            blue_world = self.projector.pixel_to_world(*markers["blue"])
            if red_world is None or blue_world is None:
                continue
            pose = self.estimator.estimate(red_world, blue_world)
            if pose is None:
                continue
            point = TrajectoryPoint(t=i * self.time_step, pose=pose, frame=i)
            trajectory.append(point)
        return trajectory

    def process_and_save(self, image_dir: str, output: str, fmt: str = "csv") -> Trajectory:
        fmt = fmt.lower()
        trajectory = self.process_directory(image_dir)
        if fmt == "json":
            trajectory.save_json(output)
        else:
            trajectory.save_csv(output)
        return trajectory

    def _collect_images(self, image_dir: str) -> List[str]:
        patterns = ("*.png", "*.jpg", "*.jpeg", "*.bmp")
        files: List[str] = []
        for pattern in patterns:
            files.extend(glob.glob(os.path.join(image_dir, pattern)))
            files.extend(glob.glob(os.path.join(image_dir, pattern.upper())))
        files.sort()
        return files
