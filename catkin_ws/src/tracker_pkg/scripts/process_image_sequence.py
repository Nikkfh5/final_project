#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Thin wrapper over the offline tracker core. Detects markers in an image
sequence and exports a trajectory without duplicating business logic.
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parents[4]
offline_pkg = PROJECT_ROOT / "python" / "tracker_pkg"
if offline_pkg.exists() and str(offline_pkg) not in sys.path:
    sys.path.insert(0, str(offline_pkg))

from tracker_pkg.camera import CameraIntrinsics, CameraPose, StaticRayProjector
from tracker_pkg.detectors import DetectionConfig, MarkerDetector
from tracker_pkg.pose import PoseEstimator
from tracker_pkg.processor import ImageSequenceProcessor


def parse_matrix(matrix_str: str) -> CameraIntrinsics:
    values = [float(x) for x in matrix_str.split(",")]
    if len(values) != 9:
        raise ValueError("camera-matrix must contain 9 comma separated numbers")
    return CameraIntrinsics.from_matrix(values)


def parse_pose(pose_str: str) -> CameraPose:
    values = [float(x) for x in pose_str.split(",")]
    if len(values) != 6:
        raise ValueError("camera-pose must contain x,y,z,roll,pitch,yaw")
    x, y, z, roll, pitch, yaw = values
    return CameraPose.from_euler((x, y, z), roll, pitch, yaw)


def build_config(red_hsv: Optional[str], blue_hsv: Optional[str]) -> DetectionConfig:
    config = DetectionConfig()
    if red_hsv:
        vals = [int(x) for x in red_hsv.split(",")]
        config.red_ranges = [((vals[0], vals[1], vals[2]), (vals[3], vals[4], vals[5]))]
    if blue_hsv:
        vals = [int(x) for x in blue_hsv.split(",")]
        config.blue_ranges = [((vals[0], vals[1], vals[2]), (vals[3], vals[4], vals[5]))]
    return config


def main():
    parser = argparse.ArgumentParser(description="Обработка последовательности изображений для трекинга робота")
    parser.add_argument("image_dir", type=str, help="Путь к папке с изображениями")
    parser.add_argument("output_file", type=str, help="Путь к выходному файлу")
    parser.add_argument("--format", choices=["csv", "json"], default="csv", help="Формат выходного файла")
    parser.add_argument("--camera-matrix", type=str, required=True, help='Матрица камеры "fx,0,cx,0,fy,cy,0,0,1"')
    parser.add_argument("--camera-pose", type=str, required=True, help='Поза камеры "x,y,z,roll,pitch,yaw"')
    parser.add_argument("--ground-z", type=float, default=0.0, help="Z координата плоскости пола")
    parser.add_argument("--frame-skip", type=int, default=1, help="Обрабатывать каждый N-й кадр")
    parser.add_argument("--time-step", type=float, default=1.0 / 30.0, help="Шаг времени между кадрами")
    parser.add_argument("--red-hsv", type=str, help='HSV пороги красной метки "lh,ls,lv,uh,us,uv"')
    parser.add_argument("--blue-hsv", type=str, help='HSV пороги синей метки "lh,ls,lv,uh,us,uv"')
    parser.add_argument("--min-distance", type=float, default=0.01, help="Мин. расстояние между метками")
    parser.add_argument("--max-distance", type=float, default=5.0, help="Макс. расстояние между метками")
    parser.add_argument("--smooth-alpha", type=float, help="Сглаживание yaw/позиции (0..1)")
    args = parser.parse_args()

    intrinsics = parse_matrix(args.camera_matrix)
    pose = parse_pose(args.camera_pose)
    config = build_config(args.red_hsv, args.blue_hsv)

    detector = MarkerDetector(config=config)
    projector = StaticRayProjector(intrinsics=intrinsics, pose=pose, ground_z=args.ground_z)
    estimator = PoseEstimator(
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        smoothing_alpha=args.smooth_alpha,
    )
    processor = ImageSequenceProcessor(
        detector=detector,
        projector=projector,
        estimator=estimator,
        frame_skip=args.frame_skip,
        time_step=args.time_step,
    )

    processor.process_and_save(args.image_dir, args.output_file, fmt=args.format)
    print(f"Готово: траектория сохранена в {args.output_file}")


if __name__ == "__main__":
    main()
