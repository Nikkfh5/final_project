import argparse
import math
import sys

from tracker_pkg.adapters.detection import DetectionConfig, MarkerDetector
from tracker_pkg.domain.camera import CameraIntrinsics, CameraPose, StaticRayProjector
from tracker_pkg.domain.pose import PoseEstimator
from tracker_pkg.usecases.image_sequence import ImageSequenceProcessor


def parse_intrinsics(args) -> CameraIntrinsics:
    if args.camera_matrix:
        values = [float(v) for v in args.camera_matrix.split(",")]
        if len(values) != 9:
            raise ValueError("camera_matrix must contain 9 comma separated numbers")
        return CameraIntrinsics.from_matrix(values)
    return CameraIntrinsics.from_fov(width=args.width, height=args.height, horizontal_fov_rad=args.fov)


def parse_pose(args) -> CameraPose:
    if len(args.camera_pose) != 6:
        raise ValueError("camera_pose must include x y z roll pitch yaw")
    x, y, z, roll, pitch, yaw = [float(v) for v in args.camera_pose]
    return CameraPose.from_euler(position=(x, y, z), roll=roll, pitch=pitch, yaw=yaw)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline tracker for image sequences.")
    parser.add_argument("image_dir", help="Directory with input images")
    parser.add_argument("output", help="Output file (csv or json)")
    parser.add_argument("--format", choices=["csv", "json"], default="csv", help="Output format")
    parser.add_argument("--camera-matrix", dest="camera_matrix", help='Flattened camera matrix "fx,0,cx,0,fy,cy,0,0,1"')
    parser.add_argument("--width", type=int, default=640, help="Image width (used with fov)")
    parser.add_argument("--height", type=int, default=480, help="Image height (used with fov)")
    parser.add_argument("--fov", type=float, default=math.radians(60.0), help="Horizontal FOV in radians (used when camera_matrix not set)")
    parser.add_argument("--camera-pose", nargs=6, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"), default=[0.0, 0.0, 1.0, 0.0, math.pi, 0.0])
    parser.add_argument("--ground-z", type=float, default=0.0, help="Ground plane height")
    parser.add_argument("--frame-skip", type=int, default=1, help="Process every Nth frame")
    parser.add_argument("--time-step", type=float, default=1.0 / 30.0, help="Seconds between frames")
    parser.add_argument("--min-distance", type=float, default=0.05, help="Minimum marker spacing in meters")
    parser.add_argument("--max-distance", type=float, default=2.0, help="Maximum marker spacing in meters")
    parser.add_argument("--smooth-alpha", type=float, help="Enable smoothing with given alpha in [0,1]")
    args = parser.parse_args(argv)

    intrinsics = parse_intrinsics(args)
    pose = parse_pose(args)
    projector = StaticRayProjector(intrinsics=intrinsics, pose=pose, ground_z=args.ground_z)
    detector = MarkerDetector(config=DetectionConfig())
    estimator = PoseEstimator(min_distance=args.min_distance, max_distance=args.max_distance, smoothing_alpha=args.smooth_alpha)
    processor = ImageSequenceProcessor(
        detector=detector,
        projector=projector,
        estimator=estimator,
        frame_skip=args.frame_skip,
        time_step=args.time_step,
    )

    trajectory = processor.process_and_save(args.image_dir, args.output, fmt=args.format)
    print(f"Saved {len(trajectory)} points to {args.output}")


if __name__ == "__main__":
    main()
