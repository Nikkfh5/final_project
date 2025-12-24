import math

import pytest

from tracker_pkg.domain.camera import CameraIntrinsics, CameraPose, StaticRayProjector


def test_pixel_to_world_hits_ground_plane():
    intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=100.0, cy=100.0)
    pose = CameraPose.from_euler(position=(0.0, 0.0, 1.0), roll=0.0, pitch=math.pi, yaw=0.0)
    projector = StaticRayProjector(intrinsics=intrinsics, pose=pose, ground_z=0.0)

    center_hit = projector.pixel_to_world(100, 100)
    assert center_hit is not None
    assert abs(center_hit[0]) < 1e-6
    assert abs(center_hit[1]) < 1e-6
    assert abs(center_hit[2]) < 1e-6

    right_hit = projector.pixel_to_world(200, 100)
    assert right_hit is not None
    assert right_hit[0] == pytest.approx(-1.0, rel=1e-3)
    assert right_hit[1] == pytest.approx(0.0, abs=1e-6)
    assert right_hit[2] == pytest.approx(0.0, abs=1e-6)


def test_intrinsics_and_pose_helpers():
    mat = [200.0, 0, 50.0, 0, 200.0, 60.0, 0, 0, 1.0]
    intr = CameraIntrinsics.from_matrix(mat)
    assert intr.as_matrix()[0, 0] == pytest.approx(200.0)

    fov_intr = CameraIntrinsics.from_fov(width=200, height=100, horizontal_fov_rad=math.pi / 3)
    assert fov_intr.cx == pytest.approx(100.0)
    assert fov_intr.cy == pytest.approx(50.0)

    pose = CameraPose.from_euler((0, 0, 1), roll=0.0, pitch=0.0, yaw=0.0)
    assert pose.position[2] == pytest.approx(1.0)


def test_project_rejects_parallel_and_upward_rays():
    intr = CameraIntrinsics(fx=100.0, fy=100.0, cx=0.0, cy=0.0)

    # Ray parallel to ground: z component ~0 -> None
    pose_parallel = CameraPose.from_euler((0, 0, 1.0), roll=0.0, pitch=math.pi / 2, yaw=0.0)
    projector_parallel = StaticRayProjector(intr, pose_parallel, ground_z=0.0)
    assert projector_parallel.pixel_to_world(0, 0) is None

    # Ray upwards: intersection behind camera -> None
    pose_up = CameraPose.from_euler((0, 0, 1.0), roll=0.0, pitch=0.0, yaw=0.0)
    projector_up = StaticRayProjector(intr, pose_up, ground_z=0.0)
    assert projector_up.pixel_to_world(0, 0) is None
