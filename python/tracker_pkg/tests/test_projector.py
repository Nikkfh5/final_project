import math

import pytest

from tracker_pkg.camera import CameraIntrinsics, CameraPose, StaticRayProjector


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
