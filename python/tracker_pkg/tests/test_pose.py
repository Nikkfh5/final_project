import pytest

from tracker_pkg.pose import PoseEstimator


def test_pose_estimator_validates_distance_and_smoothes():
    estimator = PoseEstimator(min_distance=0.01, max_distance=5.0, smoothing_alpha=0.5)

    first = estimator.estimate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert first is not None
    assert first.x == pytest.approx(0.5)
    assert first.y == pytest.approx(0.0)

    second = estimator.estimate((0.0, 1.0, 0.0), (1.0, 1.0, 0.0))
    assert second is not None
    assert second.x == pytest.approx(0.5)
    assert second.y == pytest.approx(0.5)

    # Too far apart should be rejected
    assert estimator.estimate((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)) is None
