def test_public_shims_importable():
    import tracker_pkg.camera as camera
    import tracker_pkg.pose as pose
    import tracker_pkg.processor as processor
    import tracker_pkg.trajectory as trajectory
    import tracker_pkg.detectors as detectors
    import tracker_pkg.scripts as scripts

    # Access exported names to ensure __all__ is correct
    assert hasattr(camera, "CameraIntrinsics")
    assert hasattr(pose, "PoseEstimator")
    assert hasattr(processor, "ImageSequenceProcessor")
    assert hasattr(trajectory, "TrajectoryPoint")
    assert hasattr(detectors, "MarkerDetector")
    assert scripts.__all__ == []
