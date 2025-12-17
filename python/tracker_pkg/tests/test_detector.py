import cv2
import numpy as np

from tracker_pkg.detectors import DetectionConfig, MarkerDetector


def test_hsv_detection_finds_markers():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(img, (50, 100), 15, (0, 0, 255), -1)  # red in BGR
    cv2.circle(img, (150, 100), 15, (255, 0, 0), -1)  # blue in BGR

    detector = MarkerDetector(DetectionConfig(min_area=10, max_area=5000, morph_kernel=3))
    result = detector.detect(img)

    assert result["red"] is not None
    assert result["blue"] is not None
    assert np.linalg.norm(np.array(result["red"]) - np.array([50, 100])) < 2
    assert np.linalg.norm(np.array(result["blue"]) - np.array([150, 100])) < 2


def test_channel_fallback_triggers_when_hsv_fails():
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(img, (30, 50), 10, (0, 0, 200), -1)  # red
    cv2.circle(img, (70, 50), 10, (200, 0, 0), -1)  # blue

    config = DetectionConfig(
        red_ranges=[((0, 0, 0), (0, 0, 0))],  # intentionally wrong to force fallback
        blue_ranges=[((0, 0, 0), (0, 0, 0))],
        min_area=5,
        max_area=2000,
        morph_kernel=3,
        enable_channel_fallback=True,
        enable_circle_fallback=False,
    )
    detector = MarkerDetector(config)
    result = detector.detect(img)

    assert result["red"] is not None
    assert result["blue"] is not None
