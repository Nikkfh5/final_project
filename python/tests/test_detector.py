import cv2
import numpy as np

from tracker_pkg.adapters.detection import DetectionConfig, MarkerDetector


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


def test_detection_helpers_cover_branches(monkeypatch):
    detector = MarkerDetector(DetectionConfig(min_area=1, max_area=5000, morph_kernel=4, channel_margin=5))

    # _detections_from_mask returns [] for empty mask
    assert detector._detections_from_mask(None) == []

    # Force m00 == 0 branch by monkeypatching cv2.moments
    mask = np.zeros((20, 20), dtype=np.uint8)
    cv2.rectangle(mask, (5, 5), (10, 10), 255, -1)
    monkeypatch.setattr(cv2, "moments", lambda cnt: {"m00": 0})
    assert detector._detections_from_mask(mask) == []

    # Morphology makes kernel odd when even provided
    morph_mask = detector._apply_morphology(mask)
    assert morph_mask.shape == mask.shape

    # Channel dominance returns None for unknown color
    color_image = np.dstack([mask, mask, mask])
    assert detector._channel_dominance_mask(color_image, "green") is None
    assert detector._select_best([]) is None


def test_circle_detection_branches(monkeypatch):
    detector = MarkerDetector(
        DetectionConfig(circle_min_radius=1, circle_max_radius=10, circle_min_distance=1, circle_dp=1.0, min_area=1)
    )
    img = np.zeros((20, 20, 3), dtype=np.uint8)

    # First: no circles detected path
    monkeypatch.setattr(cv2, "HoughCircles", lambda *args, **kwargs: None)
    assert detector._detections_from_circles(img, "red") == []

    # Second: circles present but skipped (out of bounds and unknown color)
    circles = np.array([[[ -1, 5, 3], [5, 5, 3]]], dtype=float)
    monkeypatch.setattr(cv2, "HoughCircles", lambda *args, **kwargs: circles)
    results = detector._detections_from_circles(img, "green")  # unknown color triggers continue branch
    assert results == []

    # Centroid helper returns center for valid mask
    mask = np.zeros((10, 10), dtype=np.uint8)
    cv2.circle(mask, (4, 6), 2, 255, -1)
    centroid = detector._centroid_from_mask(mask)
    assert centroid == (4, 6)
