from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

ColorRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]


@dataclass
class DetectionConfig:
    """Configuration container for marker detection."""

    red_ranges: List[ColorRange] = None
    blue_ranges: List[ColorRange] = None
    morph_kernel: int = 5
    min_area: int = 50
    max_area: int = 10000
    channel_margin: int = 15
    channel_min: int = 60
    circle_min_radius: int = 5
    circle_max_radius: int = 80
    circle_dp: float = 1.2
    circle_param1: int = 100
    circle_param2: int = 15
    circle_min_distance: int = 25
    enable_channel_fallback: bool = True
    enable_circle_fallback: bool = True

    def __post_init__(self):
        if self.red_ranges is None:
            self.red_ranges = [((0, 25, 25), (20, 255, 255)), ((170, 25, 25), (180, 255, 255))]
        if self.blue_ranges is None:
            self.blue_ranges = [((95, 25, 25), (140, 255, 255))]


@dataclass(frozen=True)
class MarkerCandidate:
    """Single marker detection candidate in pixel space."""

    x: int
    y: int
    area: float
    radius: float

    @property
    def center(self) -> Tuple[int, int]:
        return self.x, self.y


class MarkerDetector:
    """Detects colored markers using configurable strategies."""

    def __init__(self, config: Optional[DetectionConfig] = None):
        self.config = config or DetectionConfig()

    def detect(self, image: np.ndarray) -> Dict[str, object]:
        """
        Returns best marker centers together with all candidates for debugging/selection.

        Structure:
        {
            "red": Optional[Tuple[int, int]],
            "blue": Optional[Tuple[int, int]],
            "red_candidates": List[MarkerCandidate],
            "blue_candidates": List[MarkerCandidate],
        }
        """
        red_candidates = self._detect_all(image, "red", self.config.red_ranges)
        blue_candidates = self._detect_all(image, "blue", self.config.blue_ranges)

        red_best = self._select_best(red_candidates)
        blue_best = self._select_best(blue_candidates)

        return {
            "red": red_best.center if red_best else None,
            "blue": blue_best.center if blue_best else None,
            "red_candidates": red_candidates,
            "blue_candidates": blue_candidates,
        }

    def _detect_all(
        self,
        image: np.ndarray,
        color: str,
        ranges: List[ColorRange],
    ) -> List[MarkerCandidate]:
        detections: List[MarkerCandidate] = []

        hsv_mask = self._mask_from_hsv(image, ranges)
        detections.extend(self._detections_from_mask(hsv_mask))

        if self.config.enable_channel_fallback:
            channel_mask = self._channel_dominance_mask(image, color)
            if channel_mask is not None:
                detections.extend(self._detections_from_mask(self._apply_morphology(channel_mask)))

        if self.config.enable_circle_fallback:
            detections.extend(self._detections_from_circles(image, color))

        return detections

    def _detections_from_mask(self, mask: Optional[np.ndarray]) -> List[MarkerCandidate]:
        if mask is None or np.count_nonzero(mask) == 0:
            return []

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections: List[MarkerCandidate] = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.config.min_area or area > self.config.max_area:
                continue

            m = cv2.moments(cnt)
            if m["m00"] == 0:
                continue

            x = int(m["m10"] / m["m00"])
            y = int(m["m01"] / m["m00"])

            (_, _), radius = cv2.minEnclosingCircle(cnt)

            detections.append(MarkerCandidate(x=x, y=y, area=float(area), radius=float(radius)))

        return detections

    def _detections_from_circles(self, image: np.ndarray, color: str) -> List[MarkerCandidate]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        params = self.config
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=params.circle_dp,
            minDist=params.circle_min_distance,
            param1=params.circle_param1,
            param2=params.circle_param2,
            minRadius=params.circle_min_radius,
            maxRadius=params.circle_max_radius,
        )

        if circles is None:
            return []

        h, w = image.shape[:2]
        results: List[MarkerCandidate] = []

        for x, y, r in np.round(circles[0]).astype(int):
            if x < 0 or y < 0 or x >= w or y >= h:
                continue

            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.circle(mask, (x, y), r, 255, -1)

            b, g, r_val = cv2.mean(image, mask=mask)[:3]

            if color == "red":
                score = r_val - max(b, g)
            elif color == "blue":
                score = b - max(r_val, g)
            else:
                continue

            if score < self.config.channel_margin:
                continue

            results.append(MarkerCandidate(x=x, y=y, area=float(np.pi * r * r), radius=float(r)))

        return results

    def _mask_from_hsv(self, image: np.ndarray, ranges: List[ColorRange]) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask: Optional[np.ndarray] = None
        for lower, upper in ranges:
            current = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            mask = current if mask is None else cv2.bitwise_or(mask, current)
        return self._apply_morphology(mask)

    def _apply_morphology(self, mask: np.ndarray) -> np.ndarray:
        kernel_size = max(1, int(self.config.morph_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _centroid_from_mask(self, mask: Optional[np.ndarray]) -> Optional[Tuple[int, int]]:
        if mask is None or np.count_nonzero(mask) == 0:
            return None

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.config.min_area or area > self.config.max_area:
            return None

        m = cv2.moments(largest)
        if m["m00"] == 0:
            return None
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])
        return u, v

    def _channel_dominance_mask(self, image: np.ndarray, color: str) -> Optional[np.ndarray]:
        b = image[:, :, 0].astype(np.int16)
        g = image[:, :, 1].astype(np.int16)
        r = image[:, :, 2].astype(np.int16)
        margin = self.config.channel_margin
        channel_min = self.config.channel_min

        if color == "red":
            dominant = r - np.maximum(b, g)
            mask = np.where((dominant >= margin) & (r >= channel_min), 255, 0)
        elif color == "blue":
            dominant = b - np.maximum(r, g)
            mask = np.where((dominant >= margin) & (b >= channel_min), 255, 0)
        else:
            return None

        mask = np.asarray(mask, dtype=np.uint8)
        return mask if np.count_nonzero(mask) else None

    def _select_best(self, candidates: List[MarkerCandidate]) -> Optional[MarkerCandidate]:
        if not candidates:
            return None
        return max(candidates, key=lambda c: c.area)
