"""Colour-blob detection and geometry helpers.

This module has no ROS imports on purpose, so it can be unit-tested anywhere
(``pytest`` in CI) and reused outside ROS.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

# HSV ranges (OpenCV scale: H 0-179, S/V 0-255). Red wraps around 0/180.
COLOR_RANGES = {
    'red': [((0, 120, 50), (10, 255, 255)), ((170, 120, 50), (180, 255, 255))],
    'green': [((40, 120, 40), (85, 255, 255))],
    'blue': [((100, 120, 40), (130, 255, 255))],
}


@dataclass
class Blob:
    """A detected coloured region in the image (pixel coordinates)."""

    color: str
    x: int
    y: int
    w: int
    h: int
    area: float

    @property
    def cx(self):
        return self.x + self.w / 2.0

    @property
    def cy(self):
        return self.y + self.h / 2.0


def find_blobs(bgr, min_area=60.0, colors=None):
    """Return the coloured blobs found in a BGR image, largest first."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    blobs = []
    for color in (colors or COLOR_RANGES):
        mask = None
        for lo, hi in COLOR_RANGES[color]:
            part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            blobs.append(Blob(color, x, y, w, h, float(area)))
    return sorted(blobs, key=lambda b: -b.area)


def pixel_to_bearing(u, cx, fx):
    """Horizontal angle [rad] of pixel column ``u``; positive = left of the optical axis."""
    return math.atan2(-(u - cx), fx)


def range_at_bearing(ranges, angle_min, angle_increment, bearing, window_rad,
                     range_min, range_max):
    """Smallest valid LiDAR range within ``bearing +/- window_rad`` (None if no hit)."""
    n = len(ranges)
    first = int(round((bearing - window_rad - angle_min) / angle_increment))
    last = int(round((bearing + window_rad - angle_min) / angle_increment))
    idx = np.arange(first, last + 1)
    idx = idx[(idx >= 0) & (idx < n)]
    if idx.size == 0:
        return None
    values = np.asarray(ranges, dtype=float)[idx]
    values = values[np.isfinite(values) & (values >= range_min) & (values <= range_max)]
    if values.size == 0:
        return None
    return float(values.min())


def transform_point(translation, quaternion, point):
    """Apply a rigid transform (translation + unit quaternion x,y,z,w) to a 3D point."""
    qx, qy, qz, qw = quaternion
    q = np.array([qx, qy, qz], dtype=float)
    v = np.asarray(point, dtype=float)
    t = 2.0 * np.cross(q, v)
    rotated = v + qw * t + np.cross(q, t)
    return tuple(rotated + np.asarray(translation, dtype=float))


@dataclass
class Track:
    """A persistent object estimate in the target frame."""

    label: str
    x: float
    y: float
    hits: int = 1


class ObjectTracker:
    """Merges repeated detections of the same object into one running-average position."""

    def __init__(self, merge_distance=0.6, min_hits=3, max_weight=50):
        self.merge_distance = merge_distance
        self.min_hits = min_hits
        self.max_weight = max_weight
        self.tracks = []

    def update(self, label, x, y):
        best = None
        best_dist = self.merge_distance
        for track in self.tracks:
            if track.label != label:
                continue
            dist = math.hypot(track.x - x, track.y - y)
            if dist <= best_dist:
                best, best_dist = track, dist
        if best is None:
            best = Track(label, x, y, 1)
            self.tracks.append(best)
            return best
        w = min(best.hits, self.max_weight)
        best.x = (best.x * w + x) / (w + 1)
        best.y = (best.y * w + y) / (w + 1)
        best.hits += 1
        return best

    def confirmed(self):
        return [t for t in self.tracks if t.hits >= self.min_hits]
