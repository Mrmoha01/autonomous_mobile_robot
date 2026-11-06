import math

import cv2
import numpy as np

from amr_perception.detection import (ObjectTracker, find_blobs, pixel_to_bearing,
                                      range_at_bearing, transform_point)


def make_image():
    img = np.full((240, 320, 3), 190, np.uint8)  # light grey background
    cv2.circle(img, (80, 120), 20, (0, 0, 220), -1)        # red (BGR)
    cv2.rectangle(img, (150, 100), (190, 140), (220, 40, 20), -1)  # blue
    cv2.rectangle(img, (240, 90), (270, 150), (20, 180, 30), -1)   # green
    return img


def test_find_blobs_colors_and_positions():
    blobs = find_blobs(make_image(), min_area=60)
    by_color = {b.color: b for b in blobs}
    assert set(by_color) == {'red', 'blue', 'green'}
    assert abs(by_color['red'].cx - 80) < 3
    assert abs(by_color['blue'].cx - 170) < 3
    assert abs(by_color['green'].cx - 255) < 3


def test_min_area_filters_small_blobs():
    img = np.full((240, 320, 3), 190, np.uint8)
    cv2.circle(img, (100, 100), 3, (0, 0, 220), -1)
    assert find_blobs(img, min_area=60) == []


def test_grey_scene_has_no_blobs():
    img = np.full((240, 320, 3), 120, np.uint8)
    assert find_blobs(img) == []


def test_bearing_sign_and_centre():
    assert abs(pixel_to_bearing(160, 160, 267)) < 1e-9
    assert pixel_to_bearing(100, 160, 267) > 0   # left of centre -> positive
    assert pixel_to_bearing(220, 160, 267) < 0   # right of centre -> negative


def test_range_at_bearing_picks_nearest_in_window():
    n = 360
    inc = 2 * math.pi / n
    angle_min = -math.pi
    ranges = [5.0] * n
    ranges[180] = 2.0                      # bearing 0 rad
    r = range_at_bearing(ranges, angle_min, inc, 0.0, math.radians(2.5), 0.12, 8.0)
    assert r == 2.0


def test_range_at_bearing_ignores_inf_and_out_of_range():
    n = 360
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    assert range_at_bearing(ranges, -math.pi, inc, 0.0, 0.05, 0.12, 8.0) is None
    ranges[180] = 0.05  # below range_min
    assert range_at_bearing(ranges, -math.pi, inc, 0.0, 0.05, 0.12, 8.0) is None


def test_transform_point_yaw_90_and_translation():
    s = math.sqrt(0.5)
    x, y, z = transform_point((1.0, 2.0, 0.0), (0.0, 0.0, s, s), (1.0, 0.0, 0.0))
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 3.0) < 1e-9
    assert abs(z) < 1e-9


def test_tracker_merges_and_confirms():
    tracker = ObjectTracker(merge_distance=0.6, min_hits=3)
    for dx in (0.00, 0.05, -0.05):
        tracker.update('ball', 2.0 + dx, 1.0)
    tracker.update('ball', 4.0, 4.0)   # far away -> separate object
    tracker.update('box', 2.0, 1.0)    # same place, other label -> separate object
    assert len(tracker.tracks) == 3
    confirmed = tracker.confirmed()
    assert len(confirmed) == 1
    assert confirmed[0].label == 'ball'
    assert abs(confirmed[0].x - 2.0) < 0.05
