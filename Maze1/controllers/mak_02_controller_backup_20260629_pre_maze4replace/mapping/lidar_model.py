"""Lidar range image -> body-frame point cloud (pure math).

Webots' ``Lidar.getRangeImage()`` returns a 1-D array of ``N`` ranges.  Beam
``i`` points at angle ``theta_i = fov/2 - i * fov/(N-1)`` in the sensor frame
(CCW from +x), matching Webots' convention.  Body frame here is identical to the
sensor frame in x/y (the lidar is mounted on the robot's axis).
"""
from __future__ import annotations

import numpy as np

import settings as S


class LidarModel:
    def __init__(self, n_beams, fov, r_min=None, r_max=None):
        self.n = int(n_beams)
        self.fov = float(fov)
        self.r_min = S.LIDAR_RANGE_MIN if r_min is None else max(r_min, 0.02)
        self.r_max = S.LIDAR_RANGE_MAX if r_max is None else min(r_max, 30.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self.angles = start - step * np.arange(self.n)

    def ranges_to_body(self, ranges):
        """Return ``(pts_body (M, 2), valid_mask (N,))`` for in-range beams."""
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.size != self.n:
            return np.empty((0, 2)), np.zeros(self.n, dtype=bool)
        valid = np.isfinite(ranges) & (ranges > self.r_min) & (ranges < self.r_max)
        a = self.angles[valid]
        r = ranges[valid]
        x = r * np.cos(a)
        y = r * np.sin(a)
        return np.stack([x, y], axis=1), valid
