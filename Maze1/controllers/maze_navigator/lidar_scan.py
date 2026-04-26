"""Lidar → world-frame point cloud.

The RpLidarA2 reports a 1-D range image of length `N = horizontalResolution`.
Ray i has angle `θ_i = fov/2 − i·fov/(N−1)` in the sensor frame (CCW from +x),
matching Webots' convention for lidar scans.
"""
import math
import numpy as np
import config as C


class ScanProcessor:
    def __init__(self, lidar):
        self.lidar = lidar
        self.n = lidar.getHorizontalResolution()
        self.fov = lidar.getFov()
        self.r_min = max(lidar.getMinRange(), 0.05)
        self.r_max = min(lidar.getMaxRange(), 8.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self._angles = start - step * np.arange(self.n)

    def scan_world(self, pose):
        """Return an (M, 2) array of world-frame (x, y) hits from a pose (x, y, θ).
        Drops rays out of range or reporting NaN/inf."""
        ranges = np.asarray(self.lidar.getRangeImage(), dtype=np.float32)
        if ranges.size != self.n:
            return np.empty((0, 2), dtype=np.float32), ranges
        valid = np.isfinite(ranges) & (ranges > self.r_min) & (ranges < self.r_max)
        a = self._angles[valid]
        r = ranges[valid]
        x, y, th = pose
        lx = r * np.cos(a)
        ly = r * np.sin(a)
        c, s = math.cos(th), math.sin(th)
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        return np.stack([wx, wy], axis=1), ranges

    @property
    def horizontal_resolution(self):
        return self.n
