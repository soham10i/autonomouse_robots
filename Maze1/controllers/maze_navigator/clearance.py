"""Depth-camera 3D clearance check for overhead / floating-wall obstacles.

The 2D lidar at LIDAR_MOUNT_Z (0.10 m) can't see walls whose lower edge is
above the lidar plane but below the chassis top (0.22 m). The depth camera
at CAMERA_MOUNT_Z (0.165 m) does see them. This module:

  1. Reports the minimum forward clearance at chassis height for `_safe_cmd`.
  2. Projects the depth image into a world-frame point cloud for the final
     3D visualization (one sample every few pixels, subsampled in time).

Camera / robot frame assumptions:
  - Webots RangeFinder `getRangeImage()` returns Z-depth along the optical
    axis (not Euclidean), in meters.
  - Image u is right, v is down (standard CV convention).
  - Robot frame: +x forward, +y left, +z up. Camera is aligned with robot,
    translated up by CAMERA_MOUNT_Z, no pitch/roll.
"""
import math
import numpy as np

import config as C


class ClearanceChecker:
    def __init__(self, camera_depth, camera_rgb):
        self.depth = camera_depth
        self.ok = camera_depth is not None and camera_rgb is not None
        if not self.ok:
            self.w = 0
            self.h = 0
            return
        self.w = camera_rgb.getWidth()
        self.h = camera_rgb.getHeight()
        fov = camera_rgb.getFov()
        self.fx = 0.5 * self.w / math.tan(0.5 * fov)
        self.fy = self.fx
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        try:
            self.depth_min = float(camera_depth.getMinRange())
            self.depth_max = float(camera_depth.getMaxRange())
        except Exception:
            self.depth_min = 0.05
            self.depth_max = 10.0
        self._u_grid = np.arange(self.w, dtype=np.float32).reshape(1, -1)
        self._v_grid = np.arange(self.h, dtype=np.float32).reshape(-1, 1)
        print(f"[clearance] depth fx={self.fx:.1f} "
              f"range=[{self.depth_min:.2f},{self.depth_max:.2f}] m")

    def _depth_image(self):
        if not self.ok or self.depth is None:
            return None
        try:
            raw = self.depth.getRangeImage()
            if raw is None:
                return None
            return np.asarray(raw, dtype=np.float32).reshape(self.h, self.w)
        except Exception:
            return None

    def _robot_frame(self, d):
        """(x_r, y_r, z_r) in robot frame for every depth pixel.
        NaN depths are silently ignored here — callers always apply the
        `valid = np.isfinite(d)` mask before using the results.
        """
        with np.errstate(invalid="ignore"):
            x_r = d
            y_r = -(self._u_grid - self.cx) * d / self.fx
            z_r = -(self._v_grid - self.cy) * d / self.fy + C.CAMERA_MOUNT_Z
        return x_r, y_r, z_r

    def min_forward_at_chassis_height(self,
                                      half_angle_rad=math.radians(25.0),
                                      max_forward=0.9,
                                      height_lo=None,
                                      height_hi=None):
        """Return min forward distance of any obstacle at chassis height
        inside the forward cone. +inf if the cone is clear.

        height_lo defaults to LIDAR_MOUNT_Z so we only cover the blind zone
        that the 2D lidar already handles up to that height.
        height_hi defaults to ROBOT_HEIGHT + 5 cm margin.
        """
        if height_lo is None:
            height_lo = C.LIDAR_MOUNT_Z        # don't duplicate lidar's job
        if height_hi is None:
            height_hi = C.ROBOT_HEIGHT + 0.05  # top of chassis + margin
        d = self._depth_image()
        if d is None:
            return float("inf")
        x_r, y_r, z_r = self._robot_frame(d)
        valid = np.isfinite(d) & (d > self.depth_min) & (d < max_forward)
        bearing = np.arctan2(y_r, np.where(x_r > 1e-3, x_r, 1e-3))
        in_cone = np.abs(bearing) < half_angle_rad
        at_chassis = (z_r > height_lo) & (z_r < height_hi)
        mask = valid & in_cone & at_chassis
        if not mask.any():
            return float("inf")
        return float(np.min(d[mask]))

    def depth_to_world_cloud(self, pose, stride=6,
                             max_range=4.5, min_height=-0.05, max_height=2.0):
        """Return (M, 3) world-frame points from the current depth image.

        stride : subsampling on u and v (stride=6 → ~54x40 points / frame
                 at 320x240, i.e. ~2k).
        """
        if not self.ok:
            return np.empty((0, 3), dtype=np.float32)
        d_full = self._depth_image()
        if d_full is None:
            return np.empty((0, 3), dtype=np.float32)
        d = d_full[::stride, ::stride]
        u = self._u_grid[:, ::stride]
        v = self._v_grid[::stride, :]
        with np.errstate(invalid="ignore"):
            x_r = d
            y_r = -(u - self.cx) * d / self.fx
            z_r = -(v - self.cy) * d / self.fy + C.CAMERA_MOUNT_Z
        valid = (np.isfinite(d) & (d > self.depth_min) & (d < max_range)
                 & (z_r > min_height) & (z_r < max_height))
        if not valid.any():
            return np.empty((0, 3), dtype=np.float32)
        xr = x_r[valid]
        yr = y_r[valid]
        zr = z_r[valid]
        rx, ry, rth = pose
        c, s = math.cos(rth), math.sin(rth)
        wx = rx + c * xr - s * yr
        wy = ry + s * xr + c * yr
        return np.stack([wx, wy, zr], axis=1).astype(np.float32)
