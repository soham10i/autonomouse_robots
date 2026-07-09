"""Depth (range-finder) image -> thin auxiliary obstacle scan.

The 2-D lidar sits at a single plane (``LIDAR_MOUNT_Z``) and several Maze4
wall panels are partially or fully outside it (see config.py).  The depth
camera at ``CAMERA_MOUNT_Z`` sees the full vertical extent in front of the
robot.  We back-project depth pixels into the robot frame and, for each image
COLUMN (bearing), keep only the nearest pixel whose height falls in the
collision band ``[AUX_Z_MIN, AUX_Z_MAX]`` -- one point per bearing, exactly the
``depthimage_to_laserscan``-style thin scan that fixed the "fat purple blob"
bug in the original Maze1 attempt (this is the same projection maths, ported
verbatim from ``Maze1/controllers/mak_02_controller/mapping/depth_model.py``).

Unlike that Maze1 module, this one also reports, for EVERY column, whether the
sight-line was clear out to ``AUX_MAX_RANGE`` (no in-band pixel found) so the
caller can raytrace-CLEAR stale aux marks every tick instead of relying on
sticky-hit counters and decay heuristics (the actual root cause fixed here --
see config.py's AUX_* comment and mapping.OccupancyGrid.integrate_aux()).

Pinhole model: ``x`` forward = depth, ``y`` left = -(u - cx) * d / fx,
``z`` up = -(v - cy) * d / fy + mount_z.  Webots range images are Z-depth along
the optical axis in metres.
"""
from __future__ import annotations

import math

import numpy as np

import config as C


class DepthModel:
    """Projects a depth image into a body-frame thin-scan of low-panel obstacle hits inside the robot's collision height band."""
    def __init__(self, width, height, fov, mount_z=None, depth_min=0.05, depth_max=10.0):
        self.w = int(width)
        self.h = int(height)
        self.fov = float(fov)
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.fy = self.fx
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.mount_z = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self.depth_min = depth_min
        self.depth_max = depth_max
        self._u = np.arange(self.w, dtype=np.float32).reshape(1, -1)
        self._v = np.arange(self.h, dtype=np.float32).reshape(-1, 1)

    def to_robot_frame(self, depth):
        """Return ``(x_r, y_r, z_r)`` arrays (H, W) in the robot frame."""
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        with np.errstate(invalid="ignore"):
            x_r = depth
            y_r = -(self._u - self.cx) * depth / self.fx
            z_r = -(self._v - self.cy) * depth / self.fy + self.mount_z
        return x_r, y_r, z_r

    def thin_scan(self, depth):
        """Per-column nearest in-band obstacle, AND clear-sight-line info.

        Returns ``(bearings, hit_ranges, hit_mask, clear_mask)`` -- all length
        ``w`` (one entry per image column):
          * ``hit_mask[i]``   True  -> an in-band obstacle was found; its range
            is ``hit_ranges[i]`` and its bearing is ``bearings[i]``.
          * ``clear_mask[i]`` True  -> the sight-line along that column is
            clear of in-band obstacles out to ``AUX_MAX_RANGE`` (i.e. every
            valid depth pixel in that column is either out of range or outside
            the collision height band) -> safe to raytrace-clear aux there.
          A column can be neither (e.g. all readings invalid/NaN) -- callers
          should skip those rather than treat them as a clear or a hit.
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        _, _, z_r = self.to_robot_frame(depth)
        finite = np.isfinite(depth) & (depth > self.depth_min)
        in_band = finite & (z_r > C.AUX_Z_MIN) & (z_r < C.AUX_Z_MAX)
        within_range = depth < C.AUX_MAX_RANGE

        hit_candidate = in_band & within_range
        d_hit = np.where(hit_candidate, depth, np.inf)
        col_min = d_hit.min(axis=0)                          # (w,)
        hit_mask = np.isfinite(col_min)
        hit_ranges = np.where(hit_mask, col_min, 0.0).astype(np.float64)

        # A column is "clear" if it has SOME real signal (not all NaN/garbage)
        # and no in-band candidate landed within AUX_MAX_RANGE.  A column whose
        # nearest valid reading is itself beyond AUX_MAX_RANGE (e.g. a far wall)
        # still proves the near zone is obstacle-free, so it counts as clear --
        # it must NOT require that reading to itself be within range.
        any_valid = finite.any(axis=0)
        clear_mask = any_valid & ~hit_mask

        u = np.arange(self.w, dtype=np.float64)
        bearings = np.arctan2(-(u - self.cx) / self.fx, 1.0)  # body-frame bearing per column
        return bearings, hit_ranges, hit_mask, clear_mask
