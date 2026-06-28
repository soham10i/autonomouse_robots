"""Depth (range-finder) image -> auxiliary floating-wall obstacle points.

The 2D lidar sits at ``LIDAR_MOUNT_Z`` (0.10 m) and its rays pass *over* walls
whose top edge is below that plane.  The depth camera at ``CAMERA_MOUNT_Z``
(0.165 m) sees them.  We back-project depth pixels into the robot frame and keep
only those in the height band ``[AUX_Z_MIN, AUX_Z_MAX]`` and within
``AUX_MAX_RANGE`` — these are the low/floating walls the robot would actually
collide with.  Pixels above ``ROBOT_HEIGHT`` are passages the robot drives under
and are intentionally excluded.

Pinhole model: ``x`` forward = depth, ``y`` left = -(u - cx) * d / fx,
``z`` up = -(v - cy) * d / fy + mount_z.  Webots range images are Z-depth along
the optical axis in metres.
"""
from __future__ import annotations

import math

import numpy as np

import settings as S
from geometry import transform_points


class DepthModel:
    def __init__(self, width, height, fov, mount_z=None, depth_min=0.05, depth_max=10.0):
        self.w = int(width)
        self.h = int(height)
        self.fov = float(fov)
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.fy = self.fx
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.mount_z = S.CAMERA_MOUNT_Z if mount_z is None else mount_z
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

    def aux_points_world(self, depth, pose):
        """World-frame (M, 2) thin virtual-scan of floating-wall obstacles.

        This is a ``depthimage_to_laserscan``-style collapse: for each image
        COLUMN (bearing) we keep only the *nearest* obstacle whose height is in
        the collision band ``[AUX_Z_MIN, AUX_Z_MAX]``.  That yields one point per
        bearing — a thin wall line at the surface — instead of dumping the whole
        wall band (which smeared into the fat purple blobs that narrowed every
        passage).  Heights above ``AUX_Z_MAX`` (== ROBOT_HEIGHT) are ignored, so
        walls the robot can drive *under* are correctly treated as passable.
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        _, _, z_r = self.to_robot_frame(depth)
        valid = (
            np.isfinite(depth)
            & (depth > self.depth_min)
            & (depth < S.AUX_MAX_RANGE)
            & (z_r > S.AUX_Z_MIN)
            & (z_r < S.AUX_Z_MAX)
        )
        # nearest valid in-band depth per column -> one range per bearing
        d = np.where(valid, depth, np.inf)
        col_min = d.min(axis=0)                       # (w,)
        cols = np.nonzero(np.isfinite(col_min))[0]
        if cols.size == 0:
            return np.empty((0, 2))
        rng = col_min[cols].astype(np.float64)
        u = cols.astype(np.float64)
        x_b = rng                                     # forward = depth
        y_b = -(u - self.cx) * rng / self.fx          # left
        body = np.stack([x_b, y_b], axis=1)
        return transform_points(body, pose[0], pose[1], pose[2])

    def depth_rays_world(self, depth, pose, clear_range):
        """Depth -> a 2D scan for the occupancy grid's depth obstacle layer.

        Returns ``(hits_world, free_ends_world)``:

        * ``hits_world`` (M, 2): for each image COLUMN whose nearest in-band
          (collision-height ``[AUX_Z_MIN, AUX_Z_MAX]``) obstacle is within
          ``clear_range``, the world endpoint of that obstacle (to be MARKED).
        * ``free_ends_world`` (K, 2): for each column that saw something but has
          NO in-band obstacle within ``clear_range``, a world point at
          ``clear_range`` along that bearing (to raytrace-CLEAR the robot-height
          tunnel — this is what makes the depth layer self-correct).

        This is the ``depthimage_to_laserscan`` projection: one range per bearing,
        split into "obstacle" vs "clear" so the grid can both mark and clear.
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        _, _, z_r = self.to_robot_frame(depth)
        valid_any = (np.isfinite(depth) & (depth > self.depth_min)
                     & (depth < self.depth_max))
        in_band = valid_any & (z_r > S.AUX_Z_MIN) & (z_r < S.AUX_Z_MAX)
        d = np.where(in_band, depth, np.inf)
        d_obs = d.min(axis=0)                          # (w,) nearest in-band depth
        has_any = valid_any.any(axis=0)               # (w,) column saw anything
        u = np.arange(self.w, dtype=np.float64)
        diry = -(u - self.cx) / self.fx               # body y per unit x (bearing)

        hit_cols = np.isfinite(d_obs) & (d_obs <= clear_range)
        hits = np.empty((0, 2))
        if hit_cols.any():
            r = d_obs[hit_cols]
            body = np.stack([r, diry[hit_cols] * r], axis=1)
            hits = transform_points(body, pose[0], pose[1], pose[2])

        clear_cols = has_any & ~hit_cols              # saw something, no in-band obs
        frees = np.empty((0, 2))
        if clear_cols.any():
            k = int(np.count_nonzero(clear_cols))
            dd = np.stack([np.ones(k), diry[clear_cols]], axis=1)
            dd /= np.hypot(dd[:, 0], dd[:, 1])[:, None]
            frees = transform_points(dd * clear_range, pose[0], pose[1], pose[2])
        return hits, frees
