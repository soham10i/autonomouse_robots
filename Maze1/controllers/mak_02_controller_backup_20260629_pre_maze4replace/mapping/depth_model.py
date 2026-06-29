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

    def thin_scan(self, depth):
        """Per-column nearest in-band obstacle, AND clear-sight-line info.

        This is the Maze4-proven floating-wall detector (ported verbatim).  It
        returns four length-``w`` arrays (one entry per image COLUMN/bearing):

          * ``bearings[i]``   — body-frame bearing of column ``i``.
          * ``hit_mask[i]``   — True if an in-band (collision-height) obstacle was
            found in that column; its range is ``hit_ranges[i]``.
          * ``clear_mask[i]`` — True if the sight-line along that column is clear
            of in-band obstacles out to ``AUX_MAX_RANGE`` (every valid depth pixel
            is either out of range or outside the collision band) → safe to
            raytrace-CLEAR the aux layer along it.

        A column can be neither (all readings invalid/NaN) — callers skip those.
        Pairing one-range-per-bearing HITS with explicit CLEAR sight-lines is what
        lets ``OccupancyGrid.integrate_aux`` raytrace-clear stale marks every tick
        (a boolean self-correcting layer) instead of the log-odds depth layer that
        flickered on/off and let the robot drive into WallShort(5).
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        _, _, z_r = self.to_robot_frame(depth)
        finite = np.isfinite(depth) & (depth > self.depth_min)
        in_band = finite & (z_r > S.AUX_Z_MIN) & (z_r < S.AUX_Z_MAX)
        within_range = depth < S.AUX_MAX_RANGE

        hit_candidate = in_band & within_range
        d_hit = np.where(hit_candidate, depth, np.inf)
        col_min = d_hit.min(axis=0)                          # (w,)
        hit_mask = np.isfinite(col_min)
        hit_ranges = np.where(hit_mask, col_min, 0.0).astype(np.float64)

        # A column is "clear" if it has SOME real signal (not all NaN) and no
        # in-band candidate landed within AUX_MAX_RANGE.  A column whose nearest
        # valid reading is itself beyond AUX_MAX_RANGE still proves the near zone
        # is obstacle-free, so it counts as clear (must NOT require the reading
        # itself to be within range).
        any_valid = finite.any(axis=0)
        clear_mask = any_valid & ~hit_mask

        u = np.arange(self.w, dtype=np.float64)
        bearings = np.arctan2(-(u - self.cx) / self.fx, 1.0)  # body-frame bearing
        return bearings, hit_ranges, hit_mask, clear_mask

    def forward_obstacle(self, depth, half_angle=None, z_min=None, z_max=None):
        """Diagnostic probe: what does the depth camera see straight ahead?

        Looks only at image columns within ``half_angle`` of forward and reports,
        for that cone: how many pixels are valid, how many fall in the collision
        height band ``[z_min, z_max]``, the nearest valid range, and the nearest
        IN-BAND range + its height.  This answers the key question behind the
        floating-wall fault: *is the depth cam even seeing the wall ahead, and is
        it at collision height?* — separating a sensor-blind-spot cause from a
        mapping/planning cause.  Returns a plain dict (JSON-friendly).
        """
        ha = float(S.DIAG_FWD_CONE_RAD if half_angle is None else half_angle)
        z_lo = float(S.AUX_Z_MIN if z_min is None else z_min)
        z_hi = float(S.AUX_Z_MAX if z_max is None else z_max)
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        bearing = np.arctan(-(self._u - self.cx) / self.fx)      # (1, W)
        colsel = np.abs(bearing[0]) <= ha                        # (W,)
        out = {"n_valid_fwd": 0, "n_inband_fwd": 0, "min_range": float("inf"),
               "min_inband_range": float("inf"), "min_inband_height": float("nan")}
        if not colsel.any():
            return out
        sub = depth[:, colsel]
        with np.errstate(invalid="ignore"):
            z_r = -(self._v - self.cy) * sub / self.fy + self.mount_z   # (H, Wsel)
        valid = (np.isfinite(sub) & (sub > self.depth_min) & (sub < self.depth_max))
        n_valid = int(valid.sum())
        out["n_valid_fwd"] = n_valid
        if n_valid:
            out["min_range"] = float(sub[valid].min())
        in_band = valid & (z_r > z_lo) & (z_r < z_hi)
        out["n_inband_fwd"] = int(in_band.sum())
        if in_band.any():
            masked = np.where(in_band, sub, np.inf)
            k = int(np.argmin(masked))
            out["min_inband_range"] = float(masked.ravel()[k])
            out["min_inband_height"] = float(z_r.ravel()[k])
        return out

    def cloud_world(self, depth, pose, stride=None, max_range=None,
                    z_min=None, z_max=None):
        """Subsampled world-frame ``(M, 3)`` point cloud from the depth image.

        Back-projects every ``stride``-th pixel through the pinhole model, keeps
        the finite in-range points within the height band, and transforms them to
        the world frame using ``pose``.  This is the continuous 3D accumulation
        feed (``CloudMap.add_points_world``) and mirrors the teleop_mapping cloud.
        """
        stride = int(S.CLOUD_STRIDE if stride is None else stride)
        max_range = float(S.CLOUD_MAX_RANGE if max_range is None else max_range)
        z_lo = float(S.CLOUD_Z_MIN if z_min is None else z_min)
        z_hi = float(S.CLOUD_Z_MAX if z_max is None else z_max)
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        d = depth[::stride, ::stride]
        u = self._u[:, ::stride]
        v = self._v[::stride, :]
        with np.errstate(invalid="ignore"):
            x_r = d
            y_r = -(u - self.cx) * d / self.fx
            z_r = -(v - self.cy) * d / self.fy + self.mount_z
        valid = (np.isfinite(d) & (d > self.depth_min) & (d < max_range)
                 & (z_r > z_lo) & (z_r < z_hi))
        if not valid.any():
            return np.empty((0, 3), dtype=np.float64)
        xr = x_r[valid].astype(np.float64)
        yr = y_r[valid].astype(np.float64)
        zr = z_r[valid].astype(np.float64)
        rx, ry, rth = pose
        c, s = math.cos(rth), math.sin(rth)
        wx = rx + c * xr - s * yr
        wy = ry + s * xr + c * yr
        return np.stack([wx, wy, zr], axis=1)

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
