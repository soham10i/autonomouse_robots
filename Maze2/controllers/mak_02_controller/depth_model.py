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
see config.py's AUX_* comment and mapping.OccupancyGrid.integrate_depth_obstacles()).

Pinhole model: ``x`` forward = depth, ``y`` left = -(u - cx) * d / fx,
``z`` up = -(v - cy) * d / fy + mount_z.  Webots range images are Z-depth along
the optical axis in metres.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

import config as C


class DepthModel:
    """Projects a depth image into a body-frame thin-scan of low-panel obstacle hits inside the robot's collision height band."""
    def __init__(self, width: int, height: int, fov: float, mount_z: Optional[float] = None, depth_min: float = 0.05, depth_max: float = 10.0) -> None:
        """Initializes the depth model with camera intrinsic parameters.

        Args:
            width (int): The width of the depth image in pixels.
            height (int): The height of the depth image in pixels.
            fov (float): The horizontal field of view in radians.
            mount_z (Optional[float], optional): The camera mount height in meters. Defaults to `config.CAMERA_MOUNT_Z`.
            depth_min (float, optional): Minimum valid depth in meters. Defaults to 0.05.
            depth_max (float, optional): Maximum valid depth in meters. Defaults to 10.0.
        """
        self.w: int = int(width)
        self.h: int = int(height)
        self.fov: float = float(fov)
        self.fx: float = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.fy: float = self.fx
        self.cx: float = 0.5 * self.w
        self.cy: float = 0.5 * self.h
        self.mount_z: float = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self.depth_min: float = depth_min
        self.depth_max: float = depth_max
        self._u: np.ndarray = np.arange(self.w, dtype=np.float32).reshape(1, -1)
        self._v: np.ndarray = np.arange(self.h, dtype=np.float32).reshape(-1, 1)

    def to_robot_frame(self, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Projects the depth image into the robot's body frame coordinates.

        Args:
            depth (np.ndarray): The raw depth image array of shape (H, W).

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: Three arrays `(x_r, y_r, z_r)`, each of shape (H, W), representing the 3D coordinates in the robot frame.
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        with np.errstate(invalid="ignore"):
            x_r = depth
            y_r = -(self._u - self.cx) * depth / self.fx
            z_r = -(self._v - self.cy) * depth / self.fy + self.mount_z
        return x_r, y_r, z_r

    def thin_scan(self, depth: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Extracts a single-line obstacle scan from the depth image for the robot's collision band.

        Evaluates per-column nearest in-band obstacle and clear sight-line info.

        Args:
            depth (np.ndarray): The raw depth image array of shape (H, W).

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: Four arrays of length W:
                - `bearings` (np.ndarray): The horizontal bearing of each column in the robot frame.
                - `hit_ranges` (np.ndarray): The distance to the nearest in-band obstacle per column, or 0.0 if none.
                - `hit_mask` (np.ndarray): Boolean mask indicating if an in-band obstacle was found.
                - `clear_mask` (np.ndarray): Boolean mask indicating if the column is clear of in-band obstacles out to `AUX_MAX_RANGE`.
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

        any_valid = finite.any(axis=0)
        clear_mask = any_valid & ~hit_mask

        u = np.arange(self.w, dtype=np.float64)
        bearings = np.arctan2(-(u - self.cx) / self.fx, 1.0)  # body-frame bearing per column
        return bearings, hit_ranges, hit_mask, clear_mask
