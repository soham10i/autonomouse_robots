"""Depth (range-finder) image → thin auxiliary obstacle scan.

The 2-D lidar sits at a single plane (``LIDAR_MOUNT_Z``) and several Maze3
wall panels are partially or fully outside it (see :mod:`config`).  The depth
camera at ``CAMERA_MOUNT_Z`` sees the full vertical extent in front of the
robot.  We back-project depth pixels into the robot frame and, for each image
COLUMN (bearing), keep only the nearest pixel whose height falls in the
collision band ``[AUX_Z_MIN, AUX_Z_MAX]`` — one point per bearing, exactly the
``depthimage_to_laserscan``-style thin scan that fixed the "fat purple blob"
bug in the original Maze1 attempt (this is the same projection maths, ported
verbatim from ``Maze1/controllers/mak_03_controller/mapping/depth_model.py``).

Unlike that Maze1 module, this one also reports, for EVERY column, whether the
sight-line was clear out to ``AUX_MAX_RANGE`` (no in-band pixel found).  The
caller (:meth:`mapping.OccupancyGrid.integrate_depth_obstacles`) uses the hits
to reinforce a PERSISTENT per-cell confidence layer and the clears only to DECAY
it — the decay never reaches inside the ``< depth_min`` blind shell, so a
floating wall the robot has closed in on is NOT forgotten (see ``config.py``'s
``AUX_*`` comment).

Pinhole model: ``x`` forward = depth, ``y`` left = ``-(u − cx) · d / fx``,
``z`` up = ``-(v − cy) · d / fy + mount_z``.  Webots range images are Z-depth
along the optical axis in metres.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np
import numpy.typing as npt

import config as C


class DepthModel:
    """Projects a depth image into a body-frame thin-scan of low-panel obstacles.

    Extracts obstacle hits inside the robot's collision height band from the
    depth camera, producing one range measurement per image column — a virtual
    1-D lidar scan that covers the vertical extent the 2-D lidar misses.

    Attributes:
        w: Image width in pixels.
        h: Image height in pixels.
        fov: Horizontal field-of-view (radians).
        fx: Horizontal focal length (pixels).
        fy: Vertical focal length (pixels, assumed equal to *fx*).
        cx: Horizontal principal point (pixels).
        cy: Vertical principal point (pixels).
        mount_z: Camera optical-centre height above the floor (metres).
        depth_min: Minimum trustworthy depth reading (metres).
        depth_max: Maximum depth reading used (metres).
    """

    def __init__(self, width: int, height: int, fov: float,
                 mount_z: float | None = None,
                 depth_min: float = 0.05,
                 depth_max: float = 10.0) -> None:
        """Initialise the pinhole camera model from sensor specifications.

        Args:
            width: Depth image width (pixels).
            height: Depth image height (pixels).
            fov: Horizontal field-of-view (radians).
            mount_z: Camera height above the floor (metres); defaults to
                :data:`config.CAMERA_MOUNT_Z`.
            depth_min: Nearest trustworthy reading (metres).
            depth_max: Farthest trustworthy reading (metres).
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
        self._u: npt.NDArray[np.float32] = np.arange(self.w, dtype=np.float32).reshape(1, -1)
        self._v: npt.NDArray[np.float32] = np.arange(self.h, dtype=np.float32).reshape(-1, 1)

    def to_robot_frame(
        self, depth: npt.NDArray[np.floating],
    ) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32], npt.NDArray[np.float32]]:
        """Back-project the depth image into the robot body frame.

        Args:
            depth: ``(H, W)`` depth image (metres along the optical axis).

        Returns:
            ``(x_r, y_r, z_r)`` — three ``(H, W)`` arrays giving the 3-D
            coordinates of each pixel in the robot body frame (x forward,
            y left, z up).
        """
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        with np.errstate(invalid="ignore"):
            x_r = depth
            y_r = -(self._u - self.cx) * depth / self.fx
            z_r = -(self._v - self.cy) * depth / self.fy + self.mount_z
        return x_r, y_r, z_r

    def thin_scan(
        self, depth: npt.NDArray[np.floating],
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64],
               npt.NDArray[np.bool_], npt.NDArray[np.bool_]]:
        """Per-column nearest in-band obstacle, AND clear-sight-line info.

        For each image column, determines whether an in-band obstacle exists
        and whether the sight-line is clear.

        Args:
            depth: ``(H, W)`` depth image (metres along the optical axis).

        Returns:
            A 4-tuple ``(bearings, hit_ranges, hit_mask, clear_mask)`` — all of
            length *W* (one entry per image column):

            * ``bearings`` — body-frame bearing angle per column (radians).
            * ``hit_ranges`` — range to the nearest in-band obstacle (metres);
              meaningful only where ``hit_mask`` is ``True``.
            * ``hit_mask`` — ``True`` where an in-band obstacle was found.
            * ``clear_mask`` — ``True`` where the sight-line is obstacle-free
              out to ``AUX_MAX_RANGE``.

            A column can be *neither* hit nor clear (e.g. all readings
            invalid / NaN) — callers should skip those columns.
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
        # still proves the near zone is obstacle-free, so it counts as clear —
        # it must NOT require that reading to itself be within range.
        any_valid = finite.any(axis=0)
        clear_mask = any_valid & ~hit_mask

        u = np.arange(self.w, dtype=np.float64)
        bearings = np.arctan2(-(u - self.cx) / self.fx, 1.0)  # body-frame bearing per column
        return bearings, hit_ranges, hit_mask, clear_mask
