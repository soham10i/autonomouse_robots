"""Colour perception — pillars (blue/yellow), green-poison projection, reflex.

Pure NumPy HSV (no OpenCV needed).  The camera is mounted ~horizontally at
``CAMERA_MOUNT_Z``; the flat-floor pinhole model projects green floor pixels to
world points and estimates a pillar's range from either depth or its known
physical height.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt

import config as C
from geometry import transform_points, wrap_angle


def bgr_to_hsv(bgr: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
    """Convert an RGB/BGR image to HSV (OpenCV convention).

    Reimplements ``cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)`` in pure NumPy.
    The Hue channel is scaled to ``[0, 179]`` (unlike standard ``[0, 359]``)
    so it fits within ``uint8``, matching OpenCV bounds.

    Args:
        bgr: ``(H, W, 3)`` image array of ``uint8`` pixels.

    Returns:
        ``(H, W, 3)`` HSV image array of ``uint8`` pixels.
    """
    f = bgr.astype(np.float32) / 255.0
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    cmax = np.maximum(np.maximum(r, g), b)
    cmin = np.minimum(np.minimum(r, g), b)
    delta = cmax - cmin
    denom = np.where(delta > 1e-6, delta, 1.0)
    h = np.zeros_like(cmax)
    mr = (cmax == r) & (delta > 1e-6)
    mg = (cmax == g) & (delta > 1e-6)
    mb = (cmax == b) & (delta > 1e-6)
    h = np.where(mr, 60.0 * ((g - b) / denom % 6.0), h)
    h = np.where(mg, 60.0 * ((b - r) / denom + 2.0), h)
    h = np.where(mb, 60.0 * ((r - g) / denom + 4.0), h)
    h = (h / 2.0) % 180.0
    s = np.where(cmax > 1e-6, delta / np.where(cmax > 1e-6, cmax, 1.0), 0.0) * 255.0
    v = cmax * 255.0
    return np.stack([h, s, v], axis=-1).astype(np.uint8)


def in_range(hsv: npt.NDArray[np.uint8],
             lo: Tuple[int, int, int],
             hi: Tuple[int, int, int]) -> npt.NDArray[np.bool_]:
    """Threshold an HSV image by upper and lower bounds.

    Args:
        hsv: ``(H, W, 3)`` HSV image array.
        lo: Lower bound ``(H, S, V)``.
        hi: Upper bound ``(H, S, V)``.

    Returns:
        ``(H, W)`` boolean mask where ``True`` indicates a pixel falls within
        the specified bounds.
    """
    lo_arr = np.asarray(lo, dtype=np.uint8)
    hi_arr = np.asarray(hi, dtype=np.uint8)
    return np.all((hsv >= lo_arr) & (hsv <= hi_arr), axis=-1)


class Perception:
    """Camera-based HSV pillar detection and green poison-floor projection.

    Uses the RGB and depth cameras to locate the coloured pillars and map
    the green poison decal onto the floor.  The internal state maintains a
    running average of pillar locations to smooth out single-frame noise.

    Attributes:
        w: Image width in pixels.
        h: Image height in pixels.
        fov: Horizontal field-of-view (radians).
        fx: Horizontal focal length (pixels).
        fy: Vertical focal length (pixels, assumed equal to *fx*).
        cx: Horizontal principal point (pixels).
        cy: Vertical principal point (pixels).
        mount_z: Camera optical-centre height above the floor (metres).
        pillar_world: Dictionary of confirmed world coordinates for each pillar,
            e.g. ``{"blue": (x, y)}``.
    """

    def __init__(self, width: int, height: int, fov: float,
                 mount_z: Optional[float] = None) -> None:
        """Initialise the pinhole camera model.

        Args:
            width: Image width (pixels).
            height: Image height (pixels).
            fov: Horizontal field-of-view (radians).
            mount_z: Camera height above the floor (metres); defaults to
                :data:`config.CAMERA_MOUNT_Z`.
        """
        self.w: int = int(width)
        self.h: int = int(height)
        self.fov: float = float(fov)
        self.fx: float = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.fy: float = self.fx           # square pixels (needed for the depth-validated
                                    # green projection's vertical back-projection)
        self.cx: float = 0.5 * self.w
        self.cy: float = 0.5 * self.h
        self.mount_z: float = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self._buf: Dict[str, deque[Tuple[float, float]]] = {
            "blue": deque(maxlen=C.PILLAR_OBS_AVG_N),
            "yellow": deque(maxlen=C.PILLAR_OBS_AVG_N)
        }
        # confirmed/averaged world position of each pillar once seen
        self.pillar_world: Dict[str, Tuple[float, float]] = {}

    def update_pillars(self, rgb: npt.NDArray[np.uint8],
                       depth: npt.NDArray[np.floating],
                       pose: Tuple[float, float, float]) -> None:
        """Scan the RGB frame for blue and yellow pillars and update their world positions.

        For each target colour, extracts the largest contiguous blob (above a
        minimum area). It computes the pillar's range by attempting a depth
        lookup near the centroid; if the depth is invalid (NaN/inf) it falls
        back to an analytical distance estimate derived from the blob's vertical
        extent (bounding box height) given the known physical pillar height.

        The resulting world coordinate is pushed into a rolling window buffer
        ``_buf``; once enough consistent frames are collected, the average is
        promoted to ``pillar_world``, ensuring spurious reflections are ignored.

        Args:
            rgb: ``(H, W, 3)`` BGR/RGB array of ``uint8`` pixels.
            depth: ``(H, W)`` depth image array (metres).
            pose: Current robot odometry pose ``(x, y, theta)``.
        """
        hsv = bgr_to_hsv(rgb)
        for name, (lo, hi) in C.PILLAR_HSV.items():
            mask = in_range(hsv, lo, hi)
            # Find vertical extent for distance fallback
            cols = mask.any(axis=0)
            rows = mask.any(axis=1)
            if not cols.any() or not rows.any():
                continue
            u_min, u_max = np.where(cols)[0][[0, -1]]
            v_min, v_max = np.where(rows)[0][[0, -1]]
            px_height = v_max - v_min
            px_width = u_max - u_min
            if px_width < 2 or px_height < 5:
                continue

            # Centroid
            ys, xs = np.nonzero(mask)
            cx_px = int(np.median(xs))
            cy_px = int(np.median(ys))
            bearing = math.atan2(-(cx_px - self.cx), self.fx)

            # Try depth read at centroid (with small search window)
            r = 2
            patch = depth[max(0, cy_px - r):min(self.h, cy_px + r + 1),
                          max(0, cx_px - r):min(self.w, cx_px + r + 1)]
            valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 10.0)]

            if len(valid) > 0:
                dist = float(np.median(valid))
            else:
                # Fallback: physical height projection
                # The pillar spans `px_height` pixels vertically.
                # angular height ~ px_height / fy
                # dist ~ physical_height / angular_height
                ang_h = px_height / self.fy
                dist = C.PILLAR_HEIGHT / ang_h if ang_h > 1e-3 else 5.0
                dist = max(0.1, min(6.0, dist))

            # body -> world
            bx, by = dist * math.cos(bearing), dist * math.sin(bearing)
            wx, wy = transform_points(np.array([[bx, by]]), pose[0], pose[1], pose[2])[0]

            self._buf[name].append((float(wx), float(wy)))
            # Promote to confirmed if we have enough observations
            if len(self._buf[name]) >= C.PILLAR_OBS_AVG_N:
                pts = np.array(self._buf[name])
                self.pillar_world[name] = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))

    def green_floor_world(self, rgb: npt.NDArray[np.uint8],
                          depth: npt.NDArray[np.floating],
                          pose: Tuple[float, float, float]) -> npt.NDArray[np.float64]:
        """Detect the green poison floor decal and project it to world coordinates.

        Filters the image by HSV bounds, then filters out elevated green pixels
        (e.g., walls reflecting green light) by ensuring their depth back-projects
        to ``z < 0.05`` (flat floor). The surviving pixels are mapped to world
        ``(x, y)`` coordinates.

        Args:
            rgb: ``(H, W, 3)`` BGR/RGB array of ``uint8`` pixels.
            depth: ``(H, W)`` depth image array (metres).
            pose: Current robot odometry pose ``(x, y, theta)``.

        Returns:
            ``(N, 2)`` array of world-frame points representing detected poison.
            If no poison is seen, returns an empty array ``(0, 2)``.
        """
        hsv = bgr_to_hsv(rgb)
        lo, hi = C.HSV_GREEN
        mask = in_range(hsv, lo, hi)
        if not mask.any():
            return np.empty((0, 2))

        # flat-floor + depth validation
        vs, us = np.nonzero(mask)
        d = depth[vs, us]
        valid = np.isfinite(d) & (d > 0.05) & (d < 10.0)
        us, vs, d = us[valid], vs[valid], d[valid]
        if len(us) == 0:
            return np.empty((0, 2))

        # 1) calculate z_body for these pixels to reject "flying green"
        # (walls reflecting the floor decal)
        u_c = us - self.cx
        v_c = vs - self.cy
        z_b = -v_c * d / self.fy + self.mount_z

        floor_mask = z_b < 0.05
        us = us[floor_mask]
        d = d[floor_mask]
        if len(us) == 0:
            return np.empty((0, 2))

        # 2) convert surviving floor pixels to x/y body coordinates
        u_c = us - self.cx
        xb = d
        yb = -u_c * d / self.fx
        body_pts = np.stack([xb, yb], axis=1)

        # 3) transform to world frame
        world_pts = transform_points(body_pts, pose[0], pose[1], pose[2])

        # sub-sample if too dense (performance)
        if world_pts.shape[0] > C.POISON_MAX_PTS:
            idx = np.linspace(0, world_pts.shape[0] - 1, C.POISON_MAX_PTS).astype(int)
            world_pts = world_pts[idx]

        return world_pts
