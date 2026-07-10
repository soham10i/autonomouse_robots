"""Colour perception — pillars (blue/yellow), green-poison projection, reflex.

Pure NumPy HSV (no OpenCV needed).  The camera is mounted ~horizontally at
``CAMERA_MOUNT_Z``; the flat-floor pinhole model projects green floor pixels to
world points and estimates a pillar's range from either depth or its known
physical height.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


def bgr_to_hsv(bgr: np.ndarray) -> np.ndarray:
    """Converts a BGR image to HSV color space.

    Args:
        bgr (np.ndarray): BGR image of shape (H, W, 3) and type uint8.

    Returns:
        np.ndarray: HSV image of shape (H, W, 3) and type uint8, using the OpenCV convention (H in [0, 179]).
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


def in_range(hsv: np.ndarray, lo: Tuple[int, int, int], hi: Tuple[int, int, int]) -> np.ndarray:
    """Creates a boolean mask for pixels within a specific HSV range.

    Args:
        hsv (np.ndarray): The HSV image.
        lo (Tuple[int, int, int]): The lower bound HSV values.
        hi (Tuple[int, int, int]): The upper bound HSV values.

    Returns:
        np.ndarray: A boolean mask array of shape (H, W).
    """
    lo_arr = np.asarray(lo, dtype=np.uint8)
    hi_arr = np.asarray(hi, dtype=np.uint8)
    return np.all((hsv >= lo_arr) & (hsv <= hi_arr), axis=-1)


class Perception:
    """Camera-based HSV pillar detection and green poison-floor projection into world coordinates.
    
    Attributes:
        w (int): Camera image width.
        h (int): Camera image height.
        fov (float): Camera field of view.
        fx (float): Focal length in the x-axis.
        cx (float): Principal point x-coordinate.
        cy (float): Principal point y-coordinate.
        mount_z (float): Camera mount height in meters.
        pillar_world (Dict[str, Optional[Tuple[float, float]]]): Confirmed world position of detected pillars.
    """
    def __init__(self, width: int, height: int, fov: float, mount_z: Optional[float] = None) -> None:
        """Initializes the perception system with camera intrinsics.

        Args:
            width (int): Camera image width.
            height (int): Camera image height.
            fov (float): Camera horizontal field of view.
            mount_z (Optional[float], optional): Camera mount height. Defaults to `config.CAMERA_MOUNT_Z`.
        """
        self.w: int = int(width)
        self.h: int = int(height)
        self.fov: float = float(fov)
        self.fx: float = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.cx: float = 0.5 * self.w
        self.cy: float = 0.5 * self.h
        self.mount_z: float = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self._buf: Dict[str, Deque[Tuple[float, float]]] = {
            "blue": deque(maxlen=C.PILLAR_OBS_AVG_N),
            "yellow": deque(maxlen=C.PILLAR_OBS_AVG_N)
        }
        self.pillar_world: Dict[str, Optional[Tuple[float, float]]] = {"blue": None, "yellow": None}

    def _depth_at(self, depth: Optional[np.ndarray], u: float, v: float) -> float:
        """Estimates the depth at a given pixel coordinate using a small neighborhood patch."""
        if depth is None:
            return float("nan")
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        ui, vi = int(np.clip(u, 0, self.w - 1)), int(np.clip(v, 0, self.h - 1))
        patch = depth[max(0, vi - 1):vi + 2, max(0, ui - 1):ui + 2]
        good = patch[np.isfinite(patch) & (patch > 0.0)]
        return float(np.median(good)) if good.size else float("nan")

    def _detect_one(self, hsv: np.ndarray, depth: Optional[np.ndarray], lo: Tuple[int, int, int], hi: Tuple[int, int, int]) -> Optional[Dict[str, Any]]:
        """Detects a single colored blob and computes its properties."""
        mask = in_range(hsv, lo, hi)
        area = int(mask.sum())
        if area < C.PILLAR_MIN_PIXELS:
            return None
        ys, xs = np.nonzero(mask)
        cu, cv = float(xs.mean()), float(ys.mean())
        pix_h = int(ys.max() - ys.min() + 1)
        pix_w = int(xs.max() - xs.min() + 1)
        aspect = (pix_w / pix_h) if pix_h > 0 else float("nan")
        rng_d = self._depth_at(depth, cu, cv)
        rng_h = (self.fx * C.PILLAR_HEIGHT / pix_h) if pix_h > 0 else float("nan")
        if np.isfinite(rng_d) and 0.0 < rng_d <= C.PILLAR_MAX_DETECT_RANGE:
            rng = rng_d
        else:
            rng = rng_h
        est_h = (pix_h * rng / self.fx) if (np.isfinite(rng) and rng > 0) else float("nan")
        bearing = -math.atan2(cu - self.cx, self.fx)
        return {"u": cu, "v": cv, "area": area, "bearing": bearing,
                "range": rng, "est_height": est_h, "aspect": aspect}

    def _valid(self, det: Optional[Dict[str, Any]]) -> bool:
        """Validates a pillar detection against configured constraints."""
        if det is None:
            return False
        rng = det["range"]
        if not (np.isfinite(rng) and 0.0 < rng <= C.PILLAR_MAX_DETECT_RANGE):
            return False
        est_h = det["est_height"]
        if np.isfinite(est_h):
            lo = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MIN_FRAC
            hi = C.PILLAR_HEIGHT * C.PILLAR_HEIGHT_MAX_FRAC
            if not (lo <= est_h <= hi):
                return False
        if np.isfinite(det["aspect"]) and det["aspect"] > C.PILLAR_ASPECT_MAX:
            return False
        return True

    def update_pillars(self, bgr: np.ndarray, depth: Optional[np.ndarray], pose: Tuple[float, float, float]) -> Dict[str, Optional[Dict[str, Any]]]:
        """Detects both pillars and updates their running-mean world positions.

        Args:
            bgr (np.ndarray): The BGR camera image.
            depth (Optional[np.ndarray]): The depth range image.
            pose (Tuple[float, float, float]): The current world pose of the robot.

        Returns:
            Dict[str, Optional[Dict[str, Any]]]: A dictionary containing the raw detections for 'blue' and 'yellow' pillars.
        """
        hsv = bgr_to_hsv(bgr)
        out: Dict[str, Optional[Dict[str, Any]]] = {}
        for name, (lo, hi) in (("blue", C.HSV_BLUE), ("yellow", C.HSV_YELLOW)):
            det = self._detect_one(hsv, depth, lo, hi)
            out[name] = det
            if self._valid(det):
                self._ingest(name, det, pose)
        return out

    def _ingest(self, name: str, det: Dict[str, Any], pose: Tuple[float, float, float]) -> None:
        """Ingests a valid detection into the running mean world position for the pillar."""
        x, y, th = pose
        ang = wrap_angle(th + det["bearing"])
        rng = det["range"]
        wx = x + rng * math.cos(ang)
        wy = y + rng * math.sin(ang)
        buf = self._buf[name]
        if buf:
            mean = np.mean(np.array(buf), axis=0)
            if math.hypot(wx - mean[0], wy - mean[1]) > C.PILLAR_OUTLIER_REJECT_M:
                buf.clear()
        buf.append((wx, wy))
        m = np.mean(np.array(buf), axis=0)
        self.pillar_world[name] = (float(m[0]), float(m[1]))

    def green_floor_body(self, bgr: np.ndarray, max_range: Optional[float] = None) -> np.ndarray:
        """Projects green floor pixels to body-frame coordinates using a flat-floor model.

        Args:
            bgr (np.ndarray): The BGR camera image.
            max_range (Optional[float], optional): Maximum range to project out to. Defaults to `config.GREEN_PROJECT_MAX_RANGE`.

        Returns:
            np.ndarray: Array of shape (M, 2) containing the (x, y) coordinates of green floor points in the body frame.
        """
        if max_range is None:
            max_range = C.GREEN_PROJECT_MAX_RANGE
        hsv = bgr_to_hsv(bgr)
        mask = in_range(hsv, *C.HSV_GREEN)
        st = C.GREEN_PROJECT_STRIDE
        sub = mask[::st, ::st]
        if not sub.any():
            return np.empty((0, 2))
        vs_i, us_i = np.nonzero(sub)
        us = (us_i * st).astype(np.float32)
        vs = (vs_i * st).astype(np.float32)
        below = vs > self.cy + 2
        if not below.any():
            return np.empty((0, 2))
        us, vs = us[below], vs[below]
        d = (self.mount_z * self.fx) / (vs - self.cy)
        ok = (d > 0.18) & (d < max_range)
        if not ok.any():
            return np.empty((0, 2))
        us, vs, d = us[ok], vs[ok], d[ok]
        x_r = d
        y_r = -(us - self.cx) * d / self.fx
        return np.stack([x_r, y_r], axis=1)

    def green_floor_world(self, bgr: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
        """Projects green floor pixels to world coordinates for mapping.

        Args:
            bgr (np.ndarray): The BGR camera image.
            pose (Tuple[float, float, float]): The current world pose of the robot.

        Returns:
            np.ndarray: Array of shape (M, 2) containing the (x, y) coordinates of green floor points in the world frame.
        """
        body = self.green_floor_body(bgr)
        if body.shape[0] == 0:
            return np.empty((0, 2))
        return transform_points(body, pose[0], pose[1], pose[2])

    def green_reflex(self, bgr: np.ndarray) -> bool:
        """Checks if poison is imminent directly ahead to trigger a reflex halt.

        Args:
            bgr (np.ndarray): The BGR camera image.

        Returns:
            bool: True if imminent poison is detected within `GREEN_REFLEX_DIST`, False otherwise.
        """
        if not C.GREEN_REFLEX_ENABLED:
            return False
        body = self.green_floor_body(bgr, max_range=C.GREEN_REFLEX_DIST + 0.1)
        if body.shape[0] == 0:
            return False
        ahead = ((body[:, 0] > 0.05) & (body[:, 0] < C.GREEN_REFLEX_DIST)
                 & (np.abs(body[:, 1]) < C.GREEN_REFLEX_HALF_W))
        return int(ahead.sum()) >= C.GREEN_REFLEX_MIN_PTS
