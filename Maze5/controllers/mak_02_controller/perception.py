"""Colour perception algorithms for pillar detection, poison projection, and reflex.

Provides pure NumPy implementations for HSV color space conversion and thresholding,
bypassing the need for OpenCV. Implements a pinhole camera model assuming a flat-floor
to project identified green poison patches into world coordinates and estimate distances
to blue/yellow mission pillars.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional, Any

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


def bgr_to_hsv(bgr: np.ndarray) -> np.ndarray:
    """Converts a BGR image array to HSV color space matching OpenCV's convention.

    Uses the standard algorithm to map RGB to HSV, scaling the hue to the range [0, 179]
    to match the output of ``cv2.cvtColor(img, cv2.COLOR_BGR2HSV)``.

    Args:
        bgr (np.ndarray): An ``(H, W, 3)`` uint8 array in BGR format.

    Returns:
        np.ndarray: An ``(H, W, 3)`` uint8 array representing the HSV image.
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


def in_range(hsv: np.ndarray, lo: tuple[int, int, int], hi: tuple[int, int, int]) -> np.ndarray:
    """Creates a boolean mask identifying pixels within a specified HSV range.

    Args:
        hsv (np.ndarray): The input HSV image array.
        lo (tuple[int, int, int]): The lower bound `(H, S, V)` threshold.
        hi (tuple[int, int, int]): The upper bound `(H, S, V)` threshold.

    Returns:
        np.ndarray: A 2D boolean mask where True indicates the pixel falls within the bounds.
    """
    lo_arr = np.asarray(lo, dtype=np.uint8)
    hi_arr = np.asarray(hi, dtype=np.uint8)
    return np.all((hsv >= lo_arr) & (hsv <= hi_arr), axis=-1)


class Perception:
    """Camera-based HSV pillar detection and green poison-floor projection into world coordinates.

    Attributes:
        w (int): Image width in pixels.
        h (int): Image height in pixels.
        fov (float): Horizontal field of view in radians.
        fx (float): Computed horizontal focal length in pixels.
        cx (float): Principal point x-coordinate (image center).
        cy (float): Principal point y-coordinate (image center).
        mount_z (float): Physical height of the camera above the floor in meters.
        pillar_world (dict[str, Optional[tuple[float, float]]]): Confirmed and averaged
            world `(x, y)` positions of identified pillars, keyed by color name.
    """

    def __init__(self, width: int, height: int, fov: float, mount_z: Optional[float] = None) -> None:
        """Initializes the perception model with camera intrinsics and extrinsics.

        Args:
            width (int): Pixel width of the camera image.
            height (int): Pixel height of the camera image.
            fov (float): Camera horizontal field of view in radians.
            mount_z (Optional[float], optional): Camera mounting height from the floor.
                Defaults to config `CAMERA_MOUNT_Z`.
        """
        self.w = int(width)
        self.h = int(height)
        self.fov = float(fov)
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.mount_z = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self._buf: dict[str, deque[tuple[float, float]]] = {
            "blue": deque(maxlen=C.PILLAR_OBS_AVG_N),
            "yellow": deque(maxlen=C.PILLAR_OBS_AVG_N)
        }
        # confirmed/averaged world position of each pillar once seen
        self.pillar_world: dict[str, Optional[tuple[float, float]]] = {"blue": None, "yellow": None}

    # ------------------------------------------------------------- depth
    def _depth_at(self, depth: Optional[np.ndarray], u: float, v: float) -> float:
        """Extracts a robust depth measurement at a specified pixel coordinate.

        Takes the median depth value from a 3x3 patch centered at `(u, v)` to
        filter out noise or NaN holes in the raw depth map.

        Args:
            depth (Optional[np.ndarray]): The raw depth image array. Returns NaN if None.
            u (float): The horizontal pixel coordinate (column).
            v (float): The vertical pixel coordinate (row).

        Returns:
            float: The median valid depth in meters, or NaN if unavailable.
        """
        if depth is None:
            return float("nan")
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        ui, vi = int(np.clip(u, 0, self.w - 1)), int(np.clip(v, 0, self.h - 1))
        patch = depth[max(0, vi - 1):vi + 2, max(0, ui - 1):ui + 2]
        good = patch[np.isfinite(patch) & (patch > 0.0)]
        return float(np.median(good)) if good.size else float("nan")

    # ------------------------------------------------------ pillar detect
    def _detect_one(self, hsv: np.ndarray, depth: Optional[np.ndarray], lo: tuple[int, int, int], hi: tuple[int, int, int]) -> Optional[dict[str, Any]]:
        """Detects a single color blob representing a pillar within the HSV frame.

        Calculates blob area, centroid, bounding box, aspect ratio, and estimates distance
        using the depth sensor (if available) or the pinhole model and known pillar height.
        Silently rejects detections that clip the image boundaries to prevent biased bearing
        or range estimations.

        Args:
            hsv (np.ndarray): The HSV image array.
            depth (Optional[np.ndarray]): The raw depth image array.
            lo (tuple[int, int, int]): The lower bound `(H, S, V)` color threshold.
            hi (tuple[int, int, int]): The upper bound `(H, S, V)` color threshold.

        Returns:
            Optional[dict[str, Any]]: A dictionary describing the detection (centroid, area,
                bearing, range, estimated height, aspect ratio, clipped flag), or None if
                the blob is too small.
        """
        mask = in_range(hsv, lo, hi)
        area = int(mask.sum())
        if area < C.PILLAR_MIN_PIXELS:
            return None
        ys, xs = np.nonzero(mask)
        cu, cv = float(xs.mean()), float(ys.mean())
        y0, y1 = int(ys.min()), int(ys.max())
        x0, x1 = int(xs.min()), int(xs.max())
        pix_h = int(y1 - y0 + 1)
        pix_w = int(x1 - x0 + 1)
        aspect = (pix_w / pix_h) if pix_h > 0 else float("nan")
        # If the blob runs off the top/bottom of the frame, pix_h no longer
        # spans the pillar's TRUE height, so the height-based range (which
        # assumes pix_h == full physical height) silently OVERESTIMATES range.
        # If it runs off the LEFT/RIGHT edge instead, the centroid `cu` -- and
        # therefore the BEARING derived from it -- is biased toward whichever
        # side is still visible, which misplaces the projected world position
        # sideways (this is the one that actually bit us: it fires at ANY
        # range whenever the pillar sweeps across frame edge while the robot
        # turns to face it, not just up close). Reject the WHOLE detection
        # when either happens rather than trying to patch just the range,
        # since bearing is used regardless of which range source is picked.
        clipped_v = (y0 <= 1) or (y1 >= self.h - 2)
        clipped_h = (x0 <= 1) or (x1 >= self.w - 2)
        clipped = clipped_v or clipped_h
        rng_d = self._depth_at(depth, cu, cv)
        rng_h = (self.fx * C.PILLAR_HEIGHT / pix_h) if (pix_h > 0 and not clipped_v) else float("nan")
        if np.isfinite(rng_d) and 0.0 < rng_d <= C.PILLAR_MAX_DETECT_RANGE:
            rng = rng_d
        else:
            rng = rng_h
        est_h = (pix_h * rng / self.fx) if (np.isfinite(rng) and rng > 0 and not clipped_v) else float("nan")
        bearing = -math.atan2(cu - self.cx, self.fx)
        return {"u": cu, "v": cv, "area": area, "bearing": bearing,
                "range": rng, "est_height": est_h, "aspect": aspect, "clipped": clipped}

    def _valid(self, det: Optional[dict[str, Any]]) -> bool:
        """Validates a raw pillar detection against shape and distance heuristics.

        Args:
            det (Optional[dict[str, Any]]): The detection dictionary returned by `_detect_one`.

        Returns:
            bool: True if the detection meets physical validity criteria, False otherwise.
        """
        if det is None:
            return False
        if det["clipped"]:
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

    def update_pillars(
        self,
        bgr: np.ndarray,
        depth: Optional[np.ndarray],
        pose: tuple[float, float, float]
    ) -> dict[str, Optional[dict[str, Any]]]:
        """Detects both blue and yellow pillars and updates their world position estimates.

        Args:
            bgr (np.ndarray): The current BGR camera frame.
            depth (Optional[np.ndarray]): The current depth image array (if available).
            pose (tuple[float, float, float]): The current `(x, y, theta)` robot pose in world coordinates.

        Returns:
            dict[str, Optional[dict[str, Any]]]: A dictionary containing raw valid detection data
                keyed by pillar name ('blue', 'yellow'). Note that valid detections also update
                `self.pillar_world` internally as a side effect.
        """
        hsv = bgr_to_hsv(bgr)
        out: dict[str, Optional[dict[str, Any]]] = {}
        for name, (lo, hi) in (("blue", C.HSV_BLUE), ("yellow", C.HSV_YELLOW)):
            det = self._detect_one(hsv, depth, lo, hi)
            out[name] = det
            if self._valid(det):
                self._ingest(name, det, pose)
        return out

    def _ingest(self, name: str, det: dict[str, Any], pose: tuple[float, float, float]) -> None:
        """Accumulates a valid detection and updates the running-mean pillar world coordinates.

        Detects large positional jumps (outliers) and completely flushes the rolling buffer
        if the new observation deviates severely from the historical mean.

        Args:
            name (str): The color name of the pillar ('blue' or 'yellow').
            det (dict[str, Any]): The validated detection dictionary.
            pose (tuple[float, float, float]): The robot's current world pose.
        """
        x, y, th = pose
        ang = wrap_angle(th + det["bearing"])
        # stand a little short of the centre so we measure the front face
        rng = det["range"]
        wx = x + rng * math.cos(ang)
        wy = y + rng * math.sin(ang)
        buf = self._buf[name]
        if buf:
            mean = np.mean(np.array(buf), axis=0)
            if math.hypot(wx - mean[0], wy - mean[1]) > C.PILLAR_OUTLIER_REJECT_M:
                # large jump: reset rather than average through an outlier
                buf.clear()
        buf.append((wx, wy))
        m = np.mean(np.array(buf), axis=0)
        self.pillar_world[name] = (float(m[0]), float(m[1]))

    # --------------------------------------------------- green projection
    def green_floor_body(self, bgr: np.ndarray, max_range: Optional[float] = None) -> np.ndarray:
        """Projects detected green floor pixels into body-frame Cartesian coordinates.

        Uses a flat-floor assumption to compute distance based on pixel row (v-coordinate).

        Args:
            bgr (np.ndarray): The BGR camera image array.
            max_range (Optional[float], optional): Maximum projection range cutoff. Defaults to config `GREEN_PROJECT_MAX_RANGE`.

        Returns:
            np.ndarray: An ``(M, 2)`` array containing the `(x, y)` body-frame coordinates of projected poison points.
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

    def green_floor_world(self, bgr: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
        """Projects detected green floor pixels into world coordinates for map stamping.

        Args:
            bgr (np.ndarray): The BGR camera image array.
            pose (tuple[float, float, float]): The robot's current world pose.

        Returns:
            np.ndarray: An ``(M, 2)`` array containing the `(wx, wy)` world coordinates of projected poison points.
        """
        body = self.green_floor_body(bgr)
        if body.shape[0] == 0:
            return np.empty((0, 2))
        return transform_points(body, pose[0], pose[1], pose[2])

    def green_reflex(self, bgr: np.ndarray) -> bool:
        """Determines if the robot should trigger an immediate emergency halt due to poison.

        Returns True ONLY if poison is imminent directly ahead within the `GREEN_REFLEX_DIST`.
        Distant poison patches must not trip this logic (they are handled via A*/DWA routing);
        otherwise, the robot may deadlock while facing far poison it could otherwise bypass.

        Args:
            bgr (np.ndarray): The BGR camera image array.

        Returns:
            bool: True if sufficient green floor points fall within the critical reflex zone.
        """
        if not C.GREEN_REFLEX_ENABLED:
            return False
        body = self.green_floor_body(bgr, max_range=C.GREEN_REFLEX_DIST + 0.1)
        if body.shape[0] == 0:
            return False
        ahead = ((body[:, 0] > 0.05) & (body[:, 0] < C.GREEN_REFLEX_DIST)
                 & (np.abs(body[:, 1]) < C.GREEN_REFLEX_HALF_W))
        return int(ahead.sum()) >= C.GREEN_REFLEX_MIN_PTS
