"""HSV colour detection for the blue/yellow pillars and green poison floor.

Pure NumPy (no OpenCV required — the sandbox and many lab machines lack cv2).
All methods take image arrays and return geometry, so the detector is testable
with synthetic images.  Intrinsics are supplied once at construction.

Returned pillar fields: ``area`` (pixel count), ``bearing`` (rad, +left),
``range`` (m, NaN if depth invalid), ``est_height`` (m), ``aspect`` (w/h).
"""
from __future__ import annotations

import math

import numpy as np

import settings as S
from geometry import transform_points


def bgr_to_hsv(bgr):
    """BGR uint8 (H, W, 3) -> HSV uint8, OpenCV convention (H in [0,179])."""
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


def in_range(hsv, lo, hi):
    lo = np.asarray(lo, dtype=np.uint8)
    hi = np.asarray(hi, dtype=np.uint8)
    return np.all((hsv >= lo) & (hsv <= hi), axis=-1)


class ColorDetector:
    def __init__(self, width, height, fov, mount_z=None):
        self.w = int(width)
        self.h = int(height)
        self.fov = float(fov)
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.mount_z = S.CAMERA_MOUNT_Z if mount_z is None else mount_z

    # ----------------------------------------------------------- helpers
    def _depth_at(self, depth, u, v):
        if depth is None:
            return float("nan")
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        ui = int(np.clip(u, 0, self.w - 1))
        vi = int(np.clip(v, 0, self.h - 1))
        u0, u1 = max(0, ui - 1), min(self.w, ui + 2)
        v0, v1 = max(0, vi - 1), min(self.h, vi + 2)
        patch = depth[v0:v1, u0:u1]
        good = patch[np.isfinite(patch) & (patch > 0.0)]
        if good.size == 0:
            return float("nan")
        return float(np.median(good))

    def _detect_one(self, hsv, depth, lo, hi, min_pixels):
        mask = in_range(hsv, lo, hi)
        area = int(mask.sum())
        if area < min_pixels:
            return None
        ys, xs = np.nonzero(mask)
        cu = float(xs.mean())
        cv = float(ys.mean())
        v_top, v_bot = int(ys.min()), int(ys.max())
        u_left, u_right = int(xs.min()), int(xs.max())
        pix_h = v_bot - v_top + 1
        pix_w = u_right - u_left + 1
        aspect = (pix_w / pix_h) if pix_h > 0 else float("nan")
        rng = self._depth_at(depth, cu, cv)
        est_h = (pix_h * rng / self.fx) if (np.isfinite(rng) and rng > 0) else float("nan")
        bearing = -math.atan2(cu - self.cx, self.fx)
        return {"u": cu, "v": cv, "area": area, "bearing": bearing,
                "range": rng, "est_height": est_h, "aspect": aspect}

    # ------------------------------------------------------------ public
    def detect_pillars(self, bgr, depth):
        """Return ``{'blue': det|None, 'yellow': det|None}``."""
        hsv = bgr_to_hsv(bgr)
        return {
            "blue": self._detect_one(hsv, depth, *S.HSV_BLUE, S.PILLAR_MIN_PIXELS),
            "yellow": self._detect_one(hsv, depth, *S.HSV_YELLOW, S.PILLAR_MIN_PIXELS),
        }

    def is_valid_pillar(self, det):
        """Height / aspect / range gate shared by both pillars."""
        if det is None:
            return False
        rng = det["range"]
        if not (np.isfinite(rng) and 0.0 < rng <= S.PILLAR_MAX_DETECT_RANGE):
            return False
        est_h = det["est_height"]
        if np.isfinite(est_h):
            lo = S.PILLAR_HEIGHT * S.PILLAR_HEIGHT_MIN_FRAC
            hi = S.PILLAR_HEIGHT * S.PILLAR_HEIGHT_MAX_FRAC
            if not (lo <= est_h <= hi):
                return False
        aspect = det["aspect"]
        if np.isfinite(aspect) and aspect > S.PILLAR_ASPECT_MAX:
            return False
        return True

    def green_floor_points_world(self, bgr, pose, stride=3,
                                 min_range=0.20, max_range=4.5):
        """Project green floor pixels onto the ground plane -> world (M, 2).

        Flat-floor pinhole model: a pixel ``v`` rows below the principal point
        sees the ground at ``d = mount_z * fx / (v - cy)``.  Per-pixel
        projection matches the patch outline instead of a fat centroid disc.
        """
        hsv = bgr_to_hsv(bgr)
        mask = in_range(hsv, *S.HSV_GREEN)
        sub = mask[::stride, ::stride]
        if not sub.any():
            return np.empty((0, 2))
        vs_i, us_i = np.nonzero(sub)
        us = (us_i * stride).astype(np.float32)
        vs = (vs_i * stride).astype(np.float32)
        below = vs > self.cy + 2
        if not below.any():
            return np.empty((0, 2))
        us, vs = us[below], vs[below]
        d = (self.mount_z * self.fx) / (vs - self.cy)
        ok = (d > min_range) & (d < max_range)
        if not ok.any():
            return np.empty((0, 2))
        us, vs, d = us[ok], vs[ok], d[ok]
        x_r = d
        y_r = -(us - self.cx) * d / self.fx
        body = np.stack([x_r, y_r], axis=1)
        return transform_points(body, pose[0], pose[1], pose[2])
