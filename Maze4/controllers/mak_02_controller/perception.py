"""Colour perception — pillars (blue/yellow), green-poison projection, reflex.

Pure NumPy HSV (no OpenCV needed).  The camera is mounted ~horizontally at
``CAMERA_MOUNT_Z``; the flat-floor pinhole model projects green floor pixels to
world points and estimates a pillar's range from either depth or its known
physical height.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


def bgr_to_hsv(bgr):
    """BGR uint8 (H,W,3) -> HSV uint8, OpenCV convention (H in [0,179])."""
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


class Perception:
    """Camera-based HSV pillar detection and green poison-floor projection into world coordinates."""
    def __init__(self, width, height, fov, mount_z=None):
        self.w = int(width)
        self.h = int(height)
        self.fov = float(fov)
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.mount_z = C.CAMERA_MOUNT_Z if mount_z is None else mount_z
        self._buf = {"blue": deque(maxlen=C.PILLAR_OBS_AVG_N),
                     "yellow": deque(maxlen=C.PILLAR_OBS_AVG_N)}
        # confirmed/averaged world position of each pillar once seen
        self.pillar_world = {"blue": None, "yellow": None}

    # ------------------------------------------------------------- depth
    def _depth_at(self, depth, u, v):
        if depth is None:
            return float("nan")
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        ui, vi = int(np.clip(u, 0, self.w - 1)), int(np.clip(v, 0, self.h - 1))
        patch = depth[max(0, vi - 1):vi + 2, max(0, ui - 1):ui + 2]
        good = patch[np.isfinite(patch) & (patch > 0.0)]
        return float(np.median(good)) if good.size else float("nan")

    # ------------------------------------------------------ pillar detect
    def _detect_one(self, hsv, depth, lo, hi):
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
        # range from known pillar height (works at close range where depth blinds)
        rng_h = (self.fx * C.PILLAR_HEIGHT / pix_h) if pix_h > 0 else float("nan")
        if np.isfinite(rng_d) and 0.0 < rng_d <= C.PILLAR_MAX_DETECT_RANGE:
            rng = rng_d
        else:
            rng = rng_h
        est_h = (pix_h * rng / self.fx) if (np.isfinite(rng) and rng > 0) else float("nan")
        bearing = -math.atan2(cu - self.cx, self.fx)
        return {"u": cu, "v": cv, "area": area, "bearing": bearing,
                "range": rng, "est_height": est_h, "aspect": aspect}

    def _valid(self, det):
        if det is None:
            return False
        rng = det["range"]
        if not (np.isfinite(rng) and C.PILLAR_MIN_DETECT_RANGE <= rng <= C.PILLAR_MAX_DETECT_RANGE):
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

    def update_pillars(self, bgr, depth, pose):
        """Detect both pillars, update their running-mean world positions.

        Returns ``{'blue': det|None, 'yellow': det|None}`` (raw detections).
        """
        hsv = bgr_to_hsv(bgr)
        out = {}
        for name, (lo, hi) in (("blue", C.HSV_BLUE), ("yellow", C.HSV_YELLOW)):
            det = self._detect_one(hsv, depth, lo, hi)
            out[name] = det
            if self._valid(det):
                self._ingest(name, det, pose)
        return out

    def _ingest(self, name, det, pose):
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
    def green_floor_body(self, bgr, max_range=None):
        """Project green floor pixels to BODY-frame points (M,2), flat-floor model."""
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

    def green_floor_world(self, bgr, pose):
        """Project green floor pixels to world (M,2) for the sticky poison map."""
        body = self.green_floor_body(bgr)
        if body.shape[0] == 0:
            return np.empty((0, 2))
        return transform_points(body, pose[0], pose[1], pose[2])

    def green_reflex(self, bgr):
        """True ONLY if poison is imminent: projected green within
        ``GREEN_REFLEX_DIST`` m directly ahead.  Distant poison must NOT trip
        this — it is handled by the mapped poison layer + A*/DWA — otherwise the
        robot deadlocks facing far poison it could legally route around."""
        if not C.GREEN_REFLEX_ENABLED:
            return False
        body = self.green_floor_body(bgr, max_range=C.GREEN_REFLEX_DIST + 0.1)
        if body.shape[0] == 0:
            return False
        ahead = ((body[:, 0] > 0.05) & (body[:, 0] < C.GREEN_REFLEX_DIST)
                 & (np.abs(body[:, 1]) < C.GREEN_REFLEX_HALF_W))
        return int(ahead.sum()) >= C.GREEN_REFLEX_MIN_PTS
