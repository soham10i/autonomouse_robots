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
        self.fy = self.fx           # square pixels (needed for the depth-validated
                                    # green projection's vertical back-projection)
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
    def green_floor_body(self, bgr, depth, max_range=None):
        """Project green floor pixels to BODY-frame (M,2) using the DEPTH camera.

        DEPTH-VALIDATED -- no flat-floor assumption.  Each green pixel is back-
        projected with its MEASURED depth, then kept only if its reconstructed
        3-D height is at floor level (|z| < GREEN_FLOOR_Z_TOL).

        Why this replaces the old flat-floor pinhole ``d = mount_z*fx/(v-cy)``:
        that formula's range error diverges as a pixel nears the image horizon
        (v -> cy => d -> inf), so an oblique or distant green patch smeared along
        the whole view ray.  Combined with mak_04's now-stable pose, the same
        wrong cells were reinforced every frame and locked the poison layer to
        hundreds of phantom cells (pois -> 822, never decaying).  Using the real
        depth places green on its true footprint and rejects green-tinted pixels
        that are actually on walls/pillars (their reconstructed z is not floor).
        Pixels in the < depth_min blind shell (or NaN) are simply dropped this
        frame -- they get mapped once the robot is at a depth-resolvable range.
        """
        if max_range is None:
            max_range = C.GREEN_PROJECT_MAX_RANGE
        if depth is None:
            return np.empty((0, 2))
        hsv = bgr_to_hsv(bgr)
        mask = in_range(hsv, *C.HSV_GREEN)
        st = C.GREEN_PROJECT_STRIDE
        sub = mask[::st, ::st]
        if not sub.any():
            return np.empty((0, 2))
        depth = np.asarray(depth, dtype=np.float32).reshape(self.h, self.w)
        dsub = depth[::st, ::st]
        vs_i, us_i = np.nonzero(sub)
        us = (us_i * st).astype(np.float32)
        vs = (vs_i * st).astype(np.float32)
        d = dsub[vs_i, us_i].astype(np.float32)
        # Webots range image is Z-depth along the optical axis (same convention
        # as depth_model.to_robot_frame): x fwd = d, y left = -(u-cx)d/fx,
        # z up = -(v-cy)d/fy + mount_z.
        x_r = d
        y_r = -(us - self.cx) * d / self.fx
        z_r = -(vs - self.cy) * d / self.fy + self.mount_z
        ok = (np.isfinite(d) & (d > C.GREEN_DEPTH_MIN) & (d < max_range)
              & (np.abs(z_r) < C.GREEN_FLOOR_Z_TOL))
        if not ok.any():
            return np.empty((0, 2))
        return np.stack([x_r[ok], y_r[ok]], axis=1)

    def green_floor_world(self, bgr, depth, pose):
        """Project green floor pixels to world (M,2) for the poison map."""
        body = self.green_floor_body(bgr, depth)
        if body.shape[0] == 0:
            return np.empty((0, 2))
        return transform_points(body, pose[0], pose[1], pose[2])
