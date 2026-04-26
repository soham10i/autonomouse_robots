"""HSV pillar detection on the RGB camera, with depth lookup at the centroid.

Uses OpenCV when available; falls back to a pure-NumPy HSV conversion and
bounding-box mask so the project runs on a vanilla Python install.
Returned fields: u, v (pixel centroid), area (m00 of the 0/255 mask, i.e.
255 × pixel count — matches cv2.moments convention), bearing (rad, +left),
range (m, NaN if depth invalid).
"""
import math
import numpy as np

import config as C


def _bgr_to_hsv_numpy(bgr):
    """BGR uint8 (H, W, 3) -> HSV uint8 in OpenCV convention (H ∈ [0,179])."""
    bgr_f = bgr.astype(np.float32) / 255.0
    b, g, r = bgr_f[..., 0], bgr_f[..., 1], bgr_f[..., 2]
    cmax = np.maximum(np.maximum(r, g), b)
    cmin = np.minimum(np.minimum(r, g), b)
    delta = cmax - cmin
    h = np.zeros_like(cmax)
    denom = np.where(delta > 1e-6, delta, 1.0)
    mask_r = (cmax == r) & (delta > 1e-6)
    mask_g = (cmax == g) & (delta > 1e-6)
    mask_b = (cmax == b) & (delta > 1e-6)
    h = np.where(mask_r, 60.0 * ((g - b) / denom % 6.0), h)
    h = np.where(mask_g, 60.0 * ((b - r) / denom + 2.0), h)
    h = np.where(mask_b, 60.0 * ((r - g) / denom + 4.0), h)
    h = (h / 2.0) % 180.0
    s = np.where(cmax > 1e-6, delta / np.where(cmax > 1e-6, cmax, 1.0), 0.0) * 255.0
    v = cmax * 255.0
    return np.stack([h, s, v], axis=-1).astype(np.uint8)


def _inrange_numpy(hsv, lo, hi):
    lo = np.asarray(lo, dtype=np.uint8)
    hi = np.asarray(hi, dtype=np.uint8)
    m = np.all((hsv >= lo) & (hsv <= hi), axis=-1)
    return m.astype(np.uint8) * 255


def _moments_numpy(mask):
    # 0/255 mask → m00 matches cv2.moments convention (255 × pixel count).
    m00 = float(mask.sum())
    if m00 < 1.0:
        return 0.0, 0.0, 0.0
    ys, xs = np.nonzero(mask)
    cu = float(xs.mean())
    cv = float(ys.mean())
    return m00, cu, cv


class PillarDetector:
    def __init__(self, camera_rgb, camera_depth=None):
        self.rgb = camera_rgb
        self.depth = camera_depth
        self.w = camera_rgb.getWidth()
        self.h = camera_rgb.getHeight()
        self.fov = camera_rgb.getFov()
        self.cx = 0.5 * self.w
        self.cy = 0.5 * self.h
        self.fx = 0.5 * self.w / math.tan(0.5 * self.fov)
        try:
            import cv2
            self._cv2 = cv2
            print("[perception] using opencv")
        except Exception:
            self._cv2 = None
            print("[perception] opencv unavailable, using numpy fallback")

    def _rgb_array(self):
        raw = self.rgb.getImage()
        if raw is None:
            return None
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(self.h, self.w, 4)
        return arr[:, :, :3]

    def _depth_at(self, u, v):
        if self.depth is None:
            return float("nan")
        try:
            d = self.depth.getRangeImage()
            if d is None:
                return float("nan")
            arr = np.asarray(d, dtype=np.float32).reshape(self.h, self.w)
            ui = int(np.clip(u, 0, self.w - 1))
            vi = int(np.clip(v, 0, self.h - 1))
            # 3×3 window average — single-pixel samples occasionally land
            # on a transparent pillar edge and read NaN/inf.
            u0, u1 = max(0, ui - 1), min(self.w, ui + 2)
            v0, v1 = max(0, vi - 1), min(self.h, vi + 2)
            patch = arr[v0:v1, u0:u1]
            good = patch[np.isfinite(patch) & (patch > 0.0)]
            if good.size == 0:
                return float("nan")
            return float(np.median(good))
        except Exception:
            return float("nan")

    def _detect_one(self, hsv, lo, hi, min_pixels=60):
        if self._cv2 is not None:
            mask = self._cv2.inRange(
                hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8)
            )
            m = self._cv2.moments(mask)
            if m["m00"] < min_pixels * 255:
                return None
            cu = m["m10"] / m["m00"]
            cv = m["m01"] / m["m00"]
            area = m["m00"]
        else:
            mask = _inrange_numpy(hsv, lo, hi)
            area, cu, cv = _moments_numpy(mask)
            if area < min_pixels * 255:
                return None
        bearing = -math.atan2(cu - self.cx, self.fx)
        rng = self._depth_at(cu, cv)
        return {"u": cu, "v": cv, "area": area, "bearing": bearing, "range": rng}

    def detect(self):
        out = {"blue": None, "yellow": None, "green": None}
        bgr = self._rgb_array()
        if bgr is None:
            return out
        hsv = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2HSV) if self._cv2 else _bgr_to_hsv_numpy(bgr)
        out["blue"] = self._detect_one(hsv, *C.HSV_BLUE)
        out["yellow"] = self._detect_one(hsv, *C.HSV_YELLOW)
        out["green"] = self._detect_one(hsv, *C.HSV_GREEN, min_pixels=200)
        return out
