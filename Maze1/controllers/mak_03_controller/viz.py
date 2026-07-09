"""Live map / frontier visualisation.

Renders the occupancy grid, sticky poison, detected frontiers, the global A*
path, the carrot, both pillars and the robot pose into an RGB image.  Saves PNG
snapshots and (optionally) shows a live OpenCV window.  Degrades gracefully if
OpenCV / PIL are unavailable.
"""
from __future__ import annotations

import math
import os

import numpy as np

import config as C

try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

_LIVE = os.environ.get("MAK02_LIVE", "1") != "0"

# colours are RGB
_C_UNKNOWN = (128, 128, 128)
_C_FREE = (245, 245, 245)
_C_OCC = (25, 25, 25)
_C_POISON = (0, 190, 0)
_C_FRONTIER = (0, 220, 220)
_C_PATH = (255, 140, 0)
_C_ROBOT = (220, 30, 30)
_C_BLUE = (40, 80, 255)
_C_YELLOW = (245, 220, 0)
_C_CARROT = (230, 0, 230)


class Visualizer:
    def __init__(self, grid, scale=3):
        self.grid = grid
        self.scale = scale
        self.window_ready = False

    # ----------------------------------------------------- world -> pixel
    def _to_px(self, wx, wy):
        g = self.grid
        ix = (wx - g.ox) / g.res
        iy = (wy - g.oy) / g.res
        col = int(ix * self.scale)
        row = int((g.n - 1 - iy) * self.scale)
        return col, row

    def render(self, pose, frontier_mask, path_world, carrot, pillars, state, t):
        g = self.grid
        n = g.n
        img = np.empty((n, n, 3), dtype=np.uint8)
        # base layers (note: array is [ix, iy]; flip iy so +y points up)
        occ = g.occupied_mask()
        free = g.free_mask()
        base = np.full((n, n, 3), _C_UNKNOWN, dtype=np.uint8)
        base[free] = _C_FREE
        base[occ] = _C_OCC
        base[g.poison] = _C_POISON
        if frontier_mask is not None:
            base[frontier_mask] = _C_FRONTIER
        # transpose ix,iy -> row=iy then flip vertically
        img = np.flipud(np.transpose(base, (1, 0, 2))).copy()

        if self.scale != 1:
            img = np.repeat(np.repeat(img, self.scale, axis=0), self.scale, axis=1)

        if _HAVE_CV2:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self._draw_cv(bgr, pose, path_world, carrot, pillars, state, t)
            img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return img

    def _draw_cv(self, bgr, pose, path_world, carrot, pillars, state, t):
        rgb2bgr = lambda c: (c[2], c[1], c[0])
        # path
        if path_world and len(path_world) >= 2:
            pts = [self._to_px(px, py) for px, py in path_world]
            for a, b in zip(pts[:-1], pts[1:]):
                cv2.line(bgr, a, b, rgb2bgr(_C_PATH), 2)
        # carrot
        if carrot is not None:
            cv2.circle(bgr, self._to_px(*carrot), 4, rgb2bgr(_C_CARROT), -1)
        # pillars
        for name, col in (("blue", _C_BLUE), ("yellow", _C_YELLOW)):
            p = pillars.get(name)
            if p is not None:
                cv2.circle(bgr, self._to_px(*p), 6, rgb2bgr(col), -1)
                cv2.circle(bgr, self._to_px(*p), 6, (0, 0, 0), 1)
        # robot
        rc = self._to_px(pose[0], pose[1])
        cv2.circle(bgr, rc, 5, rgb2bgr(_C_ROBOT), -1)
        hx = pose[0] + 0.28 * math.cos(pose[2])
        hy = pose[1] + 0.28 * math.sin(pose[2])
        cv2.line(bgr, rc, self._to_px(hx, hy), rgb2bgr(_C_ROBOT), 2)
        # HUD
        cv2.putText(bgr, f"{state}  t={t:5.1f}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(bgr, f"{state}  t={t:5.1f}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def show(self, img):
        if not (_LIVE and _HAVE_CV2):
            return
        try:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow("mak_03 — Maze1 navigation", bgr)
            cv2.waitKey(1)
            self.window_ready = True
        except Exception:
            pass

    def save(self, img, path):
        try:
            if _HAVE_CV2:
                cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                return True
        except Exception:
            pass
        try:
            from PIL import Image  # type: ignore
            Image.fromarray(img).save(path)
            return True
        except Exception:
            return False

    def close(self):
        if _LIVE and _HAVE_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
