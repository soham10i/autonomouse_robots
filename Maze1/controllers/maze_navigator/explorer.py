"""Frontier extraction on the 2D occupancy grid.

This module no longer drives the robot — it just answers "where should I go
next to learn about the world?" The actual driving is done by ``planning.py``
+ ``local_control.py``.

Frontier = a free cell that has at least one *unknown* 4-neighbour, and is
not lethal in the inflated mask (so it's reachable). Frontiers are clustered;
the chosen cluster is scored as

    score = cluster_size / (1 + dist_to_robot)
            * heading_pref(angle_to_robot_heading)
            * (0 if blacklisted)
"""
import math
import numpy as np

import config as C


class Explorer:
    def __init__(self, grid):
        self.grid = grid

    def find_frontier(self, pose, blacklist=None,
                      heading_bias=0.6, allow_inflated=False):
        """Return ``(wx, wy)`` of the best frontier, or ``None`` if no
        frontier exists.

        ``heading_bias`` ∈ [0, 1]: 0 → ignore heading, 1 → maximally prefer
        frontiers in front of the robot.
        ``allow_inflated``: if False (default), reject frontier centroids
        that fall inside the inflated lethal mask.
        """
        blacklist = blacklist or []
        L = self.grid.L
        free = L < C.L_THRESH_FREE
        unknown = np.abs(L) < 0.05

        pad = np.zeros_like(unknown)
        pad[1:, :] |= unknown[:-1, :]
        pad[:-1, :] |= unknown[1:, :]
        pad[:, 1:] |= unknown[:, :-1]
        pad[:, :-1] |= unknown[:, 1:]
        front = free & pad
        if not front.any():
            return None

        rx, ry, rth = pose

        try:
            from scipy.ndimage import label
            lab, n = label(front)
        except Exception:
            lab, n = self._label_fallback(front)

        if n == 0:
            return None

        sizes = np.bincount(lab.ravel())
        sizes[0] = 0
        candidates = []
        for k in range(1, n + 1):
            if sizes[k] < C.FRONTIER_MIN_CLUSTER:
                continue
            ix, iy = np.where(lab == k)
            cx_i = float(ix.mean())
            cy_i = float(iy.mean())
            cx = self.grid.origin[0] + (cx_i + 0.5) * self.grid.res
            cy = self.grid.origin[1] + (cy_i + 0.5) * self.grid.res
            d = math.hypot(cx - rx, cy - ry)
            if d < C.FRONTIER_MIN_DIST:
                continue
            if self._blacklisted(cx, cy, blacklist):
                continue
            candidates.append((cx, cy, d, sizes[k]))

        if not candidates:
            return None

        if not allow_inflated:
            inflated = self.grid.inflated_lethal()
            ok = []
            for cx, cy, d, sz in candidates:
                ix, iy = self.grid.w2i(cx, cy)
                if not self.grid.in_bounds(ix, iy):
                    continue
                if inflated[ix, iy]:
                    continue
                ok.append((cx, cy, d, sz))
            if ok:
                candidates = ok

        best = None
        best_score = -math.inf
        for cx, cy, d, sz in candidates:
            head_err = math.atan2(cy - ry, cx - rx) - rth
            while head_err > math.pi:
                head_err -= 2 * math.pi
            while head_err <= -math.pi:
                head_err += 2 * math.pi
            head_factor = 1.0 - heading_bias * (abs(head_err) / math.pi)
            score = (sz / (1.0 + d)) * head_factor
            if score > best_score:
                best_score = score
                best = (cx, cy)
        return best

    def _blacklisted(self, cx, cy, blacklist):
        for bx, by in blacklist:
            if math.hypot(cx - bx, cy - by) < C.FRONTIER_BLACKLIST_RADIUS:
                return True
        return False

    def _label_fallback(self, mask):
        """Tiny 4-connected component labeller (no scipy)."""
        cells_x, cells_y = mask.shape
        lab = np.zeros(mask.shape, dtype=np.int32)
        nxt = 0
        for x in range(cells_x):
            for y in range(cells_y):
                if mask[x, y] and lab[x, y] == 0:
                    nxt += 1
                    stack = [(x, y)]
                    while stack:
                        a, b = stack.pop()
                        if a < 0 or b < 0 or a >= cells_x or b >= cells_y:
                            continue
                        if not mask[a, b] or lab[a, b] != 0:
                            continue
                        lab[a, b] = nxt
                        stack.extend([(a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)])
        return lab, nxt
