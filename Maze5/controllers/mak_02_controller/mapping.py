"""Lidar occupancy grid, costmap, poison layer and a light scan matcher.

Pure NumPy (no Webots, no SciPy) so it is fully testable and has no fragile
dependencies.  Index convention throughout: ``arr[ix, iy]`` with axis 0 = world
x (column), axis 1 = world y (row).

Maze5 has no floating walls (every wall crosses the 2-D lidar plane), so a clean
lidar-only log-odds grid is enough — there is deliberately no depth "aux" layer.
"""
from __future__ import annotations

import math

import numpy as np

import config as C
from geometry import transform_points, wrap_angle


# --------------------------------------------------------------------------- #
#  Lidar range image -> body-frame points
# --------------------------------------------------------------------------- #
class LidarModel:
    """Beam ``i`` points at ``theta_i = fov/2 - i*fov/(N-1)`` (Webots convention).

    The lidar sits on the robot's z-axis, so its x/y frame == the body frame.
    """

    def __init__(self, n_beams, fov, r_min=None, r_max=None):
        self.n = int(n_beams)
        self.fov = float(fov)
        self.r_min = C.LIDAR_RANGE_MIN if r_min is None else max(r_min, C.LIDAR_RANGE_MIN)
        self.r_max = C.LIDAR_RANGE_MAX if r_max is None else min(r_max, 30.0)
        start = self.fov * 0.5
        step = self.fov / max(self.n - 1, 1)
        self.angles = start - step * np.arange(self.n)
        self._cos = np.cos(self.angles)
        self._sin = np.sin(self.angles)

    def ranges_to_body(self, ranges):
        """Return ``(pts_body (M,2), ranges_valid (M,))`` for in-range beams."""
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.size != self.n:
            return np.empty((0, 2)), np.empty((0,))
        valid = np.isfinite(ranges) & (ranges > self.r_min) & (ranges < self.r_max)
        r = ranges[valid]
        x = r * self._cos[valid]
        y = r * self._sin[valid]
        return np.stack([x, y], axis=1), r


# --------------------------------------------------------------------------- #
#  8-connected dilation helpers (used for inflation + scan-match field)
# --------------------------------------------------------------------------- #
def _dilate8(mask):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def _distance_bands(mask, max_band):
    """Approx. distance (in cells, Chebyshev) from each cell to nearest True."""
    dist = np.full(mask.shape, max_band + 1, dtype=np.int16)
    cur = mask.copy()
    if not cur.any():
        return dist
    dist[cur] = 0
    for k in range(1, max_band + 1):
        nxt = _dilate8(cur)
        newly = nxt & ~cur
        if newly.any():
            dist[newly] = k
        cur = nxt
        if cur.all():
            break
    return dist


# --------------------------------------------------------------------------- #
#  Occupancy grid
# --------------------------------------------------------------------------- #
class OccupancyGrid:
    """2-D log-odds occupancy map with poison/auxiliary obstacle layers, costmap construction and lidar scan-matching."""
    def __init__(self):
        self.res = C.GRID_RESOLUTION
        self.n = C.GRID_CELLS
        self.ox, self.oy = C.GRID_ORIGIN
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.poison = np.zeros((self.n, self.n), dtype=bool)

    # ----------------------------------------------------- coordinate maths
    def world_to_grid(self, wx, wy):
        ix = int((wx - self.ox) / self.res)
        iy = int((wy - self.oy) / self.res)
        return ix, iy

    def world_to_grid_arr(self, wx, wy):
        ix = ((np.asarray(wx) - self.ox) / self.res).astype(np.int32)
        iy = ((np.asarray(wy) - self.oy) / self.res).astype(np.int32)
        return ix, iy

    def grid_to_world(self, ix, iy):
        return (self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res)

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.n and 0 <= iy < self.n

    def _inb_arr(self, ix, iy):
        return (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)

    # ----------------------------------------------------------- integration
    def integrate_scan(self, pose, pts_body, ranges):
        """Ray-trace free space and stamp occupied endpoints from one scan."""
        if pts_body.shape[0] == 0:
            return
        x, y, th = pose
        pts_world = transform_points(pts_body, x, y, th)
        rix, riy = self.world_to_grid(x, y)

        free_lin = []
        occ_lin = []
        n_cells = self.n
        for k in range(pts_world.shape[0]):
            ex, ey = pts_world[k, 0], pts_world[k, 1]
            rng = ranges[k]
            steps = int(rng / self.res)
            if steps > 1:
                ts = np.arange(steps) / float(steps)  # 0..(steps-1)/steps, excl. endpoint
                fx = x + (ex - x) * ts
                fy = y + (ey - y) * ts
                ixs = ((fx - self.ox) / self.res).astype(np.int32)
                iys = ((fy - self.oy) / self.res).astype(np.int32)
                ok = (ixs >= 0) & (ixs < n_cells) & (iys >= 0) & (iys < n_cells)
                free_lin.append(ixs[ok] * n_cells + iys[ok])
            if rng < C.LIDAR_MAX_INTEGRATE:
                eix, eiy = self.world_to_grid(ex, ey)
                if self.in_bounds(eix, eiy):
                    occ_lin.append(eix * n_cells + eiy)

        Lflat = self.L.reshape(-1)
        if free_lin:
            f = np.unique(np.concatenate(free_lin))
            np.add.at(Lflat, f, C.L_FREE)
        if occ_lin:
            o = np.unique(np.asarray(occ_lin, dtype=np.int64))
            np.add.at(Lflat, o, C.L_OCC)
        np.clip(self.L, C.L_MIN, C.L_MAX, out=self.L)

    def mark_free_disc(self, wx, wy, radius):
        """Force a small disc around a known-free point to a free log-odds.

        Keeps the robot's own cell plannable even if a stray hit landed on it.
        Never touches the poison layer.
        """
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        sub = self.L[x0:x1, y0:y1]
        np.minimum(sub, C.L_FREE_THRESH - 0.1, out=sub)

    def add_poison_points(self, pts_world):
        """Stamp camera-projected green floor points into the sticky poison layer."""
        if pts_world.shape[0] == 0:
            return
        ix, iy = self.world_to_grid_arr(pts_world[:, 0], pts_world[:, 1])
        ok = self._inb_arr(ix, iy)
        self.poison[ix[ok], iy[ok]] = True

    # --------------------------------------------------------------- masks
    def occupied_mask(self):
        return self.L > C.L_OCC_THRESH

    def free_mask(self):
        return self.L < C.L_FREE_THRESH

    def unknown_mask(self):
        return (~self.occupied_mask()) & (~self.free_mask())

    # ------------------------------------------------- mapped obstacles query
    def poison_points_near(self, wx, wy, radius):
        """World (M,2) of poison-cell centres within ``radius`` of (wx,wy)."""
        if not self.poison.any():
            return np.empty((0, 2))
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        sub = self.poison[x0:x1, y0:y1]
        lx, ly = np.nonzero(sub)
        if lx.size == 0:
            return np.empty((0, 2))
        wx_ = self.ox + (x0 + lx + 0.5) * self.res
        wy_ = self.oy + (y0 + ly + 0.5) * self.res
        return np.stack([wx_, wy_], axis=1)

    def is_poison_world(self, wx, wy):
        ix, iy = self.world_to_grid(wx, wy)
        return self.in_bounds(ix, iy) and bool(self.poison[ix, iy])

    # ------------------------------------------------------------ scan match
    def _score_field(self):
        """Float field for scan matching: occupied=1, 1-ring=0.6, 2-ring=0.3."""
        occ = self.occupied_mask()
        field = np.zeros((self.n, self.n), dtype=np.float32)
        field[occ] = 1.0
        d1 = _dilate8(occ) & ~occ
        field[d1] = 0.6
        d2 = _dilate8(_dilate8(occ)) & ~_dilate8(occ)
        field[d2] = 0.3
        return field

    def scan_match(self, pred_pose, pts_body):
        """Correlative (x,y,yaw) correction of ``pred_pose`` against the map.

        Returns ``(corrected_pose, hit_fraction)``.  Falls back to ``pred_pose``
        when too few beams land on mapped walls (avoids corrupting a good odom).
        """
        m = pts_body.shape[0]
        if m == 0 or not self.occupied_mask().any():
            return pred_pose, 0.0
        if m > C.SM_MAX_BEAMS:
            idx = np.linspace(0, m - 1, C.SM_MAX_BEAMS).astype(int)
            pts = pts_body[idx]
        else:
            pts = pts_body
        field = self._score_field()
        px, py, pth = pred_pose

        lin = np.arange(-C.SM_LIN_HALF, C.SM_LIN_HALF + 1e-9, C.SM_LIN_STEP)
        ang = np.arange(-C.SM_ANG_HALF, C.SM_ANG_HALF + 1e-9, C.SM_ANG_STEP)
        best = (-1.0, 0.0, 0.0, 0.0)
        bx, by = pts[:, 0], pts[:, 1]
        for dth in ang:
            c, s = math.cos(pth + dth), math.sin(pth + dth)
            wx0 = px + c * bx - s * by
            wy0 = py + s * bx + c * by
            for dx in lin:
                wx = wx0 + dx
                for dy in lin:
                    wy = wy0 + dy
                    ix = ((wx - self.ox) / self.res).astype(np.int32)
                    iy = ((wy - self.oy) / self.res).astype(np.int32)
                    ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
                    if not ok.any():
                        continue
                    sc = field[ix[ok], iy[ok]].sum()
                    if sc > best[0]:
                        best = (sc, dx, dy, dth)
        score, dx, dy, dth = best
        hit_frac = score / max(m, 1)
        if hit_frac < C.SM_MIN_HIT_FRAC:
            return pred_pose, hit_frac
        return (px + dx, py + dy, wrap_angle(pth + dth)), hit_frac

    # -------------------------------------------------------------- costmap
    def build_costmap(self):
        """Return ``(cost float32, lethal bool)`` over the whole grid.

        ``cost`` is additive A* penalty per cell (0 in open free space).  Unknown
        cells are traversable (optimistic) so frontier goals stay reachable; the
        live-lidar DWA layer guarantees real wall clearance regardless.
        """
        occ = self.occupied_mask()
        res = self.res

        hard_b = max(1, int(round(C.HARD_OBS_DIST / res)))
        soft_b = max(hard_b, int(round((C.HARD_OBS_DIST + C.SOFT_OBS_HALO) / res)))
        d_obs = _distance_bands(occ, soft_b + 1)

        lethal = d_obs <= hard_b
        cost = np.zeros((self.n, self.n), dtype=np.float32)
        soft_zone = (d_obs > hard_b) & (d_obs <= soft_b)
        # exponential-ish decay across the soft halo
        frac = (soft_b - d_obs.astype(np.float32)) / max(1, (soft_b - hard_b))
        cost[soft_zone] += C.COST_OBS_WEIGHT * frac[soft_zone]

        if self.poison.any():
            ph = max(1, int(round(C.POISON_HARD_DIST / res)))
            ps = max(ph, int(round((C.POISON_HARD_DIST + C.POISON_SOFT_HALO) / res)))
            d_pois = _distance_bands(self.poison, ps + 1)
            lethal |= d_pois <= ph
            pz = (d_pois > ph) & (d_pois <= ps)
            pfrac = (ps - d_pois.astype(np.float32)) / max(1, (ps - ph))
            cost[pz] += C.COST_POISON_WEIGHT * pfrac[pz]

        cost[lethal] = np.inf
        return cost, lethal
