"""Lidar occupancy grid, costmap, poison layer, depth-aux layer, scan matcher.

Pure NumPy (no Webots, no SciPy) so it is fully testable and has no fragile
dependencies.  Index convention throughout: ``arr[ix, iy]`` with axis 0 = world
x (column), axis 1 = world y (row).

Maze4, unlike Maze5, has 4 low wall panels the 2-D lidar partially or fully
misses (see config.py).  This module therefore adds a depth-camera-derived
``aux`` boolean layer ON TOP of the Maze5 lidar-only design.  Critically, the
aux layer is kept deliberately simple: a hit SETS a cell, a clear depth
sight-line CLEARS cells along it, every tick, with NO sticky-hit counters, NO
decay heuristics, and NO "reconcile with lidar walls" radius hack.  That
counter/decay/reconcile design is exactly what caused the documented Maze1 bug
history (HANDOFF.md: "purple thickening", a permanently boxed-in robot) --
raytrace-clearing every tick is the principled fix Maze1's own postmortem
deferred, and is correct from the start here because there is no legacy
behaviour to preserve.
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
    def __init__(self):
        self.res = C.GRID_RESOLUTION
        self.n = C.GRID_CELLS
        self.ox, self.oy = C.GRID_ORIGIN
        self.L = np.zeros((self.n, self.n), dtype=np.float32)
        self.poison = np.zeros((self.n, self.n), dtype=bool)
        # depth-camera auxiliary obstacle layer (NEW for Maze4) -- see module
        # docstring: boolean only, no counters, cleared by raytrace every tick.
        self.aux = np.zeros((self.n, self.n), dtype=bool)

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

    def _ray_cells(self, pose, bearing_body, r0, r1):
        """Flat cell indices along a body-frame bearing from r0 to r1 (world)."""
        if r1 <= r0:
            return np.empty((0,), dtype=np.int64)
        x, y, th = pose
        ang = th + bearing_body
        steps = max(1, int((r1 - r0) / self.res))
        rs = r0 + (r1 - r0) * (np.arange(steps) / float(steps))
        fx = x + rs * math.cos(ang)
        fy = y + rs * math.sin(ang)
        ixs = ((fx - self.ox) / self.res).astype(np.int32)
        iys = ((fy - self.oy) / self.res).astype(np.int32)
        ok = (ixs >= 0) & (ixs < self.n) & (iys >= 0) & (iys < self.n)
        return (ixs[ok].astype(np.int64) * self.n + iys[ok].astype(np.int64))

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

    def integrate_aux(self, pose, bearings, hit_ranges, hit_mask, clear_mask, depth_min):
        """Stamp/raytrace-clear the depth-aux layer from one thin scan.

        Per column: a HIT sets the endpoint cell occupied and clears the ray
        from ``depth_min`` to ``range - AUX_CLEAR_MARGIN``; a CLEAR sight-line
        clears the ray from ``depth_min`` to ``AUX_MAX_RANGE``.  Clears are
        applied before sets so a hit's own partial-clear ray cannot erase its
        own (farther-out) endpoint.  Columns are subsampled for speed; the
        hit/clear classification itself (in ``depth_model.thin_scan``) already
        ran on the full-resolution image.
        """
        n_total = bearings.shape[0]
        if n_total == 0:
            return
        if n_total > C.AUX_COL_SUBSAMPLE:
            idx = np.linspace(0, n_total - 1, C.AUX_COL_SUBSAMPLE).astype(int)
        else:
            idx = np.arange(n_total)

        # The aux layer must hold ONLY obstacles the 2-D lidar is blind to.  The
        # depth camera also sees the chassis band of normal full-height walls the
        # lidar already maps; stamping those duplicates the whole wall map into a
        # lethal layer and boxes the robot in (see config.AUX_LIDAR_BLIND_CELLS).
        # Reconcile against a dilated lidar-occupied mask: a depth hit is kept
        # only where no lidar wall sits within AUX_LIDAR_BLIND_CELLS.
        blind = self.occupied_mask()
        for _ in range(C.AUX_LIDAR_BLIND_CELLS):
            blind = _dilate8(blind)

        clear_lin = []
        occ_lin = []
        n_cells = self.n
        for i in idx:
            if hit_mask[i]:
                rng = float(hit_ranges[i])
                bearing = float(bearings[i])
                clear_to = max(depth_min, rng - C.AUX_CLEAR_MARGIN)
                clear_lin.append(self._ray_cells(pose, bearing, depth_min, clear_to))
                ang = pose[2] + bearing
                ex = pose[0] + rng * math.cos(ang)
                ey = pose[1] + rng * math.sin(ang)
                eix, eiy = self.world_to_grid(ex, ey)
                if self.in_bounds(eix, eiy) and not blind[eix, eiy]:
                    occ_lin.append(eix * n_cells + eiy)
            elif clear_mask[i]:
                bearing = float(bearings[i])
                clear_lin.append(self._ray_cells(pose, bearing, depth_min, C.AUX_MAX_RANGE))

        auxflat = self.aux.reshape(-1)
        if clear_lin:
            nonempty = [c for c in clear_lin if c.size]
            if nonempty:
                c = np.unique(np.concatenate(nonempty))
                auxflat[c] = False
        if occ_lin:
            o = np.unique(np.asarray(occ_lin, dtype=np.int64))
            auxflat[o] = True
        # Retire any EXISTING aux mark the lidar has since confirmed as a normal
        # wall (clears duplicates stamped before the lidar finished mapping that
        # wall -- e.g. everything seen during the INIT_SCAN spin).  Genuine
        # lidar-blind low panels are never in `blind`, so they survive.
        self.aux &= ~blind

    def mark_free_disc(self, wx, wy, radius):
        """Force a small disc around a known-free point to a free log-odds.

        ALSO clears the aux layer in that disc: the robot is physically there,
        so that ground truth overrides any stale aux mark.  This directly
        addresses the Maze1 bug where the robot could box itself in with aux
        marks it could never erase by standing in/spinning through them.  Never
        touches the poison layer (poison is a real hazard regardless of the
        robot's current position).
        """
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        sub = self.L[x0:x1, y0:y1]
        np.minimum(sub, C.L_FREE_THRESH - 0.1, out=sub)
        self.aux[x0:x1, y0:y1] = False

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
        return self._mask_points_near(self.poison, wx, wy, radius)

    def aux_points_near(self, wx, wy, radius):
        """World (M,2) of aux-occupied cell centres within ``radius`` of (wx,wy)."""
        return self._mask_points_near(self.aux, wx, wy, radius)

    def _mask_points_near(self, mask, wx, wy, radius):
        if not mask.any():
            return np.empty((0, 2))
        cix, ciy = self.world_to_grid(wx, wy)
        r = int(math.ceil(radius / self.res))
        x0, x1 = max(0, cix - r), min(self.n, cix + r + 1)
        y0, y1 = max(0, ciy - r), min(self.n, ciy + r + 1)
        sub = mask[x0:x1, y0:y1]
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
        """Float field for scan matching: occupied=1, 1-ring=0.6, 2-ring=0.3.

        Deliberately lidar-only (does NOT include aux): the depth-aux layer is
        noisier and only used for collision avoidance / costmap, never for
        localisation, so it cannot corrupt the scan-matched pose.
        """
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
        live-lidar DWA layer guarantees real wall clearance regardless.  Aux
        (depth-derived low-obstacle) cells are folded into the SAME hard/soft
        obstacle bands as lidar-occupied cells -- both represent "a wall", aux
        is just the one the lidar plane missed.
        """
        occ = self.occupied_mask() | self.aux
        res = self.res

        hard_b = max(1, int(round(C.HARD_OBS_DIST / res)))
        pref_b = max(hard_b + 1, int(round(C.CENTER_PREF_RANGE / res)))
        d_obs = _distance_bands(occ, pref_b + 1)

        lethal = d_obs <= hard_b
        cost = np.zeros((self.n, self.n), dtype=np.float32)
        # center-seeking gradient: cost falls off with clearance out to pref_b so
        # A* rides the medial axis (maximum clearance) of every corridor rather
        # than hugging the first non-lethal cell.  This is what aligns the path
        # to the absolute centre of the tight 0.39 m gaps.  (Replaces the old
        # fixed-width soft halo, which went flat in the middle of a narrow gap
        # and so gave A* no reason to prefer dead-centre.)
        grad_zone = (d_obs > hard_b) & (d_obs <= pref_b)
        frac = (pref_b - d_obs.astype(np.float32)) / max(1, (pref_b - hard_b))
        cost[grad_zone] += C.COST_OBS_WEIGHT * frac[grad_zone]

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
